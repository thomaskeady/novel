// Adapted from aruco_detect package

#include <ros/ros.h>
//#include <tf/transform_datatypes.h>
//#include <tf2/LinearMath/Transform.h>
//#include <tf2_ros/buffer.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <dynamic_reconfigure/server.h>
//#include <std_srvs/SetBool.h>

#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

//#include <list>
//#include <string>
//#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cv;

class DetectNode {
  private:
    ros::Publisher * vertices_pub;
    ros::Publisher * pose_pub;

    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;

    // if set, we publish the images that contain fiducials
    bool publish_images;
    bool enable_detections;

    double fiducial_len;

    bool doPoseEstimation;
    bool haveCamInfo;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    int frameNum;
    std::string frameId;
    std::map<int, double> fiducialLens;

    image_transport::Publisher image_pub;

    cv::Ptr<aruco::Dictionary> dictionary;

    void estimatePoseSingleMarkers(const vector<int> &ids,
                                   const vector<vector<Point2f > >&corners,
                                   float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                   vector<double>& reprojectionError);


    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  public:
    DetectNode(ros::NodeHandle &nh);
};

/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const Point2f &p0 = pts.at(0);
    const Point2f &p1 = pts.at(1);
    const Point2f &p2 = pts.at(2);
    const Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

// estimate reprojection error
static double getReprojectionError(const vector<Point3f> &objectPoints,
                            const vector<Point2f> &imagePoints,
                            const Mat &cameraMatrix, const Mat  &distCoeffs,
                            const Vec3d &rvec, const Vec3d &tvec) {

    vector<Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}

void DetectNode::estimatePoseSingleMarkers(const vector<int> &ids,
                                const vector<vector<Point2f > >&corners,
                                float markerLength,
                                const cv::Mat &cameraMatrix,
                                const cv::Mat &distCoeffs,
                                vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    vector<Point3f> markerObjPoints;
    int nMarkers = (int)corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
       double fiducialSize = markerLength;

       std::map<int, double>::iterator it = fiducialLens.find(ids[i]);

       if (it != fiducialLens.end()) {
          fiducialSize = it->second;
       }

       getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);
       cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                    rvecs[i], tvecs[i]);

       reprojectionError[i] =
          getReprojectionError(markerObjPoints, corners[i],
                               cameraMatrix, distCoeffs,
                               rvecs[i], tvecs[i]);
    }
}

void DetectNode::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    // ROS_INFO("Got image %d", msg->header.seq);
    frameNum++;

    cv_bridge::CvImagePtr cv_ptr;

    fiducial_msgs::FiducialTransformArray fta;
    fta.header.stamp = msg->header.stamp;
    fta.header.frame_id = frameId;
    fta.image_seq = msg->header.seq;

    fiducial_msgs::FiducialArray fva;
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id =frameId;
    fva.image_seq = msg->header.seq;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        vector <int>  ids;
        vector <vector <Point2f> > corners, rejected;
        vector <Vec3d>  rvecs, tvecs;

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);
        ROS_INFO("Detected %d markers", (int)ids.size());

        for (size_t i=0; i<ids.size(); i++) {
            fiducial_msgs::Fiducial fid;
            fid.fiducial_id = ids[i];

            fid.x0 = corners[i][0].x;
            fid.y0 = corners[i][0].y;
            fid.x1 = corners[i][1].x;
            fid.y1 = corners[i][1].y;
            fid.x2 = corners[i][2].x;
            fid.y2 = corners[i][2].y;
            fid.x3 = corners[i][3].x;
            fid.y3 = corners[i][3].y;
            fva.fiducials.push_back(fid);
        }

        vertices_pub->publish(fva);

        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        if (doPoseEstimation) {
            vector <double>reprojectionError;

            estimatePoseSingleMarkers(ids, corners, (float)fiducial_len,
                                      cameraMatrix, distortionCoeffs,
                                      rvecs, tvecs,
                                      reprojectionError);

            for (size_t i=0; i<ids.size(); i++) {
                aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                rvecs[i], tvecs[i], (float)fiducial_len);

                ROS_INFO("Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                         tvecs[i][0], tvecs[i][1], tvecs[i][2],
                         rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                double angle = norm(rvecs[i]);
                Vec3d axis = rvecs[i] / angle;
                ROS_INFO("axis %f %f %f", axis[0], axis[1], axis[2]);

                fiducial_msgs::FiducialTransform ft;
                ft.fiducial_id = ids[i];

                ft.transform.translation.x = tvecs[i][0];
                ft.transform.translation.y = tvecs[i][1];
                ft.transform.translation.z = tvecs[i][2];

                tf2::Quaternion q;
                q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                ft.transform.rotation.w = q.w();
                ft.transform.rotation.x = q.x();
                ft.transform.rotation.y = q.y();
                ft.transform.rotation.z = q.z();

                ft.fiducial_area = calcFiducialArea(corners[i]);
                ft.image_error = reprojectionError[i];

                // Convert image_error (in pixels) to object_error (in meters)
                ft.object_error =
                    (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                    (norm(tvecs[i]) / fiducial_len);

                fta.transforms.push_back(ft);
            }
            pose_pub->publish(fta);
        }

        if (publish_images) {
	    image_pub.publish(cv_ptr->toImageMsg());
        }
    }
    catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception & e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}

DetectNode::DetectNode(ros::NodeHandle & nh) : it(nh)
{
    frameNum = 0;

    // Camera intrinsics
    cameraMatrix = Mat::zeros(3,3,CV_64F);
    cameraMatrix.at<double>(0,0) = 638.237881;
    cameraMatrix.at<double>(0,1) = 0.0;
    cameraMatrix.at<double>(0,2) = 315.384191;
    cameraMatrix.at<double>(1,0) = 0.0;
    cameraMatrix.at<double>(1,1) = 648.051925;
    cameraMatrix.at<double>(1,2) = 255.573930;
    cameraMatrix.at<double>(2,0) = 0.0;
    cameraMatrix.at<double>(2,1) = 0.0;
    cameraMatrix.at<double>(2,2) = 1.0;
    // cameraMatrix << 638.237881, 0.0, 315.384191, 0.0, 648.051925, 255.573930, 0.0, 0.0, 1.0;
    cout << cameraMatrix << endl;    

    // distortion coefficients
    distortionCoeffs = Mat::zeros(1,5,CV_64F);
    distortionCoeffs.at<double>(0,0) = 0.111137;
    distortionCoeffs.at<double>(0,1) = -0.541529;
    distortionCoeffs.at<double>(0,2) = 0.003274;
    distortionCoeffs.at<double>(0,3) = -0.009978;
    distortionCoeffs.at<double>(0,4) = 0.0;

    // distortionCoeffs << 0.111137, -0.541529, 0.003274, -0.009978, 0.0;

    haveCamInfo = true;
    enable_detections = true;
    publish_images = true;
    fiducial_len = 0.05; // TODO: need to adjust
    doPoseEstimation = true;

    std::string str;
    std::vector<std::string> strs;

    image_pub = it.advertise("/fiducial_images", 1);

    vertices_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialArray>("/fiducial_vertices", 1));

    pose_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("/fiducial_transforms", 1));

    dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

    img_sub = it.subscribe("/camera", 1,
                        &DetectNode::imageCallback, this);

    ROS_INFO("Aruco detection ready");
}


int main(int argc, char ** argv) {
  ros::init(argc, argv, "realsense_aruco_test");
  ros::NodeHandle nh("~");

  DetectNode * node = new DetectNode(nh);

  ros::spin();
  return 0;
}

