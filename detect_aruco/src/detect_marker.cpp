#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <novel_msgs/NovelObject.h>
#include <novel_msgs/NovelObjectArray.h>

class Alvar {
  public:
    Alvar();

  private:
    void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    ros::NodeHandle nh_;

    ros::Publisher pub;
    ros::Subscriber sub;
};

Alvar::Alvar() {
  pub = nh_.advertise<novel_msgs::NovelObjectArray>("detected", 1000);
  sub = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1000, &Alvar::callback, this);

}

void Alvar::callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  ROS_INFO_STREAM(msg->markers.size());
  if (msg->markers.size() > 0) {
    novel_msgs::NovelObjectArray arr;
    arr.header.stamp = ros::Time::now();

    for (int i = 0; i < msg->markers.size(); i++) {
      novel_msgs::NovelObject obj;
      obj.pose.pose = msg->markers[i].pose.pose;
      obj.pose.covariance[0] = 0.005; // in meters
      obj.pose.covariance[7] = 0.005;
      obj.pose.covariance[14] = 0.035;
      obj.pose.covariance[21] = 0.0523599; // approx 3 deg
      obj.pose.covariance[28] = 0.0523599; // approx 3 deg
      obj.pose.covariance[35] = 0.0349066;  // approx 2 deg
      obj.classification = std::to_string(msg->markers[i].id);
      arr.detected_objects.push_back(obj);
    }

    pub.publish(arr);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "novel_msg_publisher");

  Alvar alvar;

  ros::spin();
  return 0;
}

