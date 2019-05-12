#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <novel_msgs/NovelObject.h>
#include <novel_msgs/NovelObjectArray.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <queue>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <cstdlib>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class LIDAR_Action {
  public:
    LIDAR_Action();
    void approach();

  private:
    void push_to_queue(const novel_msgs::NovelObjectArray::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void state_callback(const std_msgs::Int8::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber det_sub;
    ros::Subscriber map_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub;

    ros::Publisher map_pub;
    ros::Publisher state_pub;

    tf::TransformListener listener;

    nav_msgs::MapMetaData map_metadata;
    float map_resolution;
    std::vector<signed char, std::allocator<signed char> > grid;
    
    geometry_msgs::PoseWithCovariance pose;
    
    bool map_known;
    bool pose_known;
    bool on;
        
    double min_marker_det_dist;

    std::queue< std::vector<double> > LIDAR_candidates;
};

LIDAR_Action::LIDAR_Action() {
  map_known = false;
  pose_known = false;
  on = true;
  min_marker_det_dist = .75;

  det_sub = nh_.subscribe<novel_msgs::NovelObjectArray>("candidates", 1000, &LIDAR_Action::push_to_queue, this);
  map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1, &LIDAR_Action::map_callback, this);
  pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &LIDAR_Action::pose_callback, this);

  map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  state_pub = nh_.advertise<std_msgs::Int8>("state", 1);
}

/*
Changes state value to indicate progression of finite state machine
0 - move based on LIDAR detected objects
1 - detect using kinect
2 - move based on kinect detected markers

Input
-----
msg: integer message

Output
------
*/
void LIDAR_Action::state_callback(const std_msgs::Int8::ConstPtr& msg) {
  if (msg->data == 0) {
    on = true;
  } else {
    on = false;
  }
}

/*
Pushes LIDAR detected objects into queue

Input
-----
msg: array of novel objects detected

Output
------
*/
void LIDAR_Action::push_to_queue(const novel_msgs::NovelObjectArray::ConstPtr& msg) {
  // convert Point msg to grid coordinate wrt map frame
  for (int i = 0; i < msg->detected_objects.size(); i++) {
    
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("/map", "/base_scan", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
 
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 R(q);
    tf::Vector3 t(x,y,z);

    double x_obj = msg->detected_objects[i].pose.pose.position.x;
    double y_obj = msg->detected_objects[i].pose.pose.position.y;
    double z_obj = msg->detected_objects[i].pose.pose.position.z;
    tf::Vector3 obj(x_obj, y_obj, z_obj);

    tf::Vector3 obj_in_map_frame = R * obj + t;
    x = obj_in_map_frame.getX();
    y = obj_in_map_frame.getY();
    /*
    double x = msg->detected_objects[i].pose.pose.position.x;
    double y = msg->detected_objects[i].pose.pose.position.y;
    */
    int x_coord = (int)round((x - map_metadata.origin.position.x)/map_resolution);
    int y_coord = (int)round((y - map_metadata.origin.position.y)/map_resolution);

    std::vector<double> coord = {x, y};
    LIDAR_candidates.push(coord);
  }

  if (msg->detected_objects.size() > 0) {
    approach();
  }

  ROS_INFO_STREAM(LIDAR_candidates.size());
}

/*
Determines action to execute based on LIDAR objs detected

Inputs
------
msg: published NovelObjectArray

Output
------

*/
void LIDAR_Action::approach() {
  // if we know pose of robot, plan is not currently executing, and msg of detected ids came after plan finished
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }

  // ROS_INFO_STREAM(LIDAR_candidates.size());
  if (pose_known && on) {
    while (!LIDAR_candidates.empty()) {
      std::vector<double> loc = LIDAR_candidates.front();      

      double x_obj = loc[0];
      double y_obj = loc[1];

      // get robot coordinates with respect to map frame
      tf::StampedTransform transform;
      try {
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      double x = transform.getOrigin().x(); // robot coordinate in map frame
      double y = transform.getOrigin().y();
      int robot_x = (int)round( (x-map_metadata.origin.position.x)/map_resolution); // robot map grid coordinate
      int robot_y = (int)round( (y-map_metadata.origin.position.y)/map_resolution);
      double angle = atan2(y_obj - y, x_obj - x);

      // tell robot to move within __ meters of LIDAR-detected obj
      // turn on kinect and see if marker is detected
      //   if detected, kinect_action node will move robot
      //   if not detected, tell robot to move in a circle around object for better view of potential marker
      bool detect = false;
      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header.frame_id = "map"; // goal is relative to this frame
      
      int i = 0;
      while (!detect && i < 8) {
        goal.target_pose.pose.position.x = x_obj - min_marker_det_dist * cos(angle);
        goal.target_pose.pose.position.y = y_obj - min_marker_det_dist * sin(angle);
        
        // TODO: check that path between goal and candidate obj is unobstructed

        goal.target_pose.pose.orientation.z = sin(angle/2.0);
        goal.target_pose.pose.orientation.w = cos(angle/2.0);
        goal.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          // start kinect detection
          std_msgs::Int8 kinect_detect;
          kinect_detect.data = 1;
          state_pub.publish(kinect_detect);
          
          ROS_INFO_STREAM("Detecting marker");
          boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> shared_arr;
          ar_track_alvar_msgs::AlvarMarkers arr;
          shared_arr = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", nh_);
          if (shared_arr != NULL) {
            arr = *shared_arr;
            if (arr.markers.size() > 0) {
              ROS_INFO_STREAM("Marker detected");

              // wait till message received that action is done
              boost::shared_ptr<std_msgs::Int8 const> shared_int;
              std_msgs::Int8 integer;
              shared_int = ros::topic::waitForMessage<std_msgs::Int8>("moved", nh_);

              integer = *shared_int;
              if (integer.data == 1) {
                detect = true;
                ROS_INFO_STREAM("Done with marker action");
              } else {
                ROS_INFO_STREAM("Not the marker associated with the object detected.");
              }
            } else {
              ROS_INFO_STREAM("Marker not detected.");
            }
          }
        } else {
          ROS_INFO_STREAM("Could not approach LIDAR-detected object. Skipping LIDAR action.");
        }

        angle = angle + 3.1415/4;
        i = i + 1;
      }

      LIDAR_candidates.pop();
    }
  }
}

/*
Stores map once it is received

Input
-----
msg: occupancy grid map message

Output
------

*/
void LIDAR_Action::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  map_metadata = msg->info;
  grid = msg->data;
  map_resolution = msg->info.resolution;
  map_known = true;
}

/*
Stores pose of robot

Input
-----
msg: pose with covariance of robot sent by amcl

Output
------
*/
void LIDAR_Action::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  pose_known = true;
  pose = msg->pose;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "action");

  LIDAR_Action lidar_action;
  
  ros::spin();
}
