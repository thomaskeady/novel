#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <novel_msgs/NovelObject.h>
#include <novel_msgs/NovelObjectArray.h>
#include <queue>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Action {
  public:
    Action();
    void sendGoals(int x, int y);

  private:
    void det_callback(const novel_msgs::NovelObjectArray::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber det_sub;
    ros::Subscriber map_sub;
    std::queue< std::vector<int> > queue_;

    nav_msgs::MapMetaData map_metadata;
    float map_resolution;
    std::vector<signed char, std::allocator<signed char> > grid;
    bool map_known;
    bool execute_plan;
};

Action::Action() {
  map_resolution = 0.05;
  map_known = false;
  execute_plan = false;

  det_sub = nh_.subscribe<novel_msgs::NovelObjectArray>("detected", 1, &Action::det_callback, this);
  map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Action::map_callback, this);
}

void Action::det_callback(const novel_msgs::NovelObjectArray::ConstPtr& msg) {
  std::vector<int> indices;

  int xsum = 0;
  int ysum = 0;
  for (int i = 0; i < msg->detected_objects.size(); i++) {
    // push back into queue
    int x_coord = (int)round((msg->detected_objects[i].pose.pose.position.x-map_metadata.origin.position.x)/map_resolution);
    int y_coord = (int)round((msg->detected_objects[i].pose.pose.position.y-map_metadata.origin.position.y)/map_resolution);

    xsum = xsum + x_coord;
    ysum = ysum + y_coord;

    // int index = (y_coord)*map_metadata.width + x_coord;
  }

  int x_mean = (int)round( xsum / msg->detected_objects.size() );
  int y_mean = (int)round( ysum / msg->detected_objects.size() );
  sendGoals(x_mean, y_mean);
}

void Action::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  map_metadata = msg->info;
  grid = msg->data;
  ROS_INFO_STREAM(map_metadata.width);
  ROS_INFO_STREAM(map_metadata.height);
  map_known = true;
}

void Action::sendGoals(int x, int y) {
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }

  if (map_known && !execute_plan) {
    execute_plan = true;
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link"; // "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // pull coordinates from queue
    

    // Test
    goal.target_pose.pose.position.x = -0.5;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.orientation.w = 1;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();
    execute_plan = false;

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved");
    else
      ROS_INFO("The base failed to move for some reason");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "action");

  Action action;
  
  ros::spin();
  return 0;
}
