#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <novel_msgs/NovelObject.h>
#include <novel_msgs/NovelObjectArray.h>
#include <queue>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <cstdlib>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Action {
  public:
    Action();
    void runAndRotate(int x_rot, int y_rot, int x_run, int y_run);
    void rotateForBetterView(int x_rot, int y_rot);
    void runAway(int x_run, int y_run);
    void constructGrid(int marker_x, int marker_y, int robot_x, int robot_y, std::vector<std::vector<double>> &weights);
    double calcDistance(double x1, double y1, double x2, double y2);

  private:
    void det_callback(const novel_msgs::NovelObjectArray::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber det_sub;
    ros::Subscriber map_sub;
    ros::Subscriber pose_sub;

    nav_msgs::MapMetaData map_metadata;
    float map_resolution;
    std::vector<signed char, std::allocator<signed char> > grid;
    geometry_msgs::PoseWithCovariance pose;
    bool map_known;
    bool execute_plan;
    bool pose_known;
    std::vector<string> prev;
    std::vector<string> run;
    std::vector<string> rotate;
};

Action::Action() {
  map_known = false;
  pose_known = false;
  execute_plan = false;

  det_sub = nh_.subscribe<novel_msgs::NovelObjectArray>("detected", 1, &Action::det_callback, this);
  map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Action::map_callback, this);
  pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &Action::pose_callback, this);

  run = {"0", "2", "4", "6", "8", "10", "12", "14", "16"};
  rotate = {"1", "3", "5", "7", "9", "11", "13", "15", "17"};
}

void Action::det_callback(const novel_msgs::NovelObjectArray::ConstPtr& msg) {
  if (pose_known && !execute_plan) {
    // det_sub.shutdown();
   
    int x_run = 0;
    int y_run = 0;
    int x_rot = 0;
    int y_rot = 0;
    int run_count = 0;
    int rot_count = 0;

    std::vector<string> detected_ids;
    
    bool should_run = false;
    bool should_rotate = false;
    bool unknown = false;
    
    for (int i = 0; i < msg->detected_objects.size(); i++) {
      // convert marker position world coordinate in terms of map grid coordinate
      int x_coord = (int)round((msg->detected_objects[i].pose.pose.position.x + pose.pose.position.x - map_metadata.origin.position.x)/map_resolution);
      int y_coord = (int)round((msg->detected_objects[i].pose.pose.position.y + pose.pose.position.y - map_metadata.origin.position.y)/map_resolution);

      xsum = xsum + x_coord;
      ysum = ysum + y_coord;

      string label = msg->detected_objects[i].classification;
      if (std::find(run.begin(), run.end(), label) != run.end()) {
        should_run = true;
        x_run = x_run + x_coord;
        y_run = y_run + y_coord;
        run_count = run_count + 1;
      } else if (std::find(rotate.begin(), rotate.end(), label) != rotate.end()) {
        should_rotate = true;
        x_rot = x_rot + x_coord;
        y_rot = y_rot + y_coord;
        rot_count = rot_count + 1;
      } else {
        unknown = true;
      }
      detected_ids.push_back(label);
    }

    // average position of detected markers
    // kinect field of view is not very wide so detected markers are probably not very far apart
    // so for convenience sake, let's clump them together as one marker at their average position
    std::sort(detected_ids.begin(), detected_ids.end());
    if (detected_ids != prev) {
      prev = detected_ids;
      if (unknown) {
        ROS_INFO("Unknown object detected. Not sure what to do.");
      } else if (run && rotate) {
        x_run = (int)round( (double)x_run / run_count );
        y_run = (int)round( (double)y_run / run_count );
        x_rot = (int)round( (double)x_rot / rot_count );
        y_rot = (int)round( (double)y_rot / rot_count );
        runAndRotate(x_run, y_run, x_rot, y_rot);
      } else if (run) {
        x_run = (int)round( (double)x_run / run_count );
        y_run = (int)round( (double)y_run / run_count );
        runAway(x_run, y_run);
      } else {
        x_rot = (int)round( (double)x_rot / rot_count );
        y_rot = (int)round( (double)y_rot / rot_count );
        rotateForBetterView(x_rot, y_rot);
      }
    }
  }
}

void Action::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  map_metadata = msg->info;
  grid = msg->data;
  map_resolution = msg->info.resolution;
  map_known = true;
}

void Action::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  pose_known = true;
  pose = msg->pose;
}

void Action::runAway(int x_run, int y_run) {
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }

  ROS_INFO("Check if plan is in progress now");

  // if we know what map looks like, where robot is, and a plan is not currently executing
  if (map_known && pose_known && !execute_plan) {
    execute_plan = true; // start plan
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map"; // goal is relative to this frame
    goal.target_pose.header.stamp = ros::Time::now();

    // pull coordinates from queue
    double x = pose.pose.position.x; // robot world coordinate
    double y = pose.pose.position.y;
    int robot_x = (int)round( (x-map_metadata.origin.position.x)/map_resolution); // robot map grid coordinate
    int robot_y = (int)round( (y-map_metadata.origin.position.y)/map_resolution);

    std::vector<std::vector<double>> weights( map_metadata.height, std::vector<double>(map_metadata.width, 0) ); // weights to sample goal position
    constructGrid(x_run, y_run, robot_x, robot_y, weights);

    // generate random number to pick grid index, convert to world coordinates
    double weight_total = 0;
    for (int i = 0; i < map_metadata.height; i++) {
      for (int j = 0; j < map_metadata.width; j++) {
         weight_total = weight_total + weights[i][j];
      }
    }    

    while (execute_plan) {
      double random = (double)rand() / RAND_MAX;
      // ROS_INFO_STREAM(random);
      random = random * weight_total;

      int index = 0;
      while (random > 0) {
        random = random - weights[index / map_metadata.width][index % map_metadata.width];
        index = index + 1;
      }
      index = index - 1;

      // goal in terms of map grid coordinate
      int goal_x = index % map_metadata.width; // column
      int goal_y = index / map_metadata.width; // row

      // convert goal position to be in world coordinate relative to origin of map
      goal.target_pose.pose.position.x = goal_x * map_resolution + map_metadata.origin.position.x;
      goal.target_pose.pose.position.y = goal_y * map_resolution + map_metadata.origin.position.y;
      goal.target_pose.pose.orientation.w = 1;

      ROS_INFO_STREAM(goal.target_pose.pose.position.x);
      ROS_INFO_STREAM(goal.target_pose.pose.position.y);

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved");
        execute_plan = false;
        prev.clear();
        // det_sub = nh_.subscribe<novel_msgs::NovelObjectArray>("detected", 1, &Action::det_callback, this);
      } else
        ROS_INFO("The base failed to move for some reason. We will sample new goal point.");
    }
  }
}

void Action::rotateForBetterView(int x_rot, int y_rot) {
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }

  ROS_INFO("Check if plan is in progress now");

  // if we know what map looks like, where robot is, and a plan is not currently executing
  if (map_known && pose_known && !execute_plan) {
    execute_plan = true; // start plan
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map"; // goal is relative to this frame
    goal.target_pose.header.stamp = ros::Time::now();

    // pull coordinates from queue
    double x = pose.pose.position.x; // robot world coordinate
    double y = pose.pose.position.y;
    int robot_x = (int)round( (x-map_metadata.origin.position.x)/map_resolution); // robot map grid coordinate
    int robot_y = (int)round( (y-map_metadata.origin.position.y)/map_resolution);

    double grid_dist = sqrt( pow(robot_x - x_rot, 2) + pow(robot_y - y_rot, 2) );
    double dist_m = grid_dist * map_resolution;

    double angle = atan2(robot_y - y_rot, robot_x - x_rot);
    double dist = 0.5;
    while (execute_plan) {
      // goal in terms of map grid coordinate
      int goal_x = index % map_metadata.width; // column
      int goal_y = index / map_metadata.width; // row

      // convert goal position to be in world coordinate relative to origin of map
      if (dist < dist_m) {
        goal.target_pose.pose.position.x = x_rot * map_resolution + map_metadata.origin.position.x - dist*cos(angle);
        goal.target_pose.pose.position.y = y_rot * map_resolution + map_metadata.origin.position.y - dist*sin(angle);
        goal.target_pose.pose.orientation.z = angle;
        goal.target_pose.pose.orientation.w = 1;
      } else {
        goal.target_pose.pose.position.x = robot_x * map_resolution + map_metadata.origin.position.x;
        goal.target_pose.pose.position.y = robot_y * map_resolution + map_metadata.origin.position.y;
        goal.target_pose.pose.orientation.z = angle;
        goal.target_pose.pose.orientation.w = 1;
        execute_plan = false;
      }

      ROS_INFO_STREAM(goal.target_pose.pose.position.x);
      ROS_INFO_STREAM(goal.target_pose.pose.position.y);

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved");
        execute_plan = false;
        prev.clear();
        // det_sub = nh_.subscribe<novel_msgs::NovelObjectArray>("detected", 1, &Action::det_callback, this);
      } else
        ROS_INFO("The base failed to move for some reason. We will sample new goal point.");
        dist = dist + 0.5;
    }
  }
}

void Action::runAndRotate(int x_rot, int y_rot, int x_run, int y_rot) {
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }

  ROS_INFO("Check if plan is in progress now");

  // if we know what map looks like, where robot is, and a plan is not currently executing
  if (map_known && pose_known && !execute_plan) {
    execute_plan = true; // start plan
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map"; // goal is relative to this frame
    goal.target_pose.header.stamp = ros::Time::now();

    // pull coordinates from queue
    double x = pose.pose.position.x; // robot world coordinate
    double y = pose.pose.position.y;
    int robot_x = (int)round( (x-map_metadata.origin.position.x)/map_resolution); // robot map grid coordinate
    int robot_y = (int)round( (y-map_metadata.origin.position.y)/map_resolution);

    // TODO
  }
}

void Action::constructGrid(int marker_x, int marker_y, int robot_x, int robot_y, std::vector<std::vector<double>>& weights) {
  // construct grid of weights based on distance from marker location
  // see which indices of grid create obstruction between robot and marker and increase weight for those
  // of those indices, add weight based on distance from where robot currently is

  double longestDist = sqrt( pow(map_metadata.width,2) + pow(map_metadata.height,2) ); // length of map diagonal

  double max = 0; // distance furthest from marker position on map
  for (int i = 0; i < map_metadata.height; i++) {
    for (int j = 0; j < map_metadata.width; j++) {
      if ( (grid[i*map_metadata.width + j] < 50) && (grid[i*map_metadata.width + j] >= 0) ) {
        double temp = calcDistance(j, i, marker_x, marker_y);
        if (temp > max) {
          max = temp;
        }
        weights[i][j] = weights[i][j] + temp; // add weight based on distance from marker 
      }
    }
  }

  for (int i = 0; i < map_metadata.height; i++) {
    for (int j = 0; j < map_metadata.width; j++) {
      // check if this grid index is where marker is and make sure it is accessible
      if ((j != marker_x) && (i != marker_y) && (grid[i*map_metadata.width + j] < 50) && grid[i*map_metadata.width + j] >= 0 && j != robot_x && i != robot_y) {
        int dist_x = abs(marker_x - j); // distance from marker position
        int dist_y = abs(marker_y - i);
        int sign_x = (marker_x - j) / abs(marker_x - j);
        int sign_y = (marker_y - i) / abs(marker_y - i);
        double slope = dist_y / dist_x;
        
        int y = i;
        int x = j;
        // Imagine a line from grid[i][j] to marker position
        // Check if each point along line is occupied / inaccessible. 
        // If it is occupied, then there must be an obstacle there
        // We want to prefer points on grid that have obstacle between it and marker pos 
        //   so that robot can better hide
        if (dist_x > dist_y) { 
          x = x + sign_x;
          y = y + slope * sign_y;
          
          while (x != marker_x) {
            if (grid[ round(y)*map_metadata.width + x ] > 50) {
              double distFromRobot = calcDistance( j, i, robot_x, robot_y);
              weights[i][j] = weights[i][j] + max*2 + longestDist / distFromRobot;
              break;
            }
            
            x = x + sign_x;
            y = y + slope * sign_y;
          }
        } else {
          double inv_slope;
          if (dist_x == 0)
            inv_slope = 0;
          else
            inv_slope = 1 / slope;

          y = y + sign_y;
          x = x + inv_slope * sign_x;
          
          while (y != marker_y) {
            if (grid[ y*map_metadata.width + round(x) ] > 50) {
              double distFromRobot = calcDistance( j, i, robot_x, robot_y);
              weights[i][j] = weights[i][j] + max*2 + longestDist / distFromRobot;
              break;
            }            

            y = y + sign_y;
            x = x + inv_slope * sign_x;
          }
        }
      }
    }
  }

}

double Action::calcDistance(double x1, double y1, double x2, double y2) {
  double x = x1 - x2;
  double y = y1 - y2;
  double dist = sqrt(pow(x,2) + pow(y,2));

  return dist;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "action");

  Action action;
  
  ros::spin();
  return 0;
}
