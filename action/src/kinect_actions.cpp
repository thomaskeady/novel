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
#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Kinect_Action {
  public:
    Kinect_Action();
    void runAway(int x_run, int y_run);
    void rotateForBetterView(int x_rot, int y_rot);
    void runAndRotate(int x_rot, int y_rot, int x_run, int y_run);
    void constructRunGrid(int marker_x, int marker_y, int robot_x, int robot_y, std::vector<std::vector<double>> &weights);
    void constructRunRotGrid(int x_rot, int y_rt, int x_run, int y_run, int robot_x, int robot_y, std::vector<std::vector<double>> & weights);
    double calcDistance(double x1, double y1, double x2, double y2);
    void putObjInMap(double obj_x, double obj_y); // , double robot_x, double robot_y);

  private:
    void det_callback(const novel_msgs::NovelObjectArray::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber det_sub;
    ros::Subscriber map_sub;
    ros::Subscriber pose_sub;

    ros::Publisher map_pub;
    ros::Publisher state_pub;
    ros::Publisher done_moving;

    tf::TransformListener listener;

    nav_msgs::MapMetaData map_metadata;
    float map_resolution;
    std::vector<signed char, std::allocator<signed char> > grid;
    
    geometry_msgs::PoseWithCovariance pose;
    
    bool map_known;
    bool pose_known;
    
    std::vector<std::string> prev;
    std::vector<std::string> run;
    std::vector<std::string> rotate;
    
    double min_marker_det_dist;
};

Kinect_Action::Kinect_Action() {
  map_known = false;
  pose_known = false;
  // min_marker_det_dist = .5;

  det_sub = nh_.subscribe<novel_msgs::NovelObjectArray>("detected", 1, &Kinect_Action::det_callback, this);
  map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Kinect_Action::map_callback, this);
  pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &Kinect_Action::pose_callback, this);

  map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  state_pub = nh_.advertise<std_msgs::Int8>("state", 1);
  done_moving = nh_.advertise<std_msgs::Int8>("moved", 1);

  prev = {}; // ids detected for previous plan
  run = {"2", "4", "6", "8", "10", "12", "14", "16"}; // ids to run away from
  rotate = {"0", "1", "3", "5", "7", "9", "11", "13", "15", "17"}; // ids to get a better view of
}

/*
Determines action to execute based on AR markers detected

Inputs
------
msg: published NovelObjectArray

Output
------

*/
void Kinect_Action::det_callback(const novel_msgs::NovelObjectArray::ConstPtr& msg) {
  // if we know pose of robot, plan is not currently executing, and msg of detected ids came after plan finished
  if (pose_known) {
    std_msgs::Int8 turn_off;
    turn_off.data = 2;
    state_pub.publish(turn_off);

    int x_run = 0;
    int y_run = 0;
    int x_rot = 0;
    int y_rot = 0;
    int run_count = 0;
    int rot_count = 0;

    std::vector<std::string> detected_ids;
    
    bool should_run = false;
    bool should_rotate = false;
    bool unknown = false;
    for (int i = 0; i < msg->detected_objects.size(); i++) {
      // convert marker position world coordinate to map grid coordinate
      std::string label = msg->detected_objects[i].classification;

      tf::StampedTransform transform;
      try {
        listener.lookupTransform("/map", "/ar_marker_"+label, ros::Time(0), transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      int x_coord = (int)round((transform.getOrigin().x() - map_metadata.origin.position.x)/map_resolution);
      int y_coord = (int)round((transform.getOrigin().y() - map_metadata.origin.position.y)/map_resolution);

      putObjInMap(x_coord, y_coord);

      double x_dist = msg->detected_objects[i].pose.pose.position.x;
      double y_dist = msg->detected_objects[i].pose.pose.position.y;  

      // depending on label, turn on bool for specific action
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

    std::sort(detected_ids.begin(), detected_ids.end()); // sort ids but don't think it's necessary
    
    bool nothing_new = true; // check if new ids detected
    for (int j = 0; j < detected_ids.size(); j++) {
      if (std::find(prev.begin(), prev.end(), detected_ids[j]) == prev.end()) {
        nothing_new = false;
        break;
      }   
    }

    // do action if new ids detected
    if (!nothing_new && detected_ids.size() > 0) {
      prev = detected_ids;
      
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

      // average position of detected markers
      // kinect field of view is not very wide so detected markers are probably not very far apart
      // so for convenience sake, let's clump them together as one marker at their average position
      if (unknown) {
        ROS_INFO("Unknown object detected. Not sure what to do.");
        for (int j = 0; j < detected_ids.size(); j++) {
          ROS_INFO_STREAM(detected_ids[j]);
        }
      } else if (should_run && should_rotate) {
        ROS_INFO("Run and rotate");
        x_run = (int)round( (double)x_run / run_count );
        y_run = (int)round( (double)y_run / run_count );
        x_rot = (int)round( (double)x_rot / rot_count );
        y_rot = (int)round( (double)y_rot / rot_count );
        
        runAndRotate(x_run, y_run, x_rot, y_rot);
      } else if (should_run) {
        ROS_INFO("Run");
        x_run = (int)round( (double)x_run / run_count );
        y_run = (int)round( (double)y_run / run_count );
        
        runAway(x_run, y_run);
      } else {
        ROS_INFO("Rotate");
        x_rot = (int)round( (double)x_rot / rot_count );
        y_rot = (int)round( (double)y_rot / rot_count );
        
        rotateForBetterView(x_rot, y_rot);
      }
    } else {
      ROS_INFO_STREAM("Nothing detected.");
    }
  }

  std_msgs::Int8 turn_on;
  turn_on.data = 0;
  state_pub.publish(turn_on);

  std_msgs::Int8 done;
  done.data = 1;
  done_moving.publish(done);
}

/*
Stores map once it is received and shuts down map subscriber

Input
-----
msg: occupancy grid map message

Output
------

*/
void Kinect_Action::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
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
void Kinect_Action::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  pose_known = true;
  pose = msg->pose;
}

/*
Sends location of where robot should run away to

Input
-----
x_run: average x grid coordinate of markers to run away from
y_run: average y grid coordinate of markers to run away from

Output
------
*/
void Kinect_Action::runAway(int x_run, int y_run) {
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }

   // if we know what map looks like, where robot is, and a plan is not currently executing
  if (map_known && pose_known) {
    bool execute_plan = true; // start plan
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map"; // goal is relative to this frame

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

    std::vector<std::vector<double>> weights( map_metadata.height, std::vector<double>(map_metadata.width, 0) ); // weights to sample goal position
    constructRunGrid(x_run, y_run, robot_x, robot_y, weights);

    // calculate total weight in grid
    double weight_total = 0;
    for (int i = 0; i < map_metadata.height; i++) {
      for (int j = 0; j < map_metadata.width; j++) {
         weight_total = weight_total + weights[i][j];
      }
    }    

    // ROS_INFO_STREAM(x);
    // ROS_INFO_STREAM(y);
    
    while (execute_plan) {
      // generate random number to pick grid index (goal position), convert to world coordinates
      double random = (double)rand() / RAND_MAX;
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

      // convert goal position to map coordinate from grid coordinate
      goal.target_pose.pose.position.x = goal_x * map_resolution + map_metadata.origin.position.x;
      goal.target_pose.pose.position.y = goal_y * map_resolution + map_metadata.origin.position.y;

      // convert marker position to map coordinate from grid coordinate
      double marker_x = x_run * map_resolution + map_metadata.origin.position.x;
      double marker_y = y_run * map_resolution + map_metadata.origin.position.y;

      // angle that robot should face to look at ID
      double angle = atan2(marker_y - goal.target_pose.pose.position.y, marker_x - goal.target_pose.pose.position.x);

      goal.target_pose.pose.orientation.z = sin(angle/2);
      goal.target_pose.pose.orientation.w = cos(angle/2);
      goal.target_pose.header.stamp = ros::Time::now();

      // ROS_INFO_STREAM(marker_x);
      // ROS_INFO_STREAM(marker_y);
      // ROS_INFO_STREAM(goal.target_pose.pose.position.x);
      // ROS_INFO_STREAM(goal.target_pose.pose.position.y);

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_INFO("Action complete.");
	execute_plan = false;
	prev.clear(); // clear previously detected ids so that robot can run away again if it still sees ids to run away from
      } else
	ROS_INFO("The base failed to move for some reason. We will sample new goal point.");
    }
  }
}

/*
Sends robot to position close to robot and an orientation such that it is facing the marker

Input
-----
x_rot: average x grid coordinate of markers to rotate towards
y_rot: average y grid coordinate of markers to rotate towards

Output
------
*/
void Kinect_Action::rotateForBetterView(int x_rot, int y_rot) {
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }

  // if we know what map looks like, where robot is, and a plan is not currently executing
  if (map_known && pose_known) {
    bool execute_plan = true; // start plan
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map"; // goal is relative to this frame

    // pull coordinates from queue
    double x = pose.pose.position.x; // robot world coordinate
    double y = pose.pose.position.y;

    int robot_x = (int)round( (x-map_metadata.origin.position.x)) / map_resolution; // robot map coordinate
    int robot_y = (int)round( (y-map_metadata.origin.position.y)) / map_resolution;

    double grid_dist = sqrt( pow(robot_x - x_rot, 2) + pow(robot_y - y_rot, 2) ); // current distance between robot and marker
    double curr_map_dist = grid_dist * map_resolution; // distance in map coordinate

    double marker_x = x_rot * map_resolution + map_metadata.origin.position.x; // marker position in map coordinate
    double marker_y = y_rot * map_resolution + map_metadata.origin.position.y;  
    double angle = atan2(marker_y - y, marker_x - x);
    
    // ROS_INFO_STREAM(x);
    // ROS_INFO_STREAM(y);
    // ROS_INFO_STREAM(marker_x);
    // ROS_INFO_STREAM(marker_y);
    
    double dist = 0.5; // min distance between robot and marker
    while (execute_plan) {
      // if amcl cannot send robot closer to marker, then keep robot in same place and just rotate
      if (dist < curr_map_dist) {
        goal.target_pose.pose.position.x = marker_x - dist*cos(angle);
        goal.target_pose.pose.position.y = marker_y - dist*sin(angle);
        goal.target_pose.pose.orientation.z = sin(angle/2.0);
        goal.target_pose.pose.orientation.w = cos(angle/2.0);
      } else {
        goal.target_pose.pose.position.x = robot_x * map_resolution + map_metadata.origin.position.x;
        goal.target_pose.pose.position.y = robot_y * map_resolution + map_metadata.origin.position.y;
        goal.target_pose.pose.orientation.z = sin(angle/2.0);
        goal.target_pose.pose.orientation.w = cos(angle/2.0);
        execute_plan = false; // even if amcl cannot simply rotate robot in place (for some reason), plan is done
      }
      goal.target_pose.header.stamp = ros::Time::now();
      
      // ROS_INFO_STREAM(angle * 180 / 3.14);
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Action complete.");
        execute_plan = false;
      } else
        if (!execute_plan) {
          ROS_INFO("The base failed to move for some reason. Skipping rotation action.");
        } else {
          ROS_INFO("The base failed to move for some reason. We will sample new goal point.");
        }

        dist = dist + 0.5; // increase distance so robot travels less, leading to (hopefully) easier plan
    }
  }
}

/*
Send location to robot so that it can run away from certain markers and rotate towards other markers

Input
-----
x_rot: average x grid coordinate of markers to rotate towards
y_rot: average y grid coordinate of markers to rotate towards
x_run: average x grid coordinate of markers to run away from
y_run: average y grid coordinate of markers to run away from

Output
------
*/
void Kinect_Action::runAndRotate(int x_rot, int y_rot, int x_run, int y_run) {
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }

  // if we know what map looks like, where robot is, and a plan is not currently executing
  if (map_known && pose_known) {
    bool execute_plan = true; // start plan
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map"; // goal is relative to this frame

    // pull coordinates from queue
    double x = pose.pose.position.x; // robot world coordinate
    double y = pose.pose.position.y;
    int robot_x = (int)round( (x-map_metadata.origin.position.x)/map_resolution); // robot map grid coordinate
    int robot_y = (int)round( (y-map_metadata.origin.position.y)/map_resolution);

    std::vector<std::vector<double>> weights( map_metadata.height, std::vector<double>(map_metadata.width, 0) ); // weights to sample goal position
    constructRunRotGrid(x_rot, y_rot, x_run, y_run, robot_x, robot_y, weights);

    // calculate total weight in grid
    double weight_total = 0;
    for (int i = 0; i < map_metadata.height; i++) {
      for (int j = 0; j < map_metadata.width; j++) {
         weight_total = weight_total + weights[i][j];
      }
    }

    while (execute_plan) {
      // generate random number to pick grid index, convert to world coordinates
      double random = (double)rand() / RAND_MAX;
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

      double marker_x = x_rot * map_resolution + map_metadata.origin.position.x; // marker position in map coordinate
      double marker_y = y_rot * map_resolution + map_metadata.origin.position.y;

      // angle to orient robot such that it is looking at ID to get better view of
      double angle = atan2(marker_y - goal.target_pose.pose.position.y, marker_x - goal.target_pose.pose.position.x);

      goal.target_pose.pose.orientation.z = sin(angle/2.0);
      goal.target_pose.pose.orientation.w = cos(angle/2.0);
      goal.target_pose.header.stamp = ros::Time::now();

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Action complete.");
        execute_plan = false;
        prev.clear(); // clear previously detected IDs in case it still needs to run away
      } else
        ROS_INFO("The base failed to move for some reason. We will sample new goal point.");
    }
  }
}

/*
Construct grid of weights to sample from when deciding where to run to

Input
-----
marker_x: average x grid coordinate of markers to run away from
marker_y: average y grid coordinate of markers to run away from
robot_x: average x grid coordinate of robot position
robot_y: average y grid coordinate of robot position
weights: empty vector of vector of doubles

Output
------
i*/
void Kinect_Action::constructRunGrid(int marker_x, int marker_y, int robot_x, int robot_y, std::vector<std::vector<double>>& weights) {
  // construct grid of weights based on distance from marker location
  // see which indices of grid create obstruction between robot and marker and increase weight for those
  // of those indices, add weight based on distance from where robot currently is
  bool nowhereToHide = true;
  double longestDist = calcDistance(0, 0, map_metadata.width, map_metadata.height); // length of map diagonal

  for (int i = 0; i < map_metadata.height; i++) {
    for (int j = 0; j < map_metadata.width; j++) {
      // check if this grid index is not where marker is and make sure it is accessible
      if ((grid[i*map_metadata.width + j] == 0) && robot_x != j && robot_y != i) {
        double dist_x = abs(marker_x - j); // distance from marker position
        double dist_y = abs(marker_y - i);

        int sign_x;
        double slope;
        if (dist_x == 0) {
          sign_x = 0;
          slope = 0;
        } else {
          sign_x = (marker_x - j) / dist_x;
          slope = dist_y / dist_x; // * sign_x * sign_y;
        }

        int sign_y;
        if (dist_y == 0) {
          sign_y = 0;
        } else {
          sign_y = (marker_y - i) / dist_y;
        }

        double y = i;
        double x = j;
        // Imagine a line from grid[i][j] to marker position
        // Check if each point along line is occupied / inaccessible. 
        // If it is occupied, then there must be an obstacle there
        // We want to prefer points on grid that have obstacle between it and marker pos 
        //   so that robot can better hide
        if (dist_x > dist_y) { 
          x = x + sign_x;
          y = y + slope * sign_y;
          
          while (x != marker_x) {
            if ((int)grid[ round(y)*map_metadata.width + x ] == 100) {
              double distFromRobot = calcDistance( j, i, robot_x, robot_y);
              weights[i][j] = longestDist / distFromRobot; // experimental formula
              nowhereToHide = false;
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
            if ((int)grid[ y*map_metadata.width + round(x) ] == 100) {
              double distFromRobot = calcDistance( j, i, robot_x, robot_y);
              weights[i][j] = longestDist / distFromRobot; // experimental formula
              nowhereToHide = false;
              break;
            }            

            y = y + sign_y;
            x = x + inv_slope * sign_x;
          }
        }
      }
    }
  }

  if (nowhereToHide) {
    for (int i = 0; i < map_metadata.height; i++) {
      for (int j = 0; j < map_metadata.width; j++) {
        if ( (grid[i*map_metadata.width + j] < 50) && (grid[i*map_metadata.width + j] >= 0) ) {
          double temp = calcDistance(j, i, marker_x, marker_y);
          weights[i][j] = temp; // add weight based on distance from marker 
        }
      }
    }
  }

}

/*
Construct grid of weights when deciding where robot should be to run away from certain markers and still
  have a view of the markers robot need to rotate towards

Input
-----
x_rot: average x grid coordinate of markers to rotate towards
y_rot: avearge y grid coordinate of markers to rotate towards
x_run: average x grid coordinate of markers to run away from
y_run: average y grid coordinate of markers to run away from
robot_x: average x grid coordinate of robot position
robot_y: average y grid coordinate of robot position
weights: empty vector of vector of doubles

Output
------

*/
void Kinect_Action::constructRunRotGrid(int x_rot, int y_rot, int x_run, int y_run, int robot_x, int robot_y, std::vector<std::vector<double>>& weights) {
  bool nowhereToHide = true;
  double longestDist = calcDistance(0, 0, map_metadata.width, map_metadata.height); // length of map diagonal

  for (int i = 0; i < map_metadata.height; i++) {
    for (int j = 0; j < map_metadata.width; j++) {
      // check if this grid index is not where average marker positions are  and make sure it is accessible
      if ( j != x_rot && i != y_rot && j != x_run && i != y_run && j != robot_x && i != robot_y && grid[i*map_metadata.width + j] == 0) {
        double dist_x = abs(x_rot - j); // distance from marker position
        double dist_y = abs(y_rot - i);
        int sign_x = (x_rot - j) / abs(x_rot - j); // direction of marker relative to i,j grid index along x,y axes
        int sign_y = (y_rot - i) / abs(y_rot - i);
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

          while (x != x_rot) {
            if (grid[ round(y)*map_metadata.width + x ] > 50 || (round(y) == y_run && x == x_run) ) {
              break;
            }

            x = x + sign_x;
            y = y + slope * sign_y;
          }

          if (x == x_rot) {
            // add weight
            double dist = pow( calcDistance(j, i, x_run, y_run), 2 ); // experimental formula
            weights[i][j] = pow( dist, 2 );
          }
        } else {
          double inv_slope;
          if (dist_x == 0)
            inv_slope = 0;
          else
            inv_slope = 1 / slope;

          y = y + sign_y;
          x = x + inv_slope * sign_x;

          while (y != y_rot) {
            if (grid[ y*map_metadata.width + round(x) ] > 50 || (round(x) == x_run && y == y_run) ) {
              break;
            }

            y = y + sign_y;
            x = x + inv_slope * sign_x;
          }

          if (y == y_rot) {
            // add weight
            double dist = pow( calcDistance(j, i, x_run, y_run), 2 ); // experimental formula
            weights[i][j] = pow( dist, 2 );
          }
        }
      }
    }
  }
}

/*
Calculate distance between two points

Input
-----
x1: x coordinate of point 1
y1: y coordinate of point 1
x2: x coordinate of point 2
y2: y coordinate of point 2

Output
------
distance between two 2D points
*/
double Kinect_Action::calcDistance(double x1, double y1, double x2, double y2) {
  double x = x1 - x2;
  double y = y1 - y2;
  double dist = sqrt(pow(x,2) + pow(y,2));

  return dist;
}

/*
Inserts object into map by changing occupancy value in occupancy grid

Input
-----
obj_x: x coordinate of object (in terms of grid row and column)
obj_y: y coordinate of object

Output
------

*/
void Kinect_Action::putObjInMap(double obj_x, double obj_y) { //, double robot_x, double robot_y) {
  nav_msgs::OccupancyGrid msg1;
  grid[ obj_y*map_metadata.width + obj_x ] = 51;
  grid[ obj_y*map_metadata.width + obj_x - 1 ] = 49;
  grid[ obj_y*map_metadata.width + obj_x + 1 ] = 49;
  grid[ (obj_y-1)*map_metadata.width + obj_x ] = 49;
  grid[ (obj_y+1)*map_metadata.width + obj_x ] = 49;
  msg1.data = grid;
  msg1.info = map_metadata;
  map_pub.publish(msg1);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "action");

  Kinect_Action kinect_action;
  
  ros::spin();
}
