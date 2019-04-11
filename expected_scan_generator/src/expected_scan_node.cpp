/*

Written by Thomas Keady for RSP Spring 2019 final project

Some code borrowed from the ROS tutorials http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

The purpose of this node is to take the estimated robot pose and the known map of the environment (as an occupancy grid) and publish the expected scan the robot's LIDAR should measure. 

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"
//#include "tf/transformations.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include "std_msgs/Int8MultiArray.h"

class ExpectedScanGenerator
{
private:
	ros::NodeHandle &nh_;
	
	// Messages
	//nav_msgs::OccupancyGrid* grid;

	// Publishers
	ros::Publisher scan_pub;

	// Subscribers
	ros::Subscriber grid_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber robot_state_pub;

	// Other
	nav_msgs::MapMetaData map_metadata;
	//std_msgs::Int8MultiArray grid;
	//ros::int8 grid;
	std::vector<signed char, std::allocator<signed char> > grid;
	
	// Append "stamped" if want header as well
	geometry_msgs::PoseWithCovariance pose;

	// Laser Parameters
	float min_angle;
	float max_angle;
	float degs_per_scan;
	int points_per_scan;
	float curr_scan_angle;
	// Assume for now it returns in the same units as the occupancy grid
	float map_scan_angle;

	bool map_known; // False before any messages received
	bool pose_known;

public:
	ExpectedScanGenerator(ros::NodeHandle& nh) :
		nh_(nh),
		

		// Messages
		//grid(NULL),
		//pose(NULL),

		// Publishers
		scan_pub(nh_.advertise<sensor_msgs::LaserScan>("expected_scan", 100)),

		// Subscribers
		grid_sub(nh_.subscribe("nav_msgs/Occupancy_Grid", 100, &ExpectedScanGenerator::ogCb, this)),
		pose_sub(nh_.subscribe("geometry_msgs/PoseWithCovarianceStamped", 100, &ExpectedScanGenerator::poseCb, this)),
		//robot_state_pub(nh_.subscribe("

		min_angle(0),
		max_angle(360),
		degs_per_scan(0.5),
		points_per_scan((int)(max_angle-min_angle)/degs_per_scan),

		// Other
		//map_metadata(NULL),
		//grid(NULL)
		map_known(false),
		pose_known(false)

	{
		ROS_INFO("Initialized ExpectedScanNode");
	}




		void ogCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
	{
		map_known = true;
		map_metadata = msg->info;
		grid = msg->data;
	}

	// For now just store the data and run it at a constant rate in the main loop 
	void poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
	{
		pose_known = true;
		pose = msg->pose;
	}


	bool checkSubscribedTopics() 
	{
		return 	!map_known && !pose_known;
	}

	void generateAndPublishScan() 
	{
		ROS_INFO("Not implemented yet lol");

		// Assume for now transform from robot to lidar is {0 0 0 0 0 0} (TODO fix)
		
		// Perhaps grab lidar specs (# scans, angles, etc) from model? For now hardcode TODO
		
		// JUST FUCKING DO IT - have it round up or down the float slope coords such that they are valid indices and then 
		// end the scan when it hits its first occupied point (they should all be binary right? 
		curr_scan_angle = min_angle;
		
		geometry_msgs::Quaternion test = pose.pose.orientation;// Can tidy this up later
		tf::Quaternion tftest(test.x, test.y, test.z, test.w);				
		tf::Matrix3x3 m(tftest);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		// Adjust for vehicle angle
		map_scan_angle = curr_scan_angle + yaw; 

		while (curr_scan_angle < min_angle) 
		{
			// Need to add pose angle of robot;
			
		}

		
		
		


	}


};

int main (int argc, char** argv) 
{
	ros::init(argc, argv, "expected_scan_node");
	
	ros::NodeHandle nh;
	
	//ros::Subscriber grid_sub = nh.subscribe("nav_msgs/OccupancyGrid", 100, ogCb); // Might need to rename these to match topic names
	//ros::Subscriber scan_sub = nh.subscribe("sensor_msgs/LaserScan");
	//ros::Subscriber pose_sub = nh.subscribe("geometry_msgs/PoseWithCovarianceStamped", 100, poseCb)	
	
	ExpectedScanGenerator myGenerator = ExpectedScanGenerator(nh);

	ROS_INFO("Waiting for one pose message and one map message");

	ros::Rate wait(2);
	while (!myGenerator.checkSubscribedTopics()) { wait.sleep(); };

	ros::Rate r(10);
	while (ros::ok()) 
	{
		myGenerator.generateAndPublishScan();

		r.sleep();
	}


	
}



