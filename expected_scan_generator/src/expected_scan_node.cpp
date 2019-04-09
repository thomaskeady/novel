/*

Written by Thomas Keady for RSP Spring 2019 final project

Some code borrowed from the ROS tutorials http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

The purpose of this node is to take the estimated robot pose and the known map of the environment (as an occupancy grid) and publish the expected scan the robot's LIDAR should measure. 

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Int8MultiArray.h"

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
	

	// Other
	nav_msgs::MapMetaData map_metadata;
	//std_msgs::Int8MultiArray grid;
	//ros::int8 grid;
	std::vector<signed char, std::allocator<signed char> > grid;
	
	// Append "stamped" if want header as well
	geometry_msgs::PoseWithCovariance pose;

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



