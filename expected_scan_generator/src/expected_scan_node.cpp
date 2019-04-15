/*

Written by Thomas Keady for RSP Spring 2019 final project

Some code borrowed from the ROS tutorials http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
And the navigation sensor tutorials http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors

The purpose of this node is to take the estimated robot pose and the known map of the environment (as an occupancy grid) and publish the expected scan the robot's LIDAR should measure. 

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"
//#include "tf/transformations.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <cmath>

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
	float map_resolution; // Since its empty in hte messages *eye roll*
	//std_msgs::Int8MultiArray grid;
	//ros::int8 grid;
	std::vector<signed char, std::allocator<signed char> > grid;
	
	// Append "stamped" if want header as well
	geometry_msgs::PoseWithCovariance pose;

	// Laser Parameters
	float min_angle;
	float max_angle;
	float angle_increment;
	float range; // Max distance of lidar in meters
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
		map_resolution(0),

		// Publishers
		scan_pub(nh_.advertise<sensor_msgs::LaserScan>("expected_scan", 100)),

		// Subscribers
		//grid_sub(nh_.subscribe("nav_msgs/Occupancy_Grid", 100, &ExpectedScanGenerator::ogCb, this)),
		//pose_sub(nh_.subscribe("geometry_msgs/PoseWithCovarianceStamped", 100, &ExpectedScanGenerator::poseCb, this)),
		grid_sub(nh_.subscribe("map", 100, &ExpectedScanGenerator::ogCb, this)),
		pose_sub(nh_.subscribe("amcl_pose", 100, &ExpectedScanGenerator::poseCb, this)),
		//robot_state_pub(nh_.subscribe("

		min_angle(-3.14),
		max_angle(3.14),
		points_per_scan(1),
		angle_increment((min_angle - max_angle)/points_per_scan),
		range(1), // meters

		// Other
		map_known(false),
		pose_known(false)

	{
		ROS_INFO("Initialized ExpectedScanNode");
	}




	void ogCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
	{
		ROS_INFO("OGCB");
		map_known = true;
		map_metadata = msg->info;
		grid = msg->data;
	}

	// For now just store the data and run it at a constant rate in the main loop 
	void poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
	{
		ROS_INFO("poseCb");
		pose_known = true;
		pose = msg->pose;
	}


	bool subscribedTopicsActive() 
	{
		ROS_INFO("map_known: %s\tpose_known: %s", map_known ? "true" : "false", pose_known ? "true":"false");
		return 	map_known && pose_known;
	}

	void generateAndPublishScan() 
	{
		//ROS_INFO("Not implemented yet lol");

		// Assume for now transform from robot to lidar is {0 0 0 0 0 0} (TODO fix)
		
		// Perhaps grab lidar specs (# scans, angles, etc) from model? For now hardcode TODO (hardcoding took plcae in constructor)
		
		// JUST FUCKING DO IT - have it round up or down the float slope coords such that they are valid indices and then 
		// end the scan when it hits its first occupied point (they should all be binary right? 
		curr_scan_angle = min_angle;
		
		geometry_msgs::Quaternion q = pose.pose.orientation;// Can tidy this up later
		tf::Quaternion tftest(q.x, q.y, q.z, q.w);				
		tf::Matrix3x3 m(tftest);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		// Adjust for vehicle angle
		map_scan_angle = curr_scan_angle + yaw; 

		int max_hyp = range/0.05; // message member variable is 0?? Read from RVIZ
		//int max_hyp = range/map_metadata.resolution;
		/*ROS_INFO("resolution = %.15f", map_metadata.resolution);
		int test = ceil(map_metadata.resolution);
		ROS_INFO("resolution = %d", test);*/
		ROS_INFO("max_hyp = %d", max_hyp);


		//while (curr_scan_angle < min_angle) 
		for (int i = 0; i < points_per_scan; ++i)
		{
			// Keep testing ahead along angle until hits range or hits occupied grid square

			//int hypotenuse = 1; // Length (in map pixels) that the simulated scan looks ahead by at each test // now replaced with step in loop
					
			ROS_INFO("per scan");

			for (int step = 1; step < max_hyp; ++step) 
			{
				ROS_INFO("Per step");

				// POSE IS IN METERS, STEPS ARE IN 0.05 METERS (one per cell, =resolution)
				int x_coord = (int)round(pose.pose.position.x/0.05 + std::sin(step)); // Need to round them individually or only after? I think after but confirm TODO
				int y_coord = (int)round(pose.pose.position.y/0.05 + std::cos(step));
				
				//if (grid[] > 
				//ROS_INFO("OG val of (%d, %d): %d", x_coord, y_coord, grid[(y_coord-1)*map_metadata.width + x_coord]);
				ROS_INFO("looking at (%d, %d)", x_coord, y_coord);
			}


			// When done,
			map_scan_angle += angle_increment;
		}

		
		
		


	}


};

int main (int argc, char** argv) 
{
	ros::init(argc, argv, "expected_scan_generator");
	
	ros::NodeHandle nh;
	
	//ros::Subscriber grid_sub = nh.subscribe("nav_msgs/OccupancyGrid", 100, ogCb); // Might need to rename these to match topic names
	//ros::Subscriber scan_sub = nh.subscribe("sensor_msgs/LaserScan");
	//ros::Subscriber pose_sub = nh.subscribe("geometry_msgs/PoseWithCovarianceStamped", 100, poseCb)	
	
	ExpectedScanGenerator myGenerator = ExpectedScanGenerator(nh);

	ROS_INFO("Waiting for one pose message and one map message");

	ros::Rate wait(2);
	while (!myGenerator.subscribedTopicsActive()) { wait.sleep(); ros::spinOnce(); };
	/*while (true) 
	{ 
		ROS_INFO("before");
		ROS_INFO("%s", myGenerator.subscribedTopicsActive() ? "true" : "false");
		ros::spinOnce();

		if (myGenerator.subscribedTopicsActive()) {
			break;
		}

		wait.sleep(); 
		ROS_INFO("after");
	};*/

	ros::Rate r(10);
	while (ros::ok()) 
	{
		myGenerator.generateAndPublishScan();

		r.sleep();
	}


	
}



