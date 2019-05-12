/*

Written by Thomas Keady for RSP Spring 2019 final project

Some code borrowed from the ROS tutorials http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
And the navigation sensor tutorials http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors

The purpose of this node is to take the estimated robot pose and the known map of the environment (as an occupancy grid) and publish the expected scan the robot"s LIDAR should measure. 

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"
//#include "tf/transformations.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <cmath>
#include <limits>
class ExpectedScanGenerator
{
private:
	ros::NodeHandle &nh_;
	
	// Messages
	sensor_msgs::LaserScan to_publish;

	// Publishers
	ros::Publisher scan_pub;

	// Subscribers
	ros::Subscriber grid_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber robot_state_pub;

	// Other
	nav_msgs::MapMetaData map_metadata;
	float map_resolution; // Since its empty in hte messages *eye roll* (m/cell)
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

		// Publishers
		scan_pub(nh_.advertise<sensor_msgs::LaserScan>("expected_scan", 100)),

		// Subscribers
		grid_sub(nh_.subscribe("map", 100, &ExpectedScanGenerator::ogCb, this)),
		pose_sub(nh_.subscribe("amcl_pose", 100, &ExpectedScanGenerator::poseCb, this))
		//robot_state_pub(nh_.subscribe("

		

	{
		nh_.param<float>("map_resolution", map_resolution, 0.05);
		nh_.param<float>("min_angle", min_angle, -3.14);
		nh_.param<float>("max_angle", max_angle, 3.14);
		nh_.param<int>("points_per_scan", points_per_scan, 360);
		nh_.param<float>("range", range, 10);
		nh_.param<bool>("map_known", map_known, false);
		nh_.param<bool>("pose_known", pose_known, false);
		angle_increment = std::abs(min_angle - max_angle)/(float)points_per_scan;

		to_publish.angle_min = min_angle;
		to_publish.angle_max = max_angle;
		to_publish.angle_increment = angle_increment;
		to_publish.time_increment = 0;
		to_publish.scan_time = 0.01; // Get this from slamtec for the A2M8
		to_publish.range_max = range;
		to_publish.range_min = 0.153; // Experimentally determined from rplidar

		to_publish.header.frame_id = "/camera_depth_frame";

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

		// Assume for now transform from robot to lidar is {0 0 0 0 0 0} (TODO fix) // does this have to do with frame_id??
		
		// Perhaps grab lidar specs (# scans, angles, etc) from model? For now hardcode TODO (hardcoding took plcae in constructor)
		
		// The plan - have it round up or down the float slope coords such that they are valid indices and then 
		// end the scan when it hits its first occupied point (they should all be binary right? 
		curr_scan_angle = min_angle;
		
		geometry_msgs::Quaternion q = pose.pose.orientation;// Can tidy this up later
		tf::Quaternion tftest(q.x, q.y, q.z, q.w);				
		tf::Matrix3x3 m(tftest);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		// Adjust for vehicle angle
		map_scan_angle = curr_scan_angle + yaw; 

		int max_hyp = range/map_resolution; // message member variable is 0?? Read from RVIZ
		//ROS_INFO("max_hyp = %d\nmin_angle = %f\nmax_angle = %f\npoints_per_scan = %d\nangle_increment = %f\ncalc = %f", max_hyp, min_angle, max_angle, points_per_scan, angle_increment, std::abs(min_angle - max_angle)/(float)points_per_scan);
		std::vector<float> ranges(points_per_scan);

		for (int i = 0; i < points_per_scan; ++i)
		{
			// Keep testing ahead along angle until hits range or hits occupied grid square
			
			//ROS_INFO("per scan (map_scan_angle = %f, angle_increment = %.15f)", map_scan_angle, angle_increment);

			int step = 0; // Declare step at this scope so can push it to laserscan msg later

			// If need more distance precision, make steps smaller (shoudln"t need this, steps proportional to grid size)
			for (step = 1; step < max_hyp; ++step) 
			{
				//ROS_INFO("Per step");

				// POSE IS IN METERS, STEPS ARE IN 0.05 METERS (one per cell, =resolution)
				int x_coord = (int)round((pose.pose.position.x-map_metadata.origin.position.x)/map_resolution + std::cos(map_scan_angle)*step); // Need to round them individually or only after? I think after but confirm TODO
				int y_coord = (int)round((pose.pose.position.y-map_metadata.origin.position.y)/map_resolution + std::sin(map_scan_angle)*step); // Check angle units TODO
				
				int index = (y_coord)*map_metadata.width + x_coord;
				//ROS_INFO("OG val of (%d, %d): %d\tindex: %d", x_coord, y_coord, grid[index], index);
				//ROS_INFO("looking at (%d, %d)", x_coord, y_coord);
	
				if (grid[index] == 100)  // 100 means occupied
				{
					break;
				} else if (grid[index] == 50) 
				{
					break; // If we want to ignore novel objects, do we need to do anything here either?
					// OH wait, the idea is that this node can look at Raph's detected objects and mark their locations on the occupancy grid?
				} // Otherwise, inf? leave at max for now TODO

			}
			if (step == max_hyp)
			{
				ranges[i] = std::numeric_limits<float>::infinity();
			}
			else
			{
				ranges[i] = step*map_resolution;
				if (ranges[i] < to_publish.range_min) 
				{
					ranges[i] = to_publish.range_min;
				}
			}
			
			

			// When done,
			map_scan_angle += angle_increment;
		}

		// Now publish
		to_publish.ranges = ranges;
		scan_pub.publish(to_publish);

	}

};

int main (int argc, char** argv) 
{
	ros::init(argc, argv, "expected_scan_generator");
	
	ros::NodeHandle nh;
	
	ExpectedScanGenerator myGenerator = ExpectedScanGenerator(nh);

	ROS_INFO("Waiting for one pose message and one map message");

	ros::Rate wait(2);
	while (!myGenerator.subscribedTopicsActive()) { wait.sleep(); ros::spinOnce(); };

	ros::Rate r(10);
	while (ros::ok()) 
	{
		ros::spinOnce();
		myGenerator.generateAndPublishScan();
		r.sleep();
	}
	
}



