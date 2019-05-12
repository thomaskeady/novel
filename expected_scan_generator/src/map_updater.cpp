/* 

The purpose of this node is to update the occupancy grid with the locations of novel objects so they are not redetected

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"


class MapUpdater 
{
private:
	ros::NodeHandle &nh_;
	
	// Messages
	

	// Publishers
	ros::Publisher map_pub;

	// Subscribers
    ros::Subscriber map_sub;
	ros::Subscriber novel_objects_sub;
	

	// Other
    bool got_map;
    nav_msgs::MapMetaData map_metadata;
	std::vector<signed char, std::allocator<signed char> > grid;
	

public:
    MapUpdater(ros::NodeHandle& nh) :
		nh_(nh),
		

		// Messages

		// Publishers
        map_pub(nh_.advertise<nav_msgs::OccupancyGrid>("map", 100)),

		// Subscribers
        map_sub(nh_.subscribe<nav_msgs::OccupancyGrid>("map", 100, &MapUpdater::mapCb, this)),
		novel_objects_sub(nh_.subscribe("lidar_objects", 100, &MapUpdater::novelObjectsCb, this)),

        // Other
        got_map(false)

	{


		ROS_INFO("Initialized MapUpdater");
	}

    void novelObjectsCb(const novel_msgs::NovelObjectArray::ConstPtr& msg) 
    {
        // Get coords of novel objects
        // Calculate region around novel objects to mark on grid
        // Publish

        if (got_map)
        {
            for (int i = 0; i < sizeof(msg->detected_objects)/sizeof(msg->detected_objects[0]); ++i) 
            {

            }



            map_pub.publish(grid);
        }
    }

    void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        if (!got_map) {
            grid = msg->data;
            map_metadata = msg->info;
            got_map = true;
        }
        
    }


};



