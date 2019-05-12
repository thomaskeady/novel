/* 

The purpose of this node is to see if novel objects are new or simply redetections of previously detected novel objects

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "novel_msgs/NovelObject.h" // Is this include redundant?
#include "novel_msgs/NovelObjectArray.h"
#include <vector>

class NOFilter 
{
private:
	ros::NodeHandle &nh_;
	
	// Messages
	novel_msgs::NovelObjectArray to_publish;

	// Publishers
	ros::Publisher filtered_novel_objects_pub;

	// Subscribers
	ros::Subscriber novel_objects_sub;

	// Other
	std::vector<novel_msgs::NovelObject> uniques;
    float distance_thresh;

public:
    NOFilter(ros::NOFilter& nh) :
		nh_(nh),
		
		// Messages

		// Publishers
        filtered_novel_objects_pub(nh_.advertise<novel_msgs::NovelObjectArray>("filtered_lidar_objects", 100)),

		// Subscribers
		novel_objects_sub(nh_.subscribe("lidar_objects", 100, &MapUpdater::novelObjectsCb, this))

        // Other
        

	{
        nh_.param<float>("distance_thresh", distance_thresh, 0.5);

		ROS_INFO("Initialized NOFilter");
	}

    void novelObjectsCb(const novel_msgs::NovelObjectArray::ConstPtr& msg) 
    {
        // Get coords of new novel objects
        // If close enough to existing objects, consider same & update object position
        // Publish

        for (int i = 0; i < sizeof(msg->detected_objects)/sizeof(msg->detected_objects[0]); ++i) 
        {
            bool matched_existing = false;
            std::vector<novel_msgs::NovelObject>::iterator closest_NO;
            float closest_distance = distance_thresh; // Since one later in the array may be even closer

            for (std::vector<novel_msgs::NovelObject>::iterator it = uniques.begin() ; it != uniques.end(); ++it)
            {
                float dist = sqrt((msg->detected_objects.pose.position.x - it->pose.position.x)**2 + (msg->detected_objects.pose.position.y - it->pose.position.y)**2);

                if (closest_distance > dist)
                {
                    matched_existing = true;
                    closest_NO = it;
                    closest_distance = dist;
                }
            }

            if (!matched_existing) {
                // Create new object in uniques

            } 
            else 
            {
                // Update closest_NO

            }

        }



        // Need to construct to_publish?
        filtered_novel_objects_pub.publish(to_publish);
    }

/*
    void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        if (!got_map) {
            grid = msg->data;
            map_metadata = msg->info;
            got_map = true;
        }
        
    }
*/

};



