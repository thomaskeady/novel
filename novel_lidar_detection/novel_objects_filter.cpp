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
    int8_t id; 

public:
    NOFilter(ros::NOFilter& nh) :
		nh_(nh),
		
		// Messages

		// Publishers
        filtered_novel_objects_pub(nh_.advertise<novel_msgs::NovelObjectArray>("filtered_lidar_objects", 100)),

		// Subscribers
		novel_objects_sub(nh_.subscribe("lidar_objects", 100, &MapUpdater::novelObjectsCb, this))

        // Other
        id(0);

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
                float dist = sqrt((msg->detected_objects[i].pose.position.x - it->pose.position.x)**2 + (msg->detected_objects[i].pose.position.y - it->pose.position.y)**2);

                if (closest_distance > dist)
                {
                    matched_existing = true;
                    closest_NO = it;
                    closest_distance = dist;
                }
            }

            if (!matched_existing) {
                // Create new object in uniques
                msg->detected_objects[i].id = id; // If error cause const, change uniques.back
                ++id;
                uniques.append(msg->detected_objects[i]);
            } 
            else 
            {
                // Update closest_NO
                // Just take an average for now
                // Should size or angular_size play a role in this too?
                closest_NO->pose.position.x = (closest_NO->pose.position.x + msg->detected_objects[i].pose.position.x)/2;
                closest_NO->pose.position.y = (closest_NO->pose.position.y + msg->detected_objects[i].pose.position.y)/2;
            }

        } // What if something should be removed from this list? Too long without being seen?

        // Need to construct to_publish?
        novel_msgs::NovelObjectArray to_publish[uniques.size]; // May need to just clear instead of re-declare
        int count = 0;
        for (std::vector<novel_msgs::NovelObject>::iterator it = uniques.begin() ; it != uniques.end(); ++it)
        {
            to_publish[count] = *it;
            ++count;
        }

        filtered_novel_objects_pub.publish(to_publish);
    }


};



