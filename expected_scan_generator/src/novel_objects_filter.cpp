/* 

The purpose of this node is to see if novel objects are new or simply redetections of previously detected novel objects

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "novel_msgs/NovelObject.h" // Is this include redundant?
#include "novel_msgs/NovelObjectArray.h"
#include <vector>
#include <tf/transform_listener.h>
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
    NOFilter(ros::NodeHandle& nh) :
		nh_(nh),
		
		// Messages

		

        // Other
        id(0)

	{
        std::string out_topic;
        std::string in_topic;
        nh_.param<float>("distance_thresh", distance_thresh, 0.5);
        nh_.param<std::string>("filtered_objects_topic", out_topic, "filtered_lidar_objects");
        nh_.param<std::string>("lidar_objects_topic", in_topic,"lidar_objects");

        // Publishers
        filtered_novel_objects_pub = nh_.advertise<novel_msgs::NovelObjectArray>(out_topic, 100);

		// Subscribers
		novel_objects_sub = nh_.subscribe(in_topic, 100, &NOFilter::novelObjectsCb, this);
		ROS_INFO("Initialized NOFilter");
	}

    void novelObjectsCb(const novel_msgs::NovelObjectArray::ConstPtr& msg) 
    {
        // Get coords of new novel objects
        // If close enough to existing objects, consider same & update object position
        // Publish
	    novel_msgs::NovelObjectArray noa;
        for (int i = 0; i < msg->detected_objects.size(); ++i) 
        {
            bool matched_existing = false;
            std::vector<novel_msgs::NovelObject>::iterator closest_NO;
            float closest_distance = distance_thresh; // Since one later in the array may be even closer

            for (std::vector<novel_msgs::NovelObject>::iterator it = uniques.begin() ; it != uniques.end(); it++)
            {
                float dist = sqrt(pow(msg->detected_objects[i].pose.pose.position.x - it->pose.pose.position.x, 2) + pow(msg->detected_objects[i].pose.pose.position.y - it->pose.pose.position.y, 2));

                if (closest_distance > dist)
                {
                    matched_existing = true;
                    closest_NO = it;
                    closest_distance = dist;
                }
            }

            if (!matched_existing) {
                // Create new object in uniques
                //msg->detected_objects[i].id = id; // If error cause const, change uniques.back
                //++id;
                ROS_INFO("New object detected");
                noa.detected_objects.push_back(msg->detected_objects[i]);
		        noa.detected_objects.back().id = id++;
            } 
            else 
            {
                ROS_INFO("Connected object");
                // Update closest_NO
                // Just take an average for now
                // Should size or angular_size play a role in this too?
                try{
                listener.lookupTransform("/turtle2", "/turtle1",
                                        ros::Time(0), transform);
                }
                catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
    }
                closest_NO->pose.pose.position.x = (closest_NO->pose.pose.position.x + msg->detected_objects[i].pose.pose.position.x)/2;
                closest_NO->pose.pose.position.y = (closest_NO->pose.pose.position.y + msg->detected_objects[i].pose.pose.position.y)/2;
            }

        } // What if something should be removed from this list? Too long without being seen?

        // Need to construct to_publish?
	
	
        //novel_msgs::NovelObject to_publish[uniques.size()]; // May need to just clear instead of re-declare
        //int count = 0;
        //for (std::vector<novel_msgs::NovelObject>::iterator it = uniques.begin() ; it != uniques.end(); ++it)
	//for (int i = 0; i < uniques.size(); ++i) 
        //{
            //to_publish[count] = it;
          //  to_publish[count] = uniques[i];
            //++count;
        //}

        filtered_novel_objects_pub.publish(noa);
    }


};
int main (int argc, char** argv) 
{
	ros::init(argc, argv, "novel_filter");
	
	ros::NodeHandle nh;
	
	NOFilter f = NOFilter(nh);

	ros::Rate r(10);
	while (ros::ok()) 
	{
		ros::spinOnce();
		r.sleep();
	}
	
}


