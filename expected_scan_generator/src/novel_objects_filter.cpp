/* 

The purpose of this node is to see if novel objects are new or simply redetections of previously detected novel objects

*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "novel_msgs/NovelObject.h" // Is this include redundant?
#include "novel_msgs/NovelObjectArray.h"
#include <vector>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>



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
	std::vector<geometry_msgs::Pose> uniques;
    std::vector<geometry_msgs::PoseStamped> uniquesStamped; // Mirrors uniques
    float distance_thresh;
    int8_t id; 
    std::string fixed_frame;
    bool continous_publish;
    tf::TransformListener &listener_;
    ros::Duration no_life; // Time in s since last seen before removing NO

public:
    NOFilter(ros::NodeHandle& nh, tf::TransformListener& listener) :
		nh_(nh),
		listener_(listener),
		// Messages
		

        // Other
        id(0)

	{
        std::string out_topic;
        std::string in_topic;
        nh_.param<float>("distance_thresh", distance_thresh, 0.5);
        nh_.param<std::string>("filtered_objects_topic", out_topic, "filtered_lidar_objects");
        nh_.param<std::string>("lidar_objects_topic", in_topic,"lidar_objects");
        nh_.param<std::string>("fixed_frame", fixed_frame, "/map");
        nh_.param<bool>("continous_publish", continous_publish, false);
        nh_.param<ros::Duration>("no_life", no_life, 1); 

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

        ros::Time cb_time = ros::Time::now();

        novel_msgs::NovelObjectArray noa;
        if(continous_publish){
            noa.header = msg->header;
            noa.header.frame_id = fixed_frame;
        }
        for (int i = 0; i < msg->detected_objects.size(); ++i) 
        {
            
            bool matched_existing = false;
            std::vector<geometry_msgs::Pose>::iterator closest_NO;
            float closest_distance = distance_thresh; // Since one later in the array may be even closer
            
            novel_msgs::NovelObject current_obj = msg->detected_objects[i];

            geometry_msgs::PoseStamped spose;
            spose.header = msg->header;
            spose.pose = current_obj.pose.pose;
            geometry_msgs::PoseStamped spose_out;
            //spose_out = spose;           
            spose_out.header.stamp = cb_time;

            try{
                listener_.waitForTransform(fixed_frame, "/base_scan",msg->header.stamp, ros::Duration(3.0));
                listener_.transformPose(fixed_frame, spose, spose_out);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("Filter encountered tf error: %s",ex.what());
            }
            int j = 0;
            for (std::vector<geometry_msgs::Pose>::iterator it = uniques.begin() ; it != uniques.end(); it++)
            {
                
                float dist = sqrt(pow(spose_out.pose.position.x - it->position.x, 2) + pow(spose_out.pose.position.y - it->position.y, 2));
                // ROS_INFO("Distance %f", dist);
                if (dist < closest_distance)
                {
                    matched_existing = true;
                    closest_NO = it;
                    closest_distance = dist;
                }
                if(continous_publish){
                    novel_msgs::NovelObject no;
                    no.pose.pose = *it;
                    no.id = j;
                    noa.detected_objects.push_back(no);
                }
                j++;
            }
            
            if (!matched_existing) {
                // Create new object in uniques
                //msg->detected_objects[i].id = id; // If error cause const, change uniques.back
                //++id;
                ROS_INFO("New object detected");
                if (!continous_publish) {
                    noa.detected_objects.push_back(msg->detected_objects[i]);
		            noa.detected_objects.back().id = id++;
                }
                uniques.push_back(spose_out.pose);

                //spose_out.header.stamp = cb_time;
                uniquesStamped.push_back(spose_out);

            } 
            else 
            {
                // ROS_INFO("Connected object");
                // Update closest_NO
                // Just take an average for now
                // Should size or angular_size play a role in this too?
                closest_NO->position.x = (closest_NO->position.x + spose_out.pose.position.x)/2;
                closest_NO->position.y = (closest_NO->position.y + spose_out.pose.position.y)/2;
            }

        } 
        
        // What if something should be removed from this list? Too long without being seen?
        ros::Time check_time = ros::Time::now();
        std::vector<geometry_msgs::PoseStamped>::iterator uit = uniques.begin();
        for (std::vector<geometry_msgs::PoseStamped>::iterator it = uniquesStamped.begin(); it != uniquesStamped.end(); ++it) 
        //for (int i = 0; i < uniquesStamped.size(); ++i) 
        {
            //if (check_time - uniquesStamped[i].header.stamp > no_life)
            if (check_time - it->header.stamp > no_life)
            {
                uniquesStamped.erase(it);
                uniqes.erase(uit++);
            }
        } // Could probably speed this up by not checking ones that were just added, oh well

        
        filtered_novel_objects_pub.publish(noa);
    }


};



int main (int argc, char** argv) 
{
	ros::init(argc, argv, "novel_filter");
	
	ros::NodeHandle nh;
	tf::TransformListener listener;

	NOFilter f = NOFilter(nh, listener);

        ros::spin();
/*	ros::Rate r(10);
	while (ros::ok()) 
	{
		ros::spinOnce();
		r.sleep();
	}
*/	
}


