#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <novel_msgs/NovelObject.h>
#include <novel_msgs/NovelObjectArray.h>
#include <std_msgs/Int8.h>

class Alvar {
  public:
    Alvar();

  private:
    void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callback2(const std_msgs::Int8::ConstPtr& msg);

    ros::NodeHandle nh_;

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber state_sub;

    bool on;
};

Alvar::Alvar() {
  on = false;
  pub = nh_.advertise<novel_msgs::NovelObjectArray>("detected", 1000);
  sub = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1000, &Alvar::callback, this);
  state_sub = nh_.subscribe<std_msgs::Int8>("state", 1, &Alvar::callback2, this);
}

/*
Publish NovelObjectArray of detected markers

Input
-----
msg: message published by ar_track_alvar

Output
------
*/
void Alvar::callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  if (msg->markers.size() > 0 && on) {
    novel_msgs::NovelObjectArray arr;
    arr.header.stamp = ros::Time::now();

    for (int i = 0; i < msg->markers.size(); i++) {
      novel_msgs::NovelObject obj;
      obj.pose.pose = msg->markers[i].pose.pose;
      obj.pose.covariance[0] = 0.005; // in meters
      obj.pose.covariance[7] = 0.005;
      obj.pose.covariance[14] = 0.035;
      obj.pose.covariance[21] = 0.0523599; // approx 3 deg
      obj.pose.covariance[28] = 0.0523599; // approx 3 deg
      obj.pose.covariance[35] = 0.0349066;  // approx 2 deg
      obj.classification = std::to_string(msg->markers[i].id);
      arr.detected_objects.push_back(obj);
    }

    pub.publish(arr);
  }
}

void Alvar::callback2(const std_msgs::Int8::ConstPtr& msg) {
  if (msg->data == 1) {
    on = true;
  } else if (msg->data == 0) {
    on = false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "novel_msg_publisher");

  Alvar alvar;

  ros::spin();
  return 0;
}

