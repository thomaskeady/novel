#include <ros/ros.h>
#include <novel_msgs/NovelObjectArray.h>
#include <novel_msgs/NovelObject.h>
#include <geometry_msgs/PointStamped.h>

class Test {
  public:
    Test();

  private:
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Publisher pub;
    ros::Subscriber sub;
};

Test::Test() {
  pub = nh_.advertise<novel_msgs::NovelObjectArray>("detected", 1);
  sub = nh_.subscribe("clicked_point", 1, &Test::callback, this);
}

void Test::callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  double x = msg->point.x;
  double y = msg->point.y;
  double z = msg->point.z;

  ROS_INFO_STREAM(x);

  novel_msgs::NovelObject obj;
  geometry_msgs::PoseWithCovariance pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  obj.pose = pose;

  novel_msgs::NovelObjectArray arr;
  arr.detected_objects.push_back(obj);

  pub.publish(arr);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "novel_msg_publisher");
  
  Test test;

  ros::spin();
  return 0;
}

