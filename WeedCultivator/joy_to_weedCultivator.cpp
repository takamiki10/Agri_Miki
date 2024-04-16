#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>

ros::Publisher pub;
std_msgs::Float32MultiArray msg;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  msg.data[0] = joy->axes[1];
  msg.data[1] = joy->axes[1];
  msg.data[2] = joy->axes[1] * -1;
  msg.data[3] = joy->axes[1] * -1;

  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "value_publisher");
  ros::NodeHandle nh;

  msg.data.resize(4);

  pub = nh.advertise<std_msgs::Float32MultiArray>("float_array", 10);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joyCallback);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
