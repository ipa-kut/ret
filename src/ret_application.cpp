#include "ret/ret_application.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ret_application");
  RETApplication app;
}

RETApplication::RETApplication()
{
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);
    ROS_INFO_STREAM("Published " << msg.data);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return;
}
