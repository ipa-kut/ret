#ifndef RET_APPLICATION_HPP
#define RET_APPLICATION_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "ur_manipulation/moveit_custom_api.hpp"

class RETApplication
{
public:
  RETApplication(ros::NodeHandle nh, std::string prompts, std::string robot);

private:
  ros::NodeHandle nh_;
  std::string prompts_;
  std::string robot_;
};

#endif // RET_APPLICATION_HPP
