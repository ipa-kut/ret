#include "ret/ret_application.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ret_application");
  ros::NodeHandle nh;
  RETApplication app(nh, argv[1], argv[2]);
}

RETApplication::RETApplication(ros::NodeHandle nh, std::string prompts, std::string robot):
  nh_(nh),
  prompts_(prompts),
  robot_(robot)
{
  ROS_INFO_STREAM("Initialising RET App for " << robot_ << " with prompts: " << prompts_);
  MoveitCustomApi moveit_api(prompts_);
  moveit_api.initialiseMoveit(nh_);
}
