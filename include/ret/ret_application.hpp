#ifndef RET_APPLICATION_HPP
#define RET_APPLICATION_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "ur_manipulation/moveit_custom_api.hpp"
#include "ret/ret_logger.hpp"

class RETApplication
{
public:
  RETApplication(ros::NodeHandle nh,
                 std::string prompts,
                 std::string robot,
                 std::string ip,
                 unsigned short port);
  void LoggedButtonMash(MoveitCustomApi* moveit_api, geometry_msgs::Pose target_pose, int button_no);
  void LoadTargetPoses(ros::NodeHandle nh);

private:
  ros::NodeHandle nh_;
  std::string prompts_;
  std::string robot_;
  EnduranceTestLogger logger_;
  geometry_msgs::Pose target_pose1_;
  geometry_msgs::Pose target_pose2_;
  double pre_mash_height_;
};

#endif // RET_APPLICATION_HPP
