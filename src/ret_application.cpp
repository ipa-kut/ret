#include "ret/ret_application.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ret_application");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RETApplication app(nh, argv[1], argv[2], "192.168.56.1", 65432);
}

/// TODO: Refer ur_manipulation/seher_demo
/// 1. Replace this demo with endurance_demo logic with button press + socket sending
/// 4. Extract the socket ip, socket port, and the button pose values into launch args
RETApplication::RETApplication(ros::NodeHandle nh,
                               std::string prompts,
                               std::string robot,
                               std::string ip,
                               unsigned short port):
  nh_(nh),
  prompts_(prompts),
  robot_(robot)
{
  ROS_INFO_STREAM("Initialising RET App for " << robot_ << " with prompts: " << prompts_);

  logger_.initialize_socket(ip, port);
  logger_.connect_to_server();

  MoveitCustomApi moveit_api(prompts_);
  moveit_api.initialiseMoveit(nh_);
  moveit_api.printBasicInfo();
  moveit_api.addCollissionObjects();

  ROS_INFO("Moving to ready pose");
  std::string ready_state;
   if (robot == "ur_ros")
   {
     ready_state = "ready";
   }
   else if (robot == "prbt")
   {
     ready_state = "all-zeros";
   }
   else
   {
     ROS_ERROR("Robot name invalid!");
     return;
   }
  moveit_api.moveToNamedTarget(ready_state);
  int seq = 0;

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose wp_pose1 = moveit_api.move_group->getCurrentPose().pose;
  waypoints.push_back(wp_pose1);

  wp_pose1.position.x += 0.1;
  waypoints.push_back(wp_pose1);
  wp_pose1.position.x += 0.1;
  waypoints.push_back(wp_pose1);

  wp_pose1.position.z -= 0.1;
  waypoints.push_back(wp_pose1);
  wp_pose1.position.z -= 0.1;
  waypoints.push_back(wp_pose1);

  wp_pose1.position.x -= 0.1;
  waypoints.push_back(wp_pose1);
  wp_pose1.position.x -= 0.1;
  waypoints.push_back(wp_pose1);

  wp_pose1.position.z += 0.1;
  waypoints.push_back(wp_pose1);
  wp_pose1.position.z += 0.1;
  waypoints.push_back(wp_pose1);

  while(ros::ok())
  {
      ROS_INFO_STREAM("----------------------SEQ Start " << seq << "-------------------------------------");
      moveit_api.executeCartesianTrajForWaypoints(waypoints,0.1);
      moveit_api.sleepSafeFor(0.5);
      logger_.setMessage(robot_+";"+
                         std::to_string(ros::Time::now().toSec())+";"+
                         std::to_string(seq));
      logger_.send_data();
      ROS_INFO_STREAM("----------------------SEQ End " << seq++ << "-------------------------------------");
  }
}
