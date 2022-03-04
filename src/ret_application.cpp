#include "ret/ret_application.hpp"
#include "tf/tf.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ret_application");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string socket_ip;
  int socket_port;
  if(!nh.getParam("/socket_ip", socket_ip)) ROS_ERROR("Param not set");
  if(!nh.getParam("/socket_port", socket_port)) ROS_ERROR("Param not set");

  RETApplication app(nh,
                     argv[1],
                     argv[2],
                     socket_ip,
                     static_cast<unsigned short>(socket_port));
}

/// TODO: Refer ur_manipulation/seher_demo
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

  ros::Publisher pub_seq = nh.advertise<std_msgs::Header>("/ur_manipulation/sequence", 1);
  ros::Publisher pub_fail = nh.advertise<std_msgs::Header>("/ur_manipulation/failure_counter", 1);
  LoadTargetPoses(nh);

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

  bool switcher = true;
  while (ros::ok())
  {
    ROS_INFO_STREAM("----------------------SEQ Start " << seq << "-------------------------------------");
    LoggedButtonMash(
        &moveit_api,
        switcher ? target_pose1_ : target_pose2_,
        switcher ? 1 : 2);
    ROS_INFO("one mash done");
    switcher = !switcher;
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    msg.seq = seq;
    pub_seq.publish(msg);
    msg.seq = moveit_api.failure_counter_;
    pub_fail.publish(msg);

    ROS_INFO_STREAM("----------------------SEQ End " << seq++
                    << " | Failure Counter = " <<  moveit_api.failure_counter_
                    << "----------------");
  }
}

void RETApplication::LoggedButtonMash(MoveitCustomApi *moveit_api,
                                      geometry_msgs::Pose target_pose,
                                      int button_no)
{
  ROS_INFO("-----Button %d Pre Mash Stage-----", button_no);
  target_pose.position.z += pre_mash_height_;
  moveit_api->executeCartesianTrajtoPose(target_pose, "Pre Mash");
  geometry_msgs::Pose curr_pos = moveit_api->move_group->getCurrentPose().pose;
  ROS_INFO("Button%d - Pre Mash : %s", button_no, moveit_api->comparePoses(curr_pos, target_pose) ? "Success" : "Fail");

  ROS_INFO("-----Button %d Mash Stage-----", button_no);
  target_pose.position.z -= pre_mash_height_;
  moveit_api->executeCartesianTrajtoPose(target_pose, "Mash");
  curr_pos = moveit_api->move_group->getCurrentPose().pose;
  ROS_INFO("Button%d - Mash : %s", button_no, moveit_api->comparePoses(curr_pos, target_pose) ? "Success" : "Fail");
  logger_.setMessage(robot_ + ";" + std::to_string(ros::Time::now().toSec()) + ";" + std::to_string(button_no));
  logger_.send_data();

  ROS_INFO("-----Button %d Post Mash Stage-----", button_no);
  target_pose.position.z += pre_mash_height_;
  moveit_api->executeCartesianTrajtoPose(target_pose, "Post Mash");
  curr_pos = moveit_api->move_group->getCurrentPose().pose;
  ROS_INFO("Button%d - Post Mash : %s", button_no, moveit_api->comparePoses(curr_pos, target_pose) ? "Success" : "Fail");
}

void RETApplication::LoadTargetPoses(ros::NodeHandle nh)
{
  double R, P, Y;
  if (!nh.getParam("/button1/x", target_pose1_.position.x)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button1/y", target_pose1_.position.y)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button1/z", target_pose1_.position.z)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button1/R", R)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button1/P", P)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button1/Y", Y)) ROS_ERROR("Param not set");
  target_pose1_.orientation = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(R), angles::from_degrees(P), angles::from_degrees(Y));
  ROS_INFO("Target pose 1 set to (x:%f, y:%f, z:%f, Roll:%f, Pitch:%f, Yaw:%f)",
           target_pose1_.position.x,
           target_pose1_.position.y,
           target_pose1_.position.z,
           R, P, Y);

  if (!nh.getParam("/button2/x", target_pose2_.position.x)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button2/y", target_pose2_.position.y)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button2/z", target_pose2_.position.z)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button2/R", R)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button2/P", P)) ROS_ERROR("Param not set");
  if (!nh.getParam("/button2/Y", Y)) ROS_ERROR("Param not set");
  target_pose2_.orientation = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(R), angles::from_degrees(P), angles::from_degrees(Y));
  ROS_INFO("Target pose 2 set to (x:%f, y:%f, z:%f, Roll:%f, Pitch:%f, Yaw:%f)",
           target_pose2_.position.x,
           target_pose2_.position.y,
           target_pose2_.position.z,
           R, P, Y);

  if (!nh.getParam("/pre_mash_height", pre_mash_height_)) ROS_ERROR("Param not set");
  ROS_INFO("Pre mash height set to %f", pre_mash_height_);
}
