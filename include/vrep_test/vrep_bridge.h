#pragma once

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
// #include <moveit_vrep_interface/Controller.h>

#define TOTAL_DOF 9 // 9 dof robot(7 + 2 gripper)
#define SIM_DT 0.01 // 10ms
#define PI 3.14159265359
#define deg2rad(deg) ((deg)*PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / PI)

const std::string JOINT_NAME[7] =
{
  "Franka_joint1", "Franka_joint2", "Franka_joint3", "Franka_joint4", "Franka_joint5", "Franka_joint6", "Franka_joint7"
};
const std::string GRIPPER_NAME[4] = 
{
  "panda_left_finger_joint1","panda_left_finger_joint2"
};

class vrep_bridge
{
public:
  vrep_bridge(ros::NodeHandle nh_, double hz_);
  ~vrep_bridge();

  void sim_time_cb(const std_msgs::Float32ConstPtr &msg);
  void sim_step_done_cb(const std_msgs::BoolConstPtr &msg);
  void sim_status_cb(const std_msgs::Int32ConstPtr &msg);
  void joint_cb(const sensor_msgs::JointStateConstPtr &msg);

  void write_robot();
  void wait();
  void read_vrep();
  void vrepStepTrigger();
  void vrepEnableSyncMode();
  void vrepStop();

  Eigen::Matrix<double, 7, 1> current_q_;
  Eigen::Matrix<double, 7, 1> current_qdot_;
  Eigen::Matrix<double, 7, 1> desired_torque_;
  Eigen::Matrix<double, 7, 1> desired_position_;

  Eigen::Matrix<double, 2, 1> current_gripper_;
  Eigen::Matrix<double, 2, 1> desired_gripper_;

private:
  ros::Publisher joint_pub, gripper_pub;
  ros::Subscriber joint_sub;

  ros::Publisher vrep_sim_start_pub_;
  ros::Publisher vrep_sim_stop_pub_;
  ros::Publisher vrep_sim_step_trigger_pub_;
  ros::Publisher vrep_sim_enable_syncmode_pub_;

  ros::Subscriber vrep_sim_step_done_sub_;
  ros::Subscriber vrep_sim_time_sub_;
  ros::Subscriber vrep_sim_status_sub_;

  bool sim_step_done_;
  float sim_time_; // from v-rep simulation time
  int tick;
  int vrep_sim_status;
  ros::Rate rate_;
  ros::WallRate wallrate_ = 1000;
  sensor_msgs::JointState joint_cmd_;
  sensor_msgs::JointState gripper_cmd_;
};