#include "vrep_test/vrep_bridge.h"
vrep_bridge::vrep_bridge(ros::NodeHandle nh_, double hz_) : rate_(hz_)
{
  current_q_.setZero();
  current_qdot_.setZero();
  // current_q_ << M_PI/6, M_PI/6, M_PI/6, -M_PI/6, M_PI/6, M_PI/6, M_PI/6;
  current_q_ << 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, 0.0;
  current_gripper_.setZero();
  
  desired_torque_.setZero();
  
  /* INITIAL POSITION VALUE */
  desired_position_ << 0, 0, 0, -1.57, 0, 0, 0.756;
  desired_gripper_ << 0.04, 0.04;

  /* finger까지 포함할지 안할지?????*/
  joint_cmd_.name.resize(dof);
  joint_cmd_.position.resize(dof);
  joint_cmd_.effort.resize(dof);

  gripper_cmd_.name.resize(4);
  gripper_cmd_.position.resize(4);

  for (size_t i = 0; i < dof; i++)
  {
    joint_cmd_.name[i] = JOINT_NAME[i];
  }

  for (size_t i = 0; i < 4; i++)
  {
    gripper_cmd_.name[i] = GRIPPER_NAME[i];
  }

  joint_sub = nh_.subscribe("/panda/joint_states", 100, &vrep_bridge::joint_cb, this); //from vrep (arm + gripper both)
  joint_pub = nh_.advertise<sensor_msgs::JointState>("/panda/joint_set", 1); //to vrep (only arm)
  gripper_pub = nh_.advertise<sensor_msgs::JointState>("/panda/gripper_set", 1); //to vrep (only gripper)

  vrep_sim_start_pub_ = nh_.advertise<std_msgs::Bool>("/startSimulation", 5);
  vrep_sim_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stopSimulation", 5);
  vrep_sim_step_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/triggerNextStep", 100);
  vrep_sim_enable_syncmode_pub_ = nh_.advertise<std_msgs::Bool>("/enableSyncMode", 5);

  vrep_sim_step_done_sub_ = nh_.subscribe("/simulationStepDone", 100, &vrep_bridge::sim_step_done_cb, this);
  vrep_sim_time_sub_ = nh_.subscribe("/simulationTime", 100, &vrep_bridge::sim_time_cb, this);
  vrep_sim_status_sub_ = nh_.subscribe("/simulationState", 100, &vrep_bridge::sim_status_cb, this);
}

vrep_bridge::~vrep_bridge()
{
}
void vrep_bridge::sim_time_cb(const std_msgs::Float32ConstPtr &msg)
{
  sim_time_ = msg->data;
  tick = (sim_time_ * 1000) / (SIM_DT * 1000);
}

void vrep_bridge::sim_step_done_cb(const std_msgs::BoolConstPtr &msg)
{
  sim_step_done_ = msg->data;
}

  void vrep_bridge::sim_status_cb(const std_msgs::Int32ConstPtr& msg)
  {
    vrep_sim_status = msg->data;
  }

void vrep_bridge::joint_cb(const sensor_msgs::JointStateConstPtr &msg)
{
  for (size_t i = 0; i < 7; i++)
  {
    current_q_[i] = msg->position[i]; // from vrep
    current_qdot_[i] = msg->velocity[i];
  }

  for (size_t i =0; i < 2; i++)
  {
    current_gripper_[i] = msg->position[i];
  }
}


void vrep_bridge::write_robot()
{
  /* TORQUE MODE */
  for (size_t i = 0; i < dof; i++)
  {
    joint_cmd_.effort[i] = desired_torque_[i];
  }

  /* POSITION MODE */
//   for (size_t i = 0; i < dof; i++)
//   {
//     joint_cmd_.position[i] = desired_position_[i];
//   }
//   for (size_t i = 0; i < 2; i++)
//   {
//     joint_cmd_.position[7 + i] = desired_gripper_[i];
//   }
  joint_pub.publish(joint_cmd_);
  // gripper_pub.publish(gripper_cmd_);
  vrepStepTrigger();
}

void vrep_bridge::wait()
{
  while (ros::ok() && !sim_step_done_)
  {
    ros::spinOnce();
  }
  sim_step_done_= false;
  wallrate_.sleep();
}

void vrep_bridge::read_vrep()
{
  ros::spinOnce();
}
void vrep_bridge::vrepStepTrigger()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_step_trigger_pub_.publish(msg);
}

void vrep_bridge::vrepEnableSyncMode()
{
  ROS_INFO("Sync Mode On");
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_enable_syncmode_pub_.publish(msg);
}

void vrep_bridge::vrepStop()
{
  ROS_INFO("Stopping V-REP Simulation");
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_stop_pub_.publish(msg);
}