#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <Eigen/Core>
#include <vrep_test/model/model.h>
using namespace Eigen;
using namespace RigidBodyDynamics;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;

struct FrankaModelUpdater
{
    Matrix<double, 7, 1> &desired_torque_;
    // Matrix<double, 7, 1> &desired_position_;
    Matrix<double, 7, 1> &q_;                //q
    Matrix<double, 7, 1> &qd_;               //qdot

    std::shared_ptr<Model> rbdl_model_;

    // for robot model construction
    Model model_;
    unsigned int body_id_[dof];
    Math::Vector3d com_position_[dof];
    Math::Matrix3d inertia_[dof];
    Vector3d joint_position_[dof];
    Body body_[dof];
    Joint joint_[dof];

    // arm parameters --
    Matrix<double, 7, 7> mass_matrix_; //m
    Matrix<double, 7, 1> coriolis_;
    Matrix<double, 7, 1> tau_measured_;     //torque
    Matrix<double, 7, 1> tau_desired_read_; //torque_desired
    Matrix<double, 7, 1> gravity_;          //g
    Matrix<double, 6, 7> jacobian_;         //j
    Matrix<double, 3, 1> position_;         //xp
    Matrix<double, 3, 3> rotation_;         //xr
    Affine3d transform_;
    Matrix<double, 6, 1> xd_; //xp_desired + xr_desired?

    Matrix<double, 7, 1> initial_q_; ///< initial joint configuration for idle control
    Affine3d initial_transform_;     ///< initial transform for idle control
    // -- arm parameters

    /* TORQUE MODE */
    FrankaModelUpdater(Matrix<double, 7, 1> &vrep_q, Matrix<double, 7, 1> &vrep_qd, Matrix<double, 7, 1> &vrep_torque) : 
    q_(vrep_q), qd_(vrep_qd), desired_torque_(vrep_torque)
    {
        PandaRBDLModel();
    }

    /* POSITION MODE */
    // FrankaModelUpdater(Matrix<double, 7, 1> &vrep_q, Matrix<double, 7, 1> &vrep_qd, Matrix<double, 7, 1> &vrep_position) :
    //  q_(vrep_q), qd_(vrep_qd), desired_position_(vrep_position)
    // {
    //     PandaRBDLModel();
    // }

    void PandaRBDLModel();
    void updatemodel();
    Affine3d getTransform(const Vector7d &q);
    Matrix<double, 6, 7> getJacobian(const Vector7d &q);
    Matrix<double, 7, 7> getMassMatrix(const Vector7d &q);
    Matrix<double, 7, 1> getGravity(const Vector7d &q);
    void setTorque(const Matrix<double, 7, 1> &torque_command);
    void setPosition(const Matrix<double, 7, 1> &position_command, bool idle_control = false);

    void setInitialTransform();
    void setRobotState(franka::RobotState state);


public:
    double delta_tau_max_{0.05};
};