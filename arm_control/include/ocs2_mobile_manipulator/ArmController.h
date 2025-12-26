#pragma once

// Must define boost mpl limits BEFORE any boost headers
#ifndef BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif
#ifndef BOOST_MPL_LIMIT_LIST_SIZE
#define BOOST_MPL_LIMIT_LIST_SIZE 30
#endif
#ifndef BOOST_MPL_LIMIT_VECTOR_SIZE
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30
#endif

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/CubicSplineTrajectory.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <mutex>
#include <thread>
#include <atomic>

namespace arm_control {

class ArmController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  ArmController() = default;
  ~ArmController() override;

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  void mpcThread();

  // ROS handles
  ros::NodeHandle nh_;
  std::vector<hardware_interface::JointHandle> jointHandles_;
  std::vector<std::string> jointNames_;

  // OCS2
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> armInterfacePtr_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrtPtr_;
  std::unique_ptr<ocs2::MPC_BASE> mpcPtr_;
  
  // MPC Thread
  std::thread mpcThread_;
  std::atomic_bool mpcRunning_;
  
  // State
  size_t numJoints_;
  Eigen::VectorXd currentQ_;
  Eigen::VectorXd currentQdot_;
  
  // Control
  CubicSplineTrajectory spline_;
  std::mutex splineMutex_;
  
  // Pinocchio for RNEA
  const pinocchio::Model* pinocchioModel_ = nullptr;
  std::unique_ptr<pinocchio::Data> pinocchioData_;
  
  // Gains
  double kp_, kd_;
  double mpcRate_;
  
  // Control Mode: 1 = MPC, 2 = IK+ID
  int controlMode_;
  
  // IK+ID mode: end-effector frame id
  pinocchio::FrameIndex eeFrameId_;
  
  // Parameters
  double phaseX_, phaseY_, phaseZ_;
  
  // Time
  ros::Time startTime_;

  // Publishers
  ros::Publisher eePosePub_;
  ros::Publisher eeTargetPub_;
};

} // namespace arm_control
