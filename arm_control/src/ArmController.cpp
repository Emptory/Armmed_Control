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

#include <ocs2_mobile_manipulator/ArmController.h>
#include <pluginlib/class_list_macros.h>
#include <fstream>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace arm_control {

ArmController::~ArmController() {
  mpcRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
}

bool ArmController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  nh_ = nh;

  // Load parameters
  std::string urdfFile;
  std::string eeFrame;
  std::string baseFrame;
  double amplitudeX, amplitudeZ, frequency;
  double basePosX, basePosY, basePosZ;
  std::string libraryFolder;
  bool recompileLibraries;

  nh.param<std::string>("urdf_file", urdfFile, "");
  nh.param<std::string>("ee_frame", eeFrame, "tool0");
  nh.param<std::string>("base_frame", baseFrame, "base_link");
  nh.param<double>("amplitude_x", amplitudeX, 0.00);
  nh.param<double>("amplitude_z", amplitudeZ, 0.00);
  nh.param<double>("frequency", frequency, 0.2);
  nh.param<double>("phase_x", phaseX_, 0.0);
  nh.param<double>("phase_z", phaseZ_, M_PI / 2.0);
  nh.param<double>("base_pos_x", basePosX, 0.0);
  nh.param<double>("base_pos_y", basePosY, 0.0);
  nh.param<double>("base_pos_z", basePosZ, 0.8);
  nh.param<std::string>("library_folder", libraryFolder, "/tmp/arm_control_ocs2");
  nh.param<bool>("recompile_libraries", recompileLibraries, true);
  
  nh.param<double>("mpc_rate", mpcRate_, 100.0);
  nh.param<double>("kp", kp_, 100.0);
  nh.param<double>("kd", kd_, 10.0);

  // Handle URDF
  if (urdfFile.empty()) {
    std::string urdfString;
    if (nh.getParam("/robot_description", urdfString)) {
      urdfFile = "/tmp/temp_arm_control.urdf";
      std::ofstream out(urdfFile);
      out << urdfString;
      out.close();
      ROS_INFO_STREAM("Written robot_description to " << urdfFile);
    } else {
      ROS_ERROR("Failed to get robot_description and urdf_file not set");
      return false;
    }
  }

  // Initialize OCS2 Interface
  Eigen::Vector3d basePosition(basePosX, basePosY, basePosZ);
  try {
    armInterfacePtr_ = std::make_unique<ocs2::mobile_manipulator::MobileManipulatorInterface>(
        urdfFile, eeFrame, baseFrame, basePosition,
        amplitudeX, amplitudeZ, frequency,
        phaseX_, phaseZ_,
        libraryFolder, recompileLibraries);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to create MobileManipulatorInterface: " << e.what());
    return false;
  }

  // Initialize MPC
  mpcPtr_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
      armInterfacePtr_->mpcSettings(),
      armInterfacePtr_->ddpSettings(),
      armInterfacePtr_->getRollout(),
      armInterfacePtr_->getOptimalControlProblem(),
      armInterfacePtr_->getInitializer());
  
  mpcPtr_->getSolverPtr()->setReferenceManager(armInterfacePtr_->getReferenceManagerPtr());

  // Initialize MRT
  mrtPtr_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpcPtr_);

  // Initialize Pinocchio
  pinocchioModel_ = &armInterfacePtr_->getPinocchioInterface().getModel();
  pinocchioData_ = std::make_unique<pinocchio::Data>(*pinocchioModel_);

  // Get Joint Handles
  jointNames_ = {"robot2_joint_a1", "robot2_joint_a2", "robot2_joint_a3",
                 "robot2_joint_a4", "robot2_joint_a5", "robot2_joint_a6",
                 "robot2_joint_a7"}; // Default fallback
  
  if (nh.hasParam("joint_names")) {
      nh.getParam("joint_names", jointNames_);
  }

  numJoints_ = jointNames_.size();
  for (const auto& name : jointNames_) {
    try {
      jointHandles_.push_back(hw->getHandle(name));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("Failed to get joint handle: " << name);
      return false;
    }
  }

  currentQ_.resize(numJoints_);
  currentQdot_.resize(numJoints_);

  ROS_INFO("ArmController initialized successfully");
  return true;
}

void ArmController::starting(const ros::Time& time) {
  ROS_INFO("ArmController::starting called");
  startTime_ = time;

  // Initial state
  for (size_t i = 0; i < numJoints_; ++i) {
    currentQ_(i) = jointHandles_[i].getPosition();
    currentQdot_(i) = jointHandles_[i].getVelocity();
  }
  ROS_INFO_STREAM("Current Joint Positions: " << currentQ_.transpose());

  // Reset MPC
  ROS_INFO("Resetting MPC...");
  mpcPtr_->reset();
  
  // Set Reference
  ROS_INFO("Setting Reference...");
  auto& refManager = armInterfacePtr_->getSineReferenceManager();
  ocs2::TargetTrajectories refTargets = refManager.generateTargetTrajectories(0.0, 10.0, 200);
  refManager.setTargetTrajectories(refTargets);
  
  // Initialize MPC Node with current state (Stationary)
  ROS_INFO("Resetting MPC Node...");
  ocs2::TargetTrajectories initGuess;
  initGuess.timeTrajectory.push_back(0.0);
  initGuess.stateTrajectory.push_back(currentQ_);
  initGuess.inputTrajectory.push_back(Eigen::VectorXd::Zero(numJoints_));
  
  mrtPtr_->resetMpcNode(initGuess);

  // Initial Observation
  ROS_INFO("Setting Initial Observation...");
  ocs2::SystemObservation initObservation;
  initObservation.time = 0.0;
  initObservation.state = currentQ_;
  initObservation.input = Eigen::VectorXd::Zero(numJoints_);
  
  mrtPtr_->setCurrentObservation(initObservation);

  // Run initial MPC
  ROS_INFO("Running initial MPC...");
  try {
    mrtPtr_->advanceMpc();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in initial advanceMpc: " << e.what());
    return;
  } catch (...) {
    ROS_ERROR("Unknown exception in initial advanceMpc");
    return;
  }
  
  int waitCount = 0;
  while (!mrtPtr_->initialPolicyReceived() && waitCount < 100) {
    ros::Duration(0.01).sleep();
    waitCount++;
    if (waitCount % 10 == 0) ROS_INFO("Waiting for initial policy...");
  }
  
  if (mrtPtr_->initialPolicyReceived()) {
    mrtPtr_->updatePolicy();
    ROS_INFO("Initial policy received");
  } else {
    ROS_ERROR("Failed to receive initial policy");
  }

  mpcRunning_ = true;
  mpcThread_ = std::thread(&ArmController::mpcThread, this);
  ROS_INFO("MPC Thread started");
}

void ArmController::update(const ros::Time& time, const ros::Duration& period) {
  // Update state
  for (size_t i = 0; i < numJoints_; ++i) {
    currentQ_(i) = jointHandles_[i].getPosition();
    currentQdot_(i) = jointHandles_[i].getVelocity();
  }

  double t = (time - startTime_).toSec();

  // Update MRT observation
  ocs2::SystemObservation observation;
  observation.time = t;
  observation.state = currentQ_;
  observation.input = Eigen::VectorXd::Zero(numJoints_);
  
  mrtPtr_->setCurrentObservation(observation);

  // Get Desired State from Spline
  Eigen::VectorXd q_des, qdot_des, qddot_des;
  bool valid = false;
  {
    std::lock_guard<std::mutex> lock(splineMutex_);
    if (spline_.isValid()) {
      spline_.evaluate(t, q_des, qdot_des, qddot_des);
      valid = true;
    }
  }

  if (valid) {
    // RNEA
    pinocchio::rnea(*pinocchioModel_, *pinocchioData_, currentQ_, currentQdot_, qddot_des);
    Eigen::VectorXd tau_ff = pinocchioData_->tau;

    std::cerr << "RENA" << tau_ff.transpose() << std::endl;

    // PD
    Eigen::VectorXd tau = tau_ff + kp_ * (q_des - currentQ_) + kd_ * (qdot_des - currentQdot_);

    // Send Torque
    for (size_t i = 0; i < numJoints_; ++i) {
      jointHandles_[i].setCommand(tau(i));
    }
  } else {
    // Zero torque if not ready
    for (size_t i = 0; i < numJoints_; ++i) {
      jointHandles_[i].setCommand(0.0);
    }
  }

}

void ArmController::stopping(const ros::Time& time) {
  mpcRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
}

void ArmController::mpcThread() {
  ros::Rate rate(mpcRate_);
  while (mpcRunning_) {
    mrtPtr_->advanceMpc();
    
    if (mrtPtr_->updatePolicy()) {
      // Get trajectory from PrimalSolution
      const auto& policy = mrtPtr_->getPolicy();
      const auto& times = policy.timeTrajectory_;
      const auto& states = policy.stateTrajectory_;
      const auto& inputs = policy.inputTrajectory_;
      
      // Convert to std::vector for spline
      std::vector<double> t_vec;
      std::vector<Eigen::VectorXd> x_vec;
      std::vector<Eigen::VectorXd> u_vec;
      
      for (size_t i = 0; i < times.size(); ++i) {
        t_vec.push_back(times[i]);
        x_vec.push_back(states[i]);
        u_vec.push_back(inputs[i]);
      }
      
      std::lock_guard<std::mutex> lock(splineMutex_);
      spline_.fit(t_vec, x_vec, u_vec);
    }
    
    rate.sleep();
  }
}

} // namespace arm_control

PLUGINLIB_EXPORT_CLASS(arm_control::ArmController, controller_interface::ControllerBase)
