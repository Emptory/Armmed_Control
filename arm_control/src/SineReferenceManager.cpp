/******************************************************************************
 * SineReferenceManager.cpp
 * 
 * Implementation of sinusoidal end-effector reference trajectory generator.
 ******************************************************************************/

#include <ocs2_mobile_manipulator/SineReferenceManager.h>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

namespace ocs2 {
namespace mobile_manipulator {

SineReferenceManager::SineReferenceManager(const vector3_t& basePosition,
                                           ocs2::scalar_t amplitudeX,
                                           ocs2::scalar_t amplitudeY,
                                           ocs2::scalar_t amplitudeZ,
                                           ocs2::scalar_t frequency,
                                           ocs2::scalar_t phaseX,
                                           ocs2::scalar_t phaseY,
                                           ocs2::scalar_t phaseZ)
    : ocs2::ReferenceManager(),
      basePosition_(basePosition),
      amplitudeX_(amplitudeX),
      amplitudeY_(amplitudeY),
      amplitudeZ_(amplitudeZ),
      frequency_(frequency),
      omega_(2.0 * M_PI * frequency),
      phaseX_(phaseX),
      phaseY_(phaseY),
      phaseZ_(phaseZ) {
}

SineReferenceManager::vector3_t SineReferenceManager::computeSinePosition(ocs2::scalar_t time) const {
  vector3_t pos;
  pos(0) = basePosition_(0) + amplitudeX_ * std::sin(omega_ * time + phaseX_);
  pos(1) = basePosition_(1) + amplitudeY_ * std::sin(omega_ * time + phaseY_);
  pos(2) = basePosition_(2) + amplitudeZ_ * std::sin(omega_ * time + phaseZ_);
  return pos;
}

SineReferenceManager::vector3_t SineReferenceManager::computeSineVelocity(ocs2::scalar_t time) const {
  vector3_t vel;
  vel(0) = amplitudeX_ * omega_ * std::cos(omega_ * time + phaseX_);
  vel(1) = amplitudeY_ * omega_ * std::cos(omega_ * time + phaseY_);
  vel(2) = amplitudeZ_ * omega_ * std::cos(omega_ * time + phaseZ_);
  return vel;
}

ocs2::TargetTrajectories SineReferenceManager::generateTargetTrajectories(
    ocs2::scalar_t initTime, ocs2::scalar_t finalTime, size_t numPoints) const {
  
  ocs2::scalar_array_t timeTrajectory(numPoints);
  ocs2::vector_array_t stateTrajectory(numPoints);
  ocs2::vector_array_t inputTrajectory(numPoints);

  const ocs2::scalar_t dt = (finalTime - initTime) / static_cast<ocs2::scalar_t>(numPoints - 1);

  for (size_t i = 0; i < numPoints; ++i) {
    ocs2::scalar_t t = initTime + static_cast<ocs2::scalar_t>(i) * dt;
    timeTrajectory[i] = t;

    // State trajectory: desired EE position [x, y, z] + Orientation [x, y, z, w]
    vector3_t pos = computeSinePosition(t);
    ocs2::vector_t state(7);
    state.head<3>() = pos;
    // Orientation: Rotate 180 deg around X-axis (Pointing Down) to match IK mode
    // Quaternion (x, y, z, w) = (1.0, 0.0, 0.0, 0.0)
    state.tail<4>() << 1.0, 0.0, 0.0, 0.0; 
    stateTrajectory[i] = state;

    // Input trajectory: desired EE velocity [vx, vy, vz] (for reference, may not be used directly)
    // Note: We pad this to 7D to match the system input dimension (joint velocities) to avoid crashes
    // if this trajectory is used for solver initialization, even though semantically it's EE velocity.
    vector3_t vel = computeSineVelocity(t);
    ocs2::vector_t input = ocs2::vector_t::Zero(7);
    input.head<3>() = vel;
    inputTrajectory[i] = input;
  }

  return ocs2::TargetTrajectories(std::move(timeTrajectory), std::move(stateTrajectory), std::move(inputTrajectory));
}

void SineReferenceManager::modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                                            const ocs2::vector_t& /*initState*/,
                                            ocs2::TargetTrajectories& targetTrajectories,
                                            ocs2::ModeSchedule& /*modeSchedule*/) {
  // Generate sinusoidal reference trajectories for the optimization horizon
  targetTrajectories = generateTargetTrajectories(initTime, finalTime, 50);

  // Publish the current target EE position (for monitoring)
  // Publish only if ROS is initialized and running
  /*
  if (ros::isInitialized() && ros::ok()) {
    static bool pub_initialized = false;
    static ros::Publisher target_pub;
    if (!pub_initialized) {
      ros::NodeHandle nh;
      target_pub = nh.advertise<geometry_msgs::PointStamped>("/ee_pose_target", 1);
      pub_initialized = true;
    }

    if (pub_initialized) {
      geometry_msgs::PointStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "world";
      auto pos = computeSinePosition(initTime);
      msg.point.x = pos(0);
      msg.point.y = pos(1);
      msg.point.z = pos(2);
      target_pub.publish(msg);
    }
  }
  */
}

}  // namespace mobile_manipulator
}  // namespace ocs2
