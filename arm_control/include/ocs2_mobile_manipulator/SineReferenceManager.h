/******************************************************************************
 * SineReferenceManager.h
 * 
 * Generates sinusoidal end-effector reference trajectories.
 * p_des(t) = p0 + A * sin(omega * t + phase)
 * 
 * Extends ReferenceManager to produce time-varying target trajectories.
 ******************************************************************************/

#pragma once

#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {
namespace mobile_manipulator {

/**
 * Reference manager that generates sinusoidal end-effector position references.
 * The state target contains the desired end-effector position (3D) which can be
 * extracted by the cost function.
 */
class SineReferenceManager : public ocs2::ReferenceManager {
 public:
  using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;

  /**
   * Constructor
   * @param basePosition Base position p0 (initial EE position)
   * @param amplitudeX Amplitude in X direction
   * @param amplitudeY Amplitude in Y direction (usually 0)
   * @param amplitudeZ Amplitude in Z direction
   * @param frequency Frequency in Hz
   * @param phaseX Phase offset for X (radians)
   * @param phaseZ Phase offset for Z (radians)
   */
  SineReferenceManager(const vector3_t& basePosition,
                       ocs2::scalar_t amplitudeX,
                       ocs2::scalar_t amplitudeY,
                       ocs2::scalar_t amplitudeZ,
                       ocs2::scalar_t frequency,
                       ocs2::scalar_t phaseX = 0.0,
                       ocs2::scalar_t phaseZ = 0.0);

  ~SineReferenceManager() override = default;

  /**
   * Compute sinusoidal position at a given time.
   */
  vector3_t computeSinePosition(ocs2::scalar_t time) const;

  /**
   * Compute sinusoidal velocity at a given time.
   */
  vector3_t computeSineVelocity(ocs2::scalar_t time) const;

  /**
   * Generate target trajectories for the given time horizon.
   * The stateTrajectory will contain [x, y, z] positions.
   */
  ocs2::TargetTrajectories generateTargetTrajectories(ocs2::scalar_t initTime,
                                                      ocs2::scalar_t finalTime,
                                                      size_t numPoints = 100) const;

  // Getters
  const vector3_t& getBasePosition() const { return basePosition_; }
  ocs2::scalar_t getAmplitudeX() const { return amplitudeX_; }
  ocs2::scalar_t getAmplitudeZ() const { return amplitudeZ_; }
  ocs2::scalar_t getFrequency() const { return frequency_; }

 protected:
  void modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                        const ocs2::vector_t& initState,
                        ocs2::TargetTrajectories& targetTrajectories,
                        ocs2::ModeSchedule& modeSchedule) override;

 private:
  vector3_t basePosition_;
  ocs2::scalar_t amplitudeX_;
  ocs2::scalar_t amplitudeY_;
  ocs2::scalar_t amplitudeZ_;
  ocs2::scalar_t frequency_;  // frequency in Hz
  ocs2::scalar_t omega_;  // 2*pi*frequency
  ocs2::scalar_t phaseX_;
  ocs2::scalar_t phaseZ_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
