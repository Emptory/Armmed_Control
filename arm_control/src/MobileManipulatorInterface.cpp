/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include <ocs2_core/cost/QuadraticStateCost.h>
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h"
#include "ocs2_mobile_manipulator/cost/QuadraticInputCost.h"
#include "ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/SineReferenceManager.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorInterface::MobileManipulatorInterface(const std::string& urdfFile,
                                                       const std::string& eeFrame,
                                                       const std::string& baseFrame,
                                                       const vector_t& basePosition,
                                                       scalar_t amplitudeX,
                                                       scalar_t amplitudeZ,
                                                       scalar_t frequency,
                                                       scalar_t phaseX,
                                                       scalar_t phaseZ,
                                                       const std::string& libraryFolder,
                                                       bool recompileLibraries) {
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
  }
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

  // Default Manipulator Type
  ManipulatorModelType modelType = ManipulatorModelType::DefaultManipulator;
  
  // Pinocchio Interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(mobile_manipulator::createPinocchioInterface(urdfFile, modelType)));
  std::cerr << *pinocchioInterfacePtr_;

  // Manipulator Model Info
  manipulatorModelInfo_ = mobile_manipulator::createManipulatorModelInfo(*pinocchioInterfacePtr_, modelType, baseFrame, eeFrame);

  // Initial State
  initialState_ = vector_t::Zero(manipulatorModelInfo_.stateDim);

  // DDP Settings
  ddpSettings_.algorithm_ = ddp::Algorithm::SLQ;
  ddpSettings_.nThreads_ = 1;
  ddpSettings_.maxNumIterations_ = 1;
  ddpSettings_.minRelCost_ = 1e-2;
  ddpSettings_.constraintTolerance_ = 1e-2;
  ddpSettings_.displayInfo_ = false;
  ddpSettings_.displayShortSummary_ = false;
  ddpSettings_.absTolODE_ = 1e-5;
  ddpSettings_.relTolODE_ = 1e-4;
  ddpSettings_.timeStep_ = 0.05;
  ddpSettings_.backwardPassIntegratorType_ = IntegratorType::ODE45;

  // MPC Settings
  mpcSettings_.timeHorizon_ = 1.0;
  mpcSettings_.solutionTimeWindow_ = 0.5;
  mpcSettings_.coldStart_ = false;
  mpcSettings_.debugPrint_ = false;
  mpcSettings_.mpcDesiredFrequency_ = -1;
  mpcSettings_.mrtDesiredFrequency_ = -1;

  // Reference Manager
  referenceManagerPtr_ = std::make_shared<SineReferenceManager>(
      basePosition, amplitudeX, 0.0, amplitudeZ, frequency, phaseX, phaseZ);
  
  // Optimal Control Problem
  // Cost
  problem_.costPtr->add("inputCost", getQuadraticInputCost("")); // Empty task file string

  // State Regularization (CRITICAL FIX: Prevents zero eigenvalues in Hessian)
  matrix_t Q(manipulatorModelInfo_.stateDim, manipulatorModelInfo_.stateDim);
  Q.setIdentity();
  Q.diagonal().head(manipulatorModelInfo_.armDim) *= 0.01;  // Position regularization (Reduced from 1.0)
  Q.diagonal().tail(manipulatorModelInfo_.armDim) *= 0.01;  // Velocity regularization (Reduced from 0.1)
  problem_.stateCostPtr->add("stateCost", std::make_unique<QuadraticStateCost>(Q));

  // Final State Regularization (CRITICAL FIX for "Ill-posed problem at final time")
  matrix_t Q_final(manipulatorModelInfo_.stateDim, manipulatorModelInfo_.stateDim);
  Q_final.setIdentity();
  Q_final.diagonal().head(manipulatorModelInfo_.armDim) *= 0.1; // Stronger position regularization at the end (Reduced from 10.0)
  Q_final.diagonal().tail(manipulatorModelInfo_.armDim) *= 0.1;
  problem_.finalCostPtr->add("finalStateCost", std::make_unique<QuadraticStateCost>(Q_final));

  // Constraints
  // Joint limits
  problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, ""));

  // End-effector
  bool usePreComputation = true;
  problem_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, "", "endEffector",
                                                                               usePreComputation, libraryFolder, recompileLibraries));
  problem_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, "", "finalEndEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries));

  // Dynamics
  problem_.dynamicsPtr.reset(
      new DefaultManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));

  /*
   * Pre-computation
   */
  if (usePreComputation) {
    problem_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, manipulatorModelInfo_));
  }

  // Rollout
  auto rolloutSettings = rollout::Settings();
  rolloutSettings.absTolODE = 1e-5;
  rolloutSettings.relTolODE = 1e-4;
  rolloutSettings.timeStep = 0.01;
  rolloutSettings.integratorType = IntegratorType::ODE45;
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  initializerPtr_.reset(new DefaultInitializer(manipulatorModelInfo_.inputDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getQuadraticInputCost(const std::string& taskFile) {
  matrix_t R = matrix_t::Identity(manipulatorModelInfo_.inputDim, manipulatorModelInfo_.inputDim);
  R *= 0.1; // Default weight

  return std::make_unique<QuadraticInputCost>(std::move(R), manipulatorModelInfo_.stateDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                const std::string& taskFile, const std::string& prefix,
                                                                                bool usePreComputation, const std::string& libraryFolder,
                                                                                bool recompileLibraries) {
  scalar_t muPosition = 100.0;
  scalar_t muOrientation = 50.0;

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrame},
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                        const std::string& taskFile) {
  const auto& model = pinocchioInterface.getModel();
  
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  scalar_t muPositionLimits = 1e-2;
  scalar_t deltaPositionLimits = 1e-3;

  for (int i = 0; i < manipulatorModelInfo_.stateDim; ++i) {
    StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
    boxConstraint.index = i;
    boxConstraint.lowerBound = model.lowerPositionLimit(i);
    boxConstraint.upperBound = model.upperPositionLimit(i);
    boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
    stateLimits.push_back(std::move(boxConstraint));
  }

  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  scalar_t muVelocityLimits = 1e-2;
  scalar_t deltaVelocityLimits = 1e-3;
  
  for (int i = 0; i < manipulatorModelInfo_.inputDim; ++i) {
    StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
    boxConstraint.index = i;
    boxConstraint.lowerBound = -model.velocityLimit(i);
    boxConstraint.upperBound = model.velocityLimit(i);
    boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
    inputLimits.push_back(std::move(boxConstraint));
  }

  auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
  boxConstraints->initializeOffset(0.0, vector_t::Zero(manipulatorModelInfo_.stateDim), vector_t::Zero(manipulatorModelInfo_.inputDim));
  return boxConstraints;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
