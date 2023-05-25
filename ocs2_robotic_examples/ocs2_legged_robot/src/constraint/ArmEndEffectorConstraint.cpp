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

#include "ocs2_legged_robot/constraint/ArmEndEffectorConstraint.h"

#include <ocs2_core/misc/LinearInterpolation.h>

#include "ocs2_legged_robot/LeggedRobotPreComputation.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ArmEndEffectorConstraint::ArmEndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                   const ReferenceManager& referenceManager)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManagerPtr_(&referenceManager) {
  if (endEffectorKinematics.getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
  }
  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ArmEndEffectorConstraint::getNumConstraints(scalar_t time) const { return 6; }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ArmEndEffectorConstraint::getValue(scalar_t time, const vector_t& state,
                                            const PreComputation& preComputation) const {
  // // print entering
  // std::cerr << "Entering ArmEndEffectorConstraint getValue" << std::endl;
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<legged_robot::LeggedRobotPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  // const auto desiredPositionOrientation = interpolateEndEffectorPose(time);
  vector_t position(3);
  quaternion_t orientation(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()));
  position << 0.4, 0.0, 0.4;
  const auto desiredPositionOrientation = std::make_pair(position, orientation);

  vector_t constraint(6);
  constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(state).front() - desiredPositionOrientation.first;
  constraint.tail<3>() =
      endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();
  // //print constraint
  // std::cerr<<"constraint"<<constraint.transpose()<<std::endl;
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ArmEndEffectorConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  // // print entering
  // std::cerr << "Entering ArmEndEffectorConstraint getLinearApproximation" << std::endl;
  
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<legged_robot::LeggedRobotPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }
  // const auto desiredPositionOrientation = interpolateEndEffectorPose(time);
  vector_t position(3);
  quaternion_t orientation(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()));
  position << 0.4, 0.0, 0.4;
  const auto desiredPositionOrientation = std::make_pair(position, orientation);
  auto approximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
  approximation.dfdx.topRows<3>() = eePosition.dfdx;

  const auto eeOrientationError =
      endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second})
          .front();
  approximation.f.tail<3>() = eeOrientationError.f;
  approximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;
  // //print approximation
  // std::cerr<<"approximation"<<std::endl;
  // std::cout<<approximation<<std::endl;

  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto ArmEndEffectorConstraint::interpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> {
  const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;

  vector_t position;
  quaternion_t orientation;

  if (stateTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

    const auto& lhs = stateTrajectory[index].tail<7>();
    const auto& rhs = stateTrajectory[index + 1].tail<7>();
    const quaternion_t q_lhs(lhs.tail<4>());
    const quaternion_t q_rhs(rhs.tail<4>());

    position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
    orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
  } else {  // stateTrajectory.size() == 1
    auto EeState = stateTrajectory.front().tail<7>();
    position = EeState.head<3>();
    orientation = quaternion_t(EeState.tail<4>());
  }

  return {position, orientation};
}

}  // namespace legged_robot
}  // namespace ocs2
