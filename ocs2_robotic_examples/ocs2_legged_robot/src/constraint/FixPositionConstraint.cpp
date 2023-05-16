/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_legged_robot/constraint/FixPositionConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FixPositionConstraint::FixPositionConstraint() : StateConstraint(ConstraintOrder::Linear) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FixPositionConstraint::isActive(scalar_t time) const { return true; }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FixPositionConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const {
    vector_t target_joint_pos(6);
    scalar_t joint1,joint2,joint3,joint4,joint5,joint6;
    // //sin wave according to time
    // joint1 = 0.5*sin(3*time);
    // joint2 = 0.5*sin(3*time)+0.5;
    // joint3 = 0.5*sin(3*time)+0.5;
    // joint4 = 1.5*sin(3*time);
    // joint5 = 1.5*sin(3*time);
    // joint6 = 1.5*sin(3*time);
    joint1 = 1.0;
    joint2 = 1.0;
    joint3 = 1.0;
    joint4 = 1.0;
    joint5 = 1.0;
    joint6 = 1.0;
    target_joint_pos << joint1, joint2, joint3, joint4, joint5, joint6;
    vector_t arm_joint_pos = state.tail(6)-target_joint_pos;
    return arm_joint_pos;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation FixPositionConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx(6, state.rows(), 0);
  approx.f = getValue(time, state, preComp);
  approx.dfdx = matrix_t::Zero(6, state.size());
  approx.dfdx.rightCols(6).diagonal() = vector_t::Ones(6);
  return approx;
}

}  // namespace legged_robot
}  // namespace ocs2
