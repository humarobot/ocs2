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

#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>

#include <ocs2_mobile_manipulator_example/MobileManipulatorDummyVisualization.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ros/init.h>

int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  // task files
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Robot Interface
  mobile_manipulator::MobileManipulatorInterface interface(taskFileFolderName);

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<mobile_manipulator::MobileManipulatorDummyVisualization> dummyVisualization(
      new mobile_manipulator::MobileManipulatorDummyVisualization(nodeHandle));

  // Dummy MRT
  ocs2::MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  dummy.subscribeObservers({dummyVisualization});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(mobile_manipulator::INPUT_DIM);
  initObservation.time = 0.0;

  // initial command
  ocs2::vector_t initTarget(6);
  initTarget << 0, 0, 0, 0, 1, 1;
  ocs2::vector_t zeroInput = ocs2::vector_t::Zero(mobile_manipulator::INPUT_DIM);
  const ocs2::CostDesiredTrajectories initCostDesiredTrajectories({initObservation.time}, {initTarget}, {zeroInput});

  // Run dummy (loops while ros is ok)
  dummy.run(initObservation, initCostDesiredTrajectories);

  // Successful exit
  return 0;
}
