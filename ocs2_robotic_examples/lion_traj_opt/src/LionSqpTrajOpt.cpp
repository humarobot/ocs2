#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

#include "LionInterface.hpp"
#include "LionVisualizer.hpp"
#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"
#include "ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h"
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

using namespace ocs2;
using namespace legged_robot;

scalar_t secs = 0;
scalar_t nsecs = 0;

void clockCallback(const rosgraph_msgs::Clock& msg) {
  std::cout << "Time: " << msg.clock << std::endl;
  secs = msg.clock.sec;
  nsecs = msg.clock.nsec;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lion_sqp_TO_node");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, urdfFile, referenceFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFile", referenceFile);
  auto clock_sub = nodeHandle.subscribe("clock", 1, clockCallback);
  auto mode_schedule_pub = nodeHandle.advertise<ocs2_msgs::mode_schedule>("legged_robot_mode_schedule", 1);
  auto mpc_target_pub = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>("legged_robot_mpc_target", 1);
  // Create the robot interface
  LionRobotInterface robotInterface(taskFile, urdfFile, referenceFile);

  // Gait receiver
  // auto gaitReceiverPtr = std::make_shared<OfflineGaitReceiver>(
  //     robotInterface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), referenceFile);

  

  // Offlinereferencemanager
  auto offlineReferenceManagerPtr =
      std::make_shared<OfflineReferenceManager>(referenceFile, robotInterface.getReferenceManagerPtr());

  // Create a multipuleshootingsolver
  MultipleShootingSolver solver(robotInterface.sqpSettings(), robotInterface.getOptimalControlProblem(),
                                robotInterface.getInitializer());
  // Set the reference manager
  solver.setReferenceManager(offlineReferenceManagerPtr);
  // solver.addSynchronizedModule(gaitReceiverPtr);

  // Solve the problem
  solver.run(0.0, robotInterface.getInitialState(), 3.5);
  PrimalSolution solution;
  solver.getPrimalSolution(0.0, &solution);
  // // Print solution's state
  // for(int i = 0; i < solution.stateTrajectory_.size(); i++){
  //   std::cout << "State " << i << ": " << solution.stateTrajectory_[i].transpose() << std::endl;
  // }
  // // Print solution's input
  // for(int i = 0; i < solution.stateTrajectory_.size(); i++){
  //   std::cout << "Input " << i << ": " << solution.inputTrajectory_[i].transpose() << std::endl;
  // }

  ros::spinOnce();
  // Visualization
  LionVisualizer lionVisualizer(solution, robotInterface.getCentroidalModelInfo(), nodeHandle);
  lionVisualizer.run();

  ModeSchedule mode_schedule;
  mode_schedule = loadModeSchedule(referenceFile, "trotModeSchedule", false);
  // every element in mode_schedule.eventTimes adds secs 
  for(int i = 0; i < mode_schedule.eventTimes.size(); i++){
    mode_schedule.eventTimes[i] += secs;
  }

  
  TargetTrajectories target_trajectories;
  target_trajectories.timeTrajectory = solution.timeTrajectory_;
  for(int i = 0; i < target_trajectories.timeTrajectory.size(); i++){
    target_trajectories.timeTrajectory[i] += secs;
  }
  target_trajectories.stateTrajectory = solution.stateTrajectory_;
  target_trajectories.inputTrajectory = solution.inputTrajectory_;
  // std::cerr<<target_trajectories<<std::endl;

  mode_schedule_pub.publish(ocs2::ros_msg_conversions::createModeScheduleMsg(mode_schedule));
  mpc_target_pub.publish(ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(target_trajectories));


  return 0;
}