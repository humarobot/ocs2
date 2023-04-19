#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

#include "LionInterface.hpp"
#include "LionVisualizer.hpp"
#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"
#include "ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h"
#include "ros/ros.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  ros::init(argc, argv, "lion_sqp_TO_node");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, urdfFile, referenceFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFile", referenceFile);

  // Create the robot interface
  LionRobotInterface robotInterface(taskFile, urdfFile, referenceFile);

  // Offlinereferencemanager
  auto offlineReferenceManagerPtr =
      std::make_shared<OfflineReferenceManager>(referenceFile, robotInterface.getReferenceManagerPtr());

  // Create a multipuleshootingsolver
  MultipleShootingSolver solver(robotInterface.sqpSettings(), robotInterface.getOptimalControlProblem(),
                                robotInterface.getInitializer());
  // Set the reference manager
  solver.setReferenceManager(offlineReferenceManagerPtr);

  // Solve the problem
  solver.run(0.0, robotInterface.getInitialState(), 1.0);
  PrimalSolution solution;
  solver.getPrimalSolution(0.0, &solution);
  // Print solution's state
  // for(int i = 0; i < solution.stateTrajectory_.size(); i++){
  //   std::cout << "State " << i << ": " << solution.stateTrajectory_[i].transpose() << std::endl;
  // }
  // Visualization
  LionVisualizer lionVisualizer(solution, robotInterface.getCentroidalModelInfo(), nodeHandle);
  lionVisualizer.run();

  return 0;
}