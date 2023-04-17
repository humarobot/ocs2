#include "ros/ros.h"
#include "lion_interface.hpp"
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

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

  // Create the robot interface node
//   RobotInterfaceNode robotInterfaceNode(nh, nh_private, robotInterface);

  // Run the node
  ros::spin();

  return 0;
}