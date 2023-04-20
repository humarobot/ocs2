#include "LionVisualizer.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace ocs2 {
namespace legged_robot {
LionVisualizer::LionVisualizer(const PrimalSolution& solution, CentroidalModelInfo centroidalModelInfo,
                               ros::NodeHandle& nodeHandle)
    : solution_(solution), centroidalModelInfo_(std::move(centroidalModelInfo)) {
  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("lion_description")) {
    std::cerr << "[LionVisualizer] Could not read URDF from: \"lion_description\"" << std::endl;
  } else {
    KDL::Tree kdlTree;
    kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LionVisualizer::run() {
  size_t numTimeSteps = solution_.timeTrajectory_.size();
  for (size_t i = 0; i < numTimeSteps; i++) {
    ros::Time timeStamp = ros::Time::now();
    SystemObservation observation;
    observation.state = solution_.stateTrajectory_[i];
    publishObservation(timeStamp, observation);
    ros::Duration(0.01).sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LionVisualizer::publishObservation(ros::Time timeStamp, const SystemObservation& observation) {
  // Extract components from state
  const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
  const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

  // Publish
  publishJointTransforms(timeStamp, qJoints);
  publishBaseTransform(timeStamp, basePose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LionVisualizer::publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const {
  if (robotStatePublisherPtr_ != nullptr) {
    std::map<std::string, scalar_t> jointPositions{
        {"LF_HAA", jointAngles[0]}, {"LF_HFE", jointAngles[1]},  {"LF_KFE", jointAngles[2]},
        {"LH_HAA", jointAngles[3]}, {"LH_HFE", jointAngles[4]},  {"LH_KFE", jointAngles[5]},
        {"RF_HAA", jointAngles[6]}, {"RF_HFE", jointAngles[7]},  {"RF_KFE", jointAngles[8]},
        {"RH_HAA", jointAngles[9]}, {"RH_HFE", jointAngles[10]}, {"RH_KFE", jointAngles[11]}};
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LionVisualizer::publishBaseTransform(ros::Time timeStamp, const vector_t& basePose) {
  if (robotStatePublisherPtr_ != nullptr) {
    geometry_msgs::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header = getHeaderMsg("odom", timeStamp);
    baseToWorldTransform.child_frame_id = "base";

    const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
    baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
    baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
    tfBroadcaster_.sendTransform(baseToWorldTransform);
  }
}
}  // namespace legged_robot
}  // namespace ocs2