#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include "ocs2_centroidal_model/CentroidalModelInfo.h"
#include "ocs2_mpc/SystemObservation.h"
#include "ocs2_oc/oc_data/PrimalSolution.h"
#include "ocs2_ros_interfaces/visualization/VisualizationHelpers.h"
#include "ocs2_legged_robot/common/Types.h"
#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_robotic_tools/common/RotationTransforms.h"

namespace ocs2{
namespace legged_robot{

class LionVisualizer{
public:
    LionVisualizer(const PrimalSolution& solution,CentroidalModelInfo centroidalModelInfo, ros::NodeHandle& nodeHandle);
    ~LionVisualizer() = default;

    void run();
private:
    void publishObservation(ros::Time timeStamp, const SystemObservation& observation);
    void publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const;
    void publishBaseTransform(ros::Time timeStamp, const vector_t& basePose);

    const CentroidalModelInfo centroidalModelInfo_;
    const PrimalSolution solution_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;
};

}
}