#pragma once

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>

#include "ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_legged_robot/gait/GaitSchedule.h"
#include "ocs2_legged_robot/gait/MotionPhaseDefinition.h"
#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot{
/**
 * Decorates ReferenceManager using reference file to get ModeSchedule and TargetTrajectories.
 */
class OfflineReferenceManager : public ReferenceManagerDecorator {
 public:
  OfflineReferenceManager(const std::string& referenceFile,
                           std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr);
  ~OfflineReferenceManager() override = default;

 private:

  const std::string referenceFile_;
};

ModeSchedule loadModeSchedule(const std::string& referenceFile, const std::string& variableName, bool verbose = false);
// TODO using a proper way to read target trajecotries from a motion file
//  TargetTrajectories loadTargetTrajectories(const std::string& referenceFile, const std::string& variableName,
//                                            bool verbose = false);
}  // namespace legged_robot
}  // namespace ocs2