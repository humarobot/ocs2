#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_legged_robot/gait/GaitSchedule.h"
#include "ocs2_legged_robot/gait/MotionPhaseDefinition.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Read the ModeSchedule and the TargetTrajectories from files.
 */
class OfflineReferenceManager : public ReferenceManager {
 public:
  OfflineReferenceManager(const std::string& modeScheduleFile, const std::string& targetTrajectoriesFile);

  ~OfflineReferenceManager() override = default;



  const std::shared_ptr<ModeSchedule>& getModeSchedule() { return modeSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

 private:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) override;

  std::shared_ptr<ModeSchedule> modeSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2