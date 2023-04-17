#include "OfflineReferenceManager.hpp"

namespace ocs2 {
namespace legged_robot {

OfflineReferenceManager::OfflineReferenceManager(const std::string& modeScheduleFile,
                                                 const std::string& targetTrajectoriesFile)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()) {
  modeSchedulePtr_ = std::make_shared<ModeSchedule>(modeScheduleFile);
  swingTrajectoryPtr_ = std::make_shared<SwingTrajectoryPlanner>(targetTrajectoriesFile);
}

void OfflineReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                               TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  // Set the mode schedule
  modeSchedule = *modeSchedulePtr_;

  // Set the target trajectories
//   targetTrajectories = swingTrajectoryPtr_->getTargetTrajectories(initTime, finalTime, initState, modeSchedule);

  // Set the target swing foot trajectories
  const scalar_t terrainHeight = 0.0;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

}  //   namespace legged_robot
}  // namespace ocs2