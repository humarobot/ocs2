#include "OfflineReferenceManager.hpp"

namespace ocs2 {
namespace legged_robot {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule loadModeSchedule(const std::string& referenceFile, const std::string& variableName, bool verbose) {
  std::vector<scalar_t> eventTimes;
  loadData::loadStdVector(referenceFile, variableName + ".eventTimes", eventTimes, verbose);

  std::vector<std::string> modeSequenceString;
  loadData::loadStdVector(referenceFile, variableName + ".modeSequence", modeSequenceString, verbose);

  if (modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSchedule] failed to load : " + variableName + " from " + referenceFile);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(legged_robot::string2ModeNumber(modeName));
  }

  return {eventTimes, modeSequence};
}

OfflineReferenceManager::OfflineReferenceManager(const std::string& referenceFile,
                                                 std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
    : ReferenceManagerDecorator(std::move(referenceManagerPtr)), referenceFile_(referenceFile) {
  // Set target trajectories
  scalar_array_t desiredTimeTrajectory(2);
  vector_array_t desiredStateTrajectory(2);
  vector_array_t desiredInputTrajectory(2);
  desiredTimeTrajectory[0] = 0.0;
  desiredTimeTrajectory[1] = 1.0;
  desiredStateTrajectory[0] = vector_t::Zero(24);
  desiredStateTrajectory[1] = vector_t::Zero(24);
  desiredInputTrajectory[0] = vector_t::Zero(24);
  desiredInputTrajectory[1] = vector_t::Zero(24);
  desiredStateTrajectory[0][8] = 0.5;
  desiredStateTrajectory[1][8] = 0.5;
  desiredStateTrajectory[0].segment(12, 12) << -0.05, 0.72, -1.44, -0.05, 0.72, -1.44, 0.05, 0.72, -1.44, 0.05, 0.72,
      -1.44;

  desiredStateTrajectory[1].segment(12, 12) << -0.05, 0.72, -1.44, -0.05, 0.72, -1.44, 0.05, 0.72, -1.44, 0.05, 0.72,
      -1.44;
  setTargetTrajectories(TargetTrajectories(desiredTimeTrajectory, desiredStateTrajectory, desiredInputTrajectory));
}


}  // namespace legged_robot
}  // namespace ocs2