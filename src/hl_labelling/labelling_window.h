#include <hl_communication/labelling.pb.h>
#include <hl_monitoring/replay_viewer.h>
#include <rhoban_utils/history/history.h>

namespace hl_labelling
{
class LabellingWindow : public hl_monitoring::ReplayViewer
{
public:
  LabellingWindow(std::unique_ptr<hl_monitoring::ReplayImageProvider> manager, const std::string& window_name);

  void paintImg() override;

  void importLabels(const hl_communication::MovieLabelCollection& movie);
  void exportLabels(hl_communication::MovieLabelCollection* movie);

  void startPoseCalibration();

private:
  /**
   * Update the pose of all the frames between previous and next position from ts
   * 'ts' should be the exact timestamp of the provider which has just been updated
   */
  void propagatePoseChange(uint64_t ts);
  /**
   * Propagate pose changes for [start_idx, end_idx[
   */
  void propagatePoseChange(int start_idx, int end_idx);
  /**
   * Returns index of the previous pose, return -1 if there is no previous pose
   */
  int getPreviousPoseIdx(uint64_t timestamp);
  /**
   * Returns index of the next pose, return -1 if there is no next pose
   */
  int getNextPoseIdx(uint64_t timestamp);
  /**
   * Return the
   */
  Eigen::Affine3d getCorrectedCameraPose(uint64_t timestamp);

  /**
   * Reads the metadata from manager to populate the histories
   */
  void importMetaData();

  /**
   * Represent the information initially present in the VideoMetaInformation
   */
  rhoban_utils::HistoryPose initial_pose_history;

  /**
   * The existing labels ordered first by camera_source and then by frame_index
   */
  std::map<int, hl_communication::LabelMsg> labels;

  /**
   * Poses obtained through manual estimation
   */
  std::map<int, Eigen::Affine3d> manual_poses;
};
}  // namespace hl_labelling
