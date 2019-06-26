#pragma once
#include <hl_communication/labelling.pb.h>
#include <hl_communication/camera.pb.h>
#include <rhoban_utils/history/history.h>

namespace hl_labelling
{
/**
 * Manages all the labels of a single video, can include relative pose
 * information regarding the camera
 */
class VideoLabellingManager
{
public:
  VideoLabellingManager();

  void setSourceID(const hl_communication::VideoSourceID& new_id);

  /**
   * Reads the metadata from manager to populate the histories
   */
  void importMetaData(const hl_communication::VideoMetaInformation& meta_information);

  void pushMsg(const hl_communication::LabelMsg& msg);
  void pushManualPose(int frame_index, const Eigen::Affine3d& camera_from_field);

  /**
   * Remove all ball labels from the current labelCollection
   */
  void clearBalls();

  void importLabels(const hl_communication::MovieLabelCollection& movie);
  void exportLabels(hl_communication::MovieLabelCollection* movie);
  /**
   * Export video meta information after pose correction based on labels
   */
  void exportCorrectedCamera(hl_communication::VideoMetaInformation* dst);

  /**
   * Extract the ball labels from the current labels ordered by frame_index
   */
  std::map<int, std::vector<hl_communication::BallMsg>> getBallLabels() const;

  /**
   * Extract the robot labels ordered by frame_index
   */
  std::map<int, std::vector<hl_communication::RobotMessage>> getRobotLabels() const;

  /**
   * Return the pose of the camera in field referential based on two elements:
   * - Manual pose provided by tagging
   * - Camera relative poses (estimated by robot,imu,etc..)
   */
  Eigen::Affine3d getCorrectedCameraPose(uint64_t timestamp);

  const hl_communication::VideoMetaInformation& getMetaInformation() const;

  /**
   * Synchronize the poses of the camera with the field_matches
   */
  void syncPoses();

  /**
   * Writes a few properties on the output stream properties
   */
  void summarize(std::ostream* out) const;

private:
  /**
   * Returns index of the previous pose, return -1 if there is no previous pose
   */
  int getPreviousPoseIdx(uint64_t timestamp);
  /**
   * Returns index of the next pose, return -1 if there is no next pose
   */
  int getNextPoseIdx(uint64_t timestamp);

  /**
   * Represent the relative pose estimated by robot,imu,etc..
   */
  rhoban_utils::HistoryPose relative_pose_history;

  /**
   * Meta-information of the video associated to the labels
   */
  hl_communication::VideoMetaInformation meta_information;

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
