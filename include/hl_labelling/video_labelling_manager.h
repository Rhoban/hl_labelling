#pragma once
#include <hl_communication/labelling.pb.h>
#include <hl_communication/camera.pb.h>
#include <rhoban_utils/history/history.h>

namespace hl_labelling
{
/**
 * Manages all the labels of a single video, can include relative pose information regarding the camera
 *
 * TODO: find another solution for ball_radius
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

  /**
   * ball_radius: [m]
   */
  void pushMsg(const hl_communication::LabelMsg& msg, double ball_radius);
  void pushManualPose(int frame_index, const Eigen::Affine3d& camera_from_field);
  void pushBall(uint64_t utc_ts, const hl_communication::BallMsg& ball, double ball_radius);

  void importLabels(const hl_communication::MovieLabelCollection& movie, double ball_radius);
  void exportLabels(hl_communication::MovieLabelCollection* movie);

  /**
   * Get the position of the balls in the field
   */
  std::map<int, Eigen::Vector3d> getBalls(uint64_t timestamp);

  /**
   * Return the pose of the camera in field referential based on two elements:
   * - Manual pose provided by tagging
   * - Camera relative poses (estimated by robot,imu,etc..)
   */
  Eigen::Affine3d getCorrectedCameraPose(uint64_t timestamp);

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
   * Position of the labelled balls in field referential
   * TODO: a bool history could be added to say if ball is present or not
   */
  std::map<int, std::unique_ptr<rhoban_utils::HistoryVector3d>> balls;

  /**
   * Poses obtained through manual estimation
   */
  std::map<int, Eigen::Affine3d> manual_poses;
};

}  // namespace hl_labelling
