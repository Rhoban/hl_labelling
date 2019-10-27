#pragma once

#include <hl_communication/robot_color_map.h>
#include <hl_labelling/activable_pos_history.h>
#include <hl_labelling/video_labelling_manager.h>

namespace hl_labelling
{
/**
 * Manage manual labels from multiple sources and combines them to extract
 * position of objects on the field
 *
 * LabellingManager contains two types of data:
 * - labels: provided by users and stored in protobuf files, they define the
 *           position of elements in images
 * - histories: extracted from the labels, they provide information on the
 *              position of elements in the field referential
 *
 * Access to individual labelling manager is disabled in order to ensure that
 * histories are properly synchronized with labels
 */
class LabellingManager
{
public:
  /**
   * ball_radius [m] is required to estimate the position of the ball on the
   * field based on its position in the image
   */
  LabellingManager(double ball_radius);

  /**
   * Check if a source id exists
   */
  bool hasSource(const hl_communication::VideoSourceID& source_id) const;

  /**
   * Add a label to the collection and synchronize histories as required
   */
  void push(const hl_communication::VideoSourceID& source_id, const hl_communication::LabelMsg& label);

  /**
   * Creates a single LabelMsg based on all the known labels.
   */
  hl_communication::LabelMsg getHistoryBasedLabel(uint64_t utc_ts);

  /**
   * Return the manual label corresponding to the given frame_idx, if no label has been specified yet,
   * return an empty LabelMsg
   */
  hl_communication::LabelMsg getLabel(const hl_communication::VideoSourceID& source_id, int frame_idx) const;

  /**
   * Push a manual pose to the given source_id.
   * TODO: this should be removed and update of the pose handled inside the labelling_manager
   */
  void pushManualPose(const hl_communication::VideoSourceID& source_id, int frame_index,
                      const Eigen::Affine3d& camera_from_world);
  void clearBall(const hl_communication::VideoSourceID& source_id, int frame_index, size_t ball_id);
  void clearRobot(const hl_communication::VideoSourceID& source_id, int frame_index,
                  const hl_communication::RobotIdentifier& robot_id);

  void clearBall(int id);
  void clearAllBalls();
  void clearRobot(hl_communication::RobotIdentifier robot_to_delete);
  void clearAllRobots();

  /**
   * Label_path should contain GameLabelCollection message
   */
  void importLabels(const std::string& label_path);
  void importLabels(const hl_communication::MovieLabelCollection& movie);
  void importLabels(const hl_communication::GameLabelCollection& movie);
  void exportLabels(const hl_communication::VideoSourceID& source_id, hl_communication::MovieLabelCollection* movie);
  void exportLabels(hl_communication::GameLabelCollection* game_labels);

  /**
   * Imports the metadata of a video in order to provide relative history to its source.
   */
  void importMetaData(const hl_communication::VideoMetaInformation& meta_information);

  Eigen::Affine3d getCameraPose(const hl_communication::VideoSourceID& source_id, uint64_t utc_ts);
  void exportCorrectedFrame(const hl_communication::VideoSourceID& source_id, uint64_t utc_ts,
                            hl_communication::CameraMetaInformation* dst);
  /**
   * Return the positions of all labelled balls in the field at given timestamp
   */
  std::map<int, Eigen::Vector3d> getBalls(uint64_t timestamp) const;

  /**
   * Return the positions of all labelled robots at given timestamp
   */
  std::map<hl_communication::RobotIdentifier, Eigen::Vector3d> getRobots(uint64_t timestamp) const;

  /**
   * Synchronize all histories from the current labels
   */
  void sync();

  /**
   * Print a few stats in output stream
   */
  void summarize(std::ostream* out) const;

  /**
   * Print detailed stats on output stream
   */
  void analyze(std::ostream* out) const;

  void updateColors(const hl_communication::RobotColorMap& colors);

private:
  /**
   * Updates poses for all video labelling manager
   */
  void syncPoses();

  /**
   * Update the position of all the balls in the field
   */
  void syncBalls();

  /**
   * Updates the positon of the robots based on labels
   */
  void syncRobots();

  /**
   * if 'update_histories' is false, then synchronization between histories and labels has to be performed manually
   */
  void push(const hl_communication::VideoSourceID& source_id, const hl_communication::LabelMsg& label,
            bool update_histories);

  /**
   * Each source has its own LabellingManager
   */
  std::map<hl_communication::VideoSourceID, VideoLabellingManager> managers;

  /**
   * Position of the balls in the field based on provided history
   */
  std::map<int, std::unique_ptr<ActivablePosHistory>> balls_in_field;

  /**
   * Position of the robots in the field based on explicit labelling
   */
  std::map<hl_communication::RobotIdentifier, std::unique_ptr<ActivablePosHistory>> robots_by_tag;

  /**
   * Position of the robots based on the projection of their camera on the ground
   */
  std::map<hl_communication::RobotIdentifier, std::unique_ptr<ActivablePosHistory>> robots_by_camera_projection;

  hl_communication::RobotColorMap robot_colors;
  /**
   * Radius of the ball used in [m]
   */
  double ball_radius;
};
}  // namespace hl_labelling
