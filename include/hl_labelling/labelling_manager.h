#pragma once

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
   * Add a label to the collection and synchronize histories as required
   */
  void push(const hl_communication::VideoSourceID& source_id, const hl_communication::LabelMsg& label);

  /**
   * Push a manual pose to the given source_id.
   * TODO: this should be removed and update of the pose handled inside the labelling_manager
   */
  void pushManualPose(const hl_communication::VideoSourceID& source_id, int frame_index,
                      const Eigen::Affine3d& camera_from_world);

  void clearBall(int id);
  void clearAllBalls();
  void clearRobot(hl_communication::RobotIdentifier robot_to_delete);
  void clearAllRobots();

  void importLabels(const hl_communication::MovieLabelCollection& movie);
  void importLabels(const hl_communication::GameLabelCollection& movie);
  void exportLabels(const hl_communication::VideoSourceID& source_id, hl_communication::MovieLabelCollection* movie);
  void exportLabels(hl_communication::GameLabelCollection* game_labels);

  /**
   * Imports the metadata of a video in order to provide relative history to its source.
   */
  void importMetaData(const hl_communication::VideoMetaInformation& meta_information);

  Eigen::Affine3d getCameraPose(const hl_communication::VideoSourceID& source_id, uint64_t utc_ts);

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

  /**
   * Each source has its own LabellingManager
   */
  std::map<hl_communication::VideoSourceID, VideoLabellingManager> managers;

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
   * Updates the position of the robots based on labels
   */
  void syncRobots();

  /**
   * if 'update_histories' is false, then synchronization between histories and labels has to be performed manually
   */
  void push(const hl_communication::VideoSourceID& source_id, const hl_communication::LabelMsg& label,
            bool update_histories);

  /**
   * Position of the balls in the field based on provided history
   */
  std::map<int, std::unique_ptr<rhoban_utils::HistoryVector3d>> balls_in_field;

  /**
   * Stores the validity status of the ball position
   */
  std::map<int, std::unique_ptr<rhoban_utils::HistoryBool>> balls_validity;

  /**
   * Position of the robots in the field based on provided history
   */
  std::map<hl_communication::RobotIdentifier, std::unique_ptr<rhoban_utils::HistoryVector3d>> robots_in_field;

  /**
   * Stores the validity status of the ball position
   */
  std::map<hl_communication::RobotIdentifier, std::unique_ptr<rhoban_utils::HistoryBool>> robots_validity;

  /**
   * Radius of the ball used in [m]
   */
  double ball_radius;
};
}  // namespace hl_labelling
