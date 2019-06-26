#include <hl_labelling/labelling_manager.h>

#include <hl_communication/utils.h>
#include <hl_labelling/utils.h>
#include <robot_model/camera_model.h>

using namespace hl_communication;

namespace hl_labelling
{
LabellingManager::LabellingManager(double ball_radius_) : ball_radius(ball_radius_)
{
}

void LabellingManager::push(const hl_communication::VideoSourceID& source_id, const hl_communication::LabelMsg& label)
{
  push(source_id, label, true);
}

void LabellingManager::push(const hl_communication::VideoSourceID& source_id, const hl_communication::LabelMsg& label,
                            bool update_histories)
{
  managers.at(source_id).pushMsg(label);
  if (update_histories)
  {
    bool has_field_matches = label.field_matches_size() > 0;
    bool has_balls = label.balls_size() > 0;
    bool has_robots = label.robots_size() > 0;
    if (has_field_matches)
    {
      // Only current source require synchronization on poses
      managers.at(source_id).syncPoses();
    }
    if (has_field_matches || has_balls)
    {
      syncBalls();
    }
    if (has_field_matches || has_robots)
    {
      syncRobots();
    }
  }
}
void LabellingManager::pushManualPose(const hl_communication::VideoSourceID& source_id, int frame_index,
                                      const Eigen::Affine3d& camera_from_world)
{
  managers[source_id].pushManualPose(frame_index, camera_from_world);
}

void LabellingManager::importLabels(const MovieLabelCollection& movie)
{
  managers[movie.source_id()].importLabels(movie);
}

void LabellingManager::importLabels(const GameLabelCollection& game_labels)
{
  for (const auto& movie : game_labels.movies())
  {
    importLabels(movie);
  }
}

void LabellingManager::exportLabels(const hl_communication::VideoSourceID& source_id, MovieLabelCollection* movie)
{
  managers.at(source_id).exportLabels(movie);
}

void LabellingManager::exportLabels(GameLabelCollection* game_labels)
{
  for (const auto& entry : managers)
  {
    exportLabels(entry.first, game_labels->add_movies());
  }
}

void LabellingManager::importMetaData(const hl_communication::VideoMetaInformation& meta_information)
{
  managers[meta_information.source_id()].importMetaData(meta_information);
}

Eigen::Affine3d LabellingManager::getCameraPose(const hl_communication::VideoSourceID& source_id, uint64_t utc_ts)
{
  return managers.at(source_id).getCorrectedCameraPose(utc_ts);
}

std::map<int, Eigen::Vector3d> LabellingManager::getBalls(uint64_t timestamp) const
{
  std::map<int, Eigen::Vector3d> result;
  for (const auto& entry : balls_in_field)
  {
    result[entry.first] = entry.second->interpolate(timestamp);
    // TODO: take validity into account
  }
  return result;
}

std::map<RobotIdentifier, Eigen::Vector3d> LabellingManager::getRobots(uint64_t timestamp) const
{
  std::map<RobotIdentifier, Eigen::Vector3d> result;
  for (const auto& entry : robots_in_field)
  {
    result[entry.first] = entry.second->interpolate(timestamp);
    // TODO: take validity into account
  }
  return result;
}

void LabellingManager::sync()
{
  // Start by sync poses, because others depend on it
  syncPoses();
  syncBalls();
  syncRobots();
}

void LabellingManager::syncPoses()
{
  for (auto& entry : managers)
  {
    entry.second.syncPoses();
  }
}

void LabellingManager::syncBalls()
{
  balls_in_field.clear();
  for (const auto& entry : managers)
  {
    const VideoSourceID source_id = entry.first;
    const hl_communication::VideoMetaInformation& meta_information = entry.second.getMetaInformation();
    rhoban::CameraModel camera_model = intrinsicParametersToCameraModel(meta_information.camera_parameters());
    // Height of the ball is determined according to the ball radius
    rhoban_geometry::Plane ball_plane_in_field(Eigen::Vector3d::UnitZ(), ball_radius);
    // TODO: import balls + pose
    for (const auto& ball_entry : entry.second.getBallLabels())
    {
      int frame_index = ball_entry.first;
      uint64_t utc_ts = getTimeStamp(meta_information, frame_index);
      Eigen::Affine3d camera_from_field = getCameraPose(source_id, utc_ts);
      for (const BallMsg& ball : ball_entry.second)
      {
        if (!ball.has_ball_id())
          throw std::logic_error(HL_DEBUG + "cannot use a ball without ball_id");
        int ball_id = ball.ball_id();
        cv::Point2f img_pos(ball.center().x(), ball.center().y());
        Eigen::Vector3d ball_in_field;
        bool success = camera_model.getPosFromPixelAndPlane(img_pos, ball_plane_in_field, &ball_in_field,
                                                            camera_from_field.inverse());
        if (success)
        {
          if (balls_in_field.count(ball_id) == 0)
          {
            balls_in_field[ball_id] =
                std::unique_ptr<rhoban_utils::HistoryVector3d>(new rhoban_utils::HistoryVector3d(-1));
          }
          balls_in_field[ball_id]->pushValue(utc_ts, ball_in_field, false);
        }
        else
        {
          std::cerr << "Failed to get position of ball at " << img_pos << std::endl;
        }
      }
    }
  }
}

void LabellingManager::syncRobots()
{
  robots_in_field.clear();
  rhoban_geometry::Plane ground_plane_in_field(Eigen::Vector3d::UnitZ(), 0.0);

  for (const auto& entry : managers)
  {
    const VideoSourceID source_id = entry.first;
    if (!source_id.has_robot_source())
    {
      continue;
    }
    const RobotIdentifier& robot_id = source_id.robot_source().robot_id();
    const hl_communication::VideoMetaInformation& meta_information = entry.second.getMetaInformation();
    // Get robots based on projection of their camera on the ground
    for (int frame_idx = 0; frame_idx < meta_information.frames_size(); frame_idx++)
    {
      uint64_t utc_ts = getTimeStamp(meta_information, frame_idx);
      Eigen::Affine3d camera_from_field = getCameraPose(source_id, utc_ts);
      // Currently, the position of the robot is the position of the camera projected in the field
      Eigen::Vector3d camera_in_field = camera_from_field.inverse() * Eigen::Vector3d::Zero();
      camera_in_field.z() = 0;
      if (robots_in_field.count(robot_id) == 0)
      {
        robots_in_field[robot_id] =
            std::unique_ptr<rhoban_utils::HistoryVector3d>(new rhoban_utils::HistoryVector3d(-1));
      }
      robots_in_field[robot_id]->pushValue(utc_ts, camera_in_field, false);
    }
    // Get robots based on explicit human tags
    for (const auto& robot_entry : entry.second.getRobotLabels())
    {
      int frame_index = robot_entry.first;
      const hl_communication::VideoMetaInformation& meta_information = entry.second.getMetaInformation();
      rhoban::CameraModel camera_model = intrinsicParametersToCameraModel(meta_information.camera_parameters());
      uint64_t utc_ts = getTimeStamp(meta_information, frame_index);
      Eigen::Affine3d camera_from_field = getCameraPose(source_id, utc_ts);
      for (const RobotMessage& robot : robot_entry.second)
      {
        if (!robot.has_robot_id())
          throw std::logic_error(HL_DEBUG + "cannot use a robot without robot_id");
        const RobotIdentifier& robot_id = robot.robot_id();
        cv::Point2f img_pos(robot.ground_position().x(), robot.ground_position().y());
        Eigen::Vector3d robot_in_field;
        bool success = camera_model.getPosFromPixelAndPlane(img_pos, ground_plane_in_field, &robot_in_field,
                                                            camera_from_field.inverse());
        if (success)
        {
          if (robots_in_field.count(robot_id) == 0)
          {
            robots_in_field[robot_id] =
                std::unique_ptr<rhoban_utils::HistoryVector3d>(new rhoban_utils::HistoryVector3d(-1));
          }
          robots_in_field[robot_id]->pushValue(utc_ts, robot_in_field, false);
        }
        else
        {
          std::cerr << "Failed to get position of robot at " << img_pos << std::endl;
        }
      }
    }
  }
}

void LabellingManager::summarize(std::ostream* out) const
{
  (*out) << "LABELLING MANAGER CONTENT" << std::endl;
  (*out) << "-------------------------" << std::endl;
  for (const auto& entry : managers)
  {
    (*out) << "#" << entry.first << std::endl;
    entry.second.summarize(out);
  }
}

}  // namespace hl_labelling
