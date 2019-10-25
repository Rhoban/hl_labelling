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

bool LabellingManager::hasSource(const hl_communication::VideoSourceID& source_id) const
{
  return managers.count(source_id) > 0;
}
void LabellingManager::push(const hl_communication::VideoSourceID& source_id, const hl_communication::LabelMsg& label)
{
  push(source_id, label, true);
}

void LabellingManager::push(const hl_communication::VideoSourceID& source_id, const hl_communication::LabelMsg& label,
                            bool update_histories)
{
  managers.at(source_id).pushMsg(label);
  for (const RobotMessage& robot : label.robots())
    if (robot.has_robot_id() && robot.has_robot_color())
      robot_colors.pushColor(robot.robot_id(), robot.robot_color());
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

LabelMsg LabellingManager::getHistoryBasedLabel(uint64_t utc_ts)
{
  LabelMsg msg;
  for (const auto& ball : getBalls(utc_ts))
  {
    BallMsg* ball_msg = msg.add_balls();
    ball_msg->set_ball_id(ball.first);
    ball_msg->set_radius(ball_radius);
    cvToProtobuf(eigen2CV(ball.second), ball_msg->mutable_center_in_field());
  }
  for (const auto& robot : getRobots(utc_ts))
  {
    RobotMessage* robot_msg = msg.add_robots();
    robot_msg->mutable_robot_id()->CopyFrom(robot.first);
    cvToProtobuf(eigen2CV(robot.second), robot_msg->mutable_robot_in_field());
    robot_msg->set_robot_color(robot_colors.getColor(robot.first));
  }
  return msg;
}

LabelMsg LabellingManager::getLabel(const hl_communication::VideoSourceID& source_id, int frame_idx) const
{
  return managers.at(source_id).getLabel(frame_idx);
}

void LabellingManager::pushManualPose(const hl_communication::VideoSourceID& source_id, int frame_index,
                                      const Eigen::Affine3d& camera_from_world)
{
  managers[source_id].pushManualPose(frame_index, camera_from_world);
}

void LabellingManager::clearBall(int id)
{
  for (auto& entry : managers)
  {
    entry.second.clearBall(id);
  }
}

void LabellingManager::clearAllBalls()
{
  for (auto& entry : managers)
  {
    entry.second.clearAllBalls();
  }
}

void LabellingManager::clearRobot(hl_communication::RobotIdentifier robot_to_delete)
{
  for (auto& entry : managers)
  {
    entry.second.clearRobot(robot_to_delete);
  }
}

void LabellingManager::clearAllRobots()
{
  for (auto& entry : managers)
  {
    entry.second.clearAllRobots();
  }
}

void LabellingManager::importLabels(const std::string& label_path)
{
  hl_communication::GameLabelCollection labels;
  hl_communication::readFromFile(label_path, &labels);
  importLabels(labels);
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

void LabellingManager::exportCorrectedFrame(const hl_communication::VideoSourceID& source_id, uint64_t utc_ts,
                                            hl_communication::CameraMetaInformation* dst)
{
  managers.at(source_id).exportCorrectedFrame(utc_ts, dst);
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
    std::cout << entry.first << " has " << entry.second.getBallLabels().size() << " nb balls" << std::endl;
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
    // Get robots based on projection of their camera on the ground
    if (source_id.has_robot_source())
    {
      const RobotIdentifier& robot_id = source_id.robot_source().robot_id();
      const hl_communication::VideoMetaInformation& meta_information = entry.second.getMetaInformation();
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

void LabellingManager::analyze(std::ostream* out) const
{
  std::cout << "nb balls: " << balls_in_field.size() << std::endl;
  for (const auto& entry : balls_in_field)
  {
    (*out) << "Ball : " << entry.first << std::endl;
    for (const auto& ball : entry.second->getValues())
    {
      uint64_t ball_ts = ball.first / std::pow(10, 6);
      std::cout << "\t" << ball_ts << " -> " << ball.second.transpose() << std::endl;
    }
  }
}

}  // namespace hl_labelling
