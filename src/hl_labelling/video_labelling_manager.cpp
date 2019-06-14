#include <hl_labelling/video_labelling_manager.h>
#include <hl_communication/labelling_utils.h>
#include <hl_communication/utils.h>
#include <hl_monitoring/manual_pose_solver.h>
#include <hl_labelling/utils.h>
#include <robot_model/camera_model.h>

#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/message_differencer.h>

using namespace hl_communication;
using namespace hl_monitoring;

using google::protobuf::util::MessageDifferencer;

namespace hl_labelling
{
VideoLabellingManager::VideoLabellingManager() : relative_pose_history(-1)
{
}

int VideoLabellingManager::getPreviousPoseIdx(uint64_t timestamp)
{
  for (auto iter = manual_poses.rbegin(); iter != manual_poses.rend(); iter++)
  {
    int index = iter->first;
    if (getTimeStamp(meta_information, index) < timestamp)
    {
      return index;
    }
  }
  return -1;
}

int VideoLabellingManager::getNextPoseIdx(uint64_t timestamp)
{
  for (auto iter = manual_poses.begin(); iter != manual_poses.end(); iter++)
  {
    int index = iter->first;
    if (getTimeStamp(meta_information, index) >= timestamp)
    {
      return index;
    }
  }
  return -1;
}

Eigen::Affine3d VideoLabellingManager::getCorrectedCameraPose(uint64_t timestamp)
{
  int prev_idx = getPreviousPoseIdx(timestamp);
  int next_idx = getNextPoseIdx(timestamp);
  if (prev_idx < 0 && next_idx < 0)
  {
    return Eigen::Affine3d::Identity();
  }
  Eigen::Affine3d pred_from_prev, pred_from_next;
  uint64_t prev_ts(0), next_ts(0);
  if (prev_idx >= 0)
  {
    Eigen::Affine3d prev_manual_pose = manual_poses.at(prev_idx);
    prev_ts = getTimeStamp(meta_information, prev_idx);
    Eigen::Affine3d diff_from_prev = relative_pose_history.getDiff(prev_ts, timestamp);
    pred_from_prev = diff_from_prev * prev_manual_pose;
    if (next_idx < 0)
    {
      return pred_from_prev;
    }
  }
  if (next_idx >= 0)
  {
    Eigen::Affine3d next_manual_pose = manual_poses.at(next_idx);
    next_ts = getTimeStamp(meta_information, next_idx);
    Eigen::Affine3d diff_from_next = relative_pose_history.getDiff(next_ts, timestamp);
    pred_from_next = diff_from_next * next_manual_pose;
    if (prev_idx < 0)
    {
      return pred_from_next;
    }
  }
  uint64_t dt = next_ts - prev_ts;
  double w_next = (timestamp - prev_ts) / (double)dt;
  return rhoban_utils::averageFrames(pred_from_prev, pred_from_next, w_next);
}

void VideoLabellingManager::importMetaData(const hl_communication::VideoMetaInformation& new_meta_information)
{
  if (!new_meta_information.has_source_id())
  {
    throw std::runtime_error(HL_DEBUG + " no source id in meta_information");
  }
  meta_information = new_meta_information;
  for (const FrameEntry& frame_entry : meta_information.frames())
  {
    const Pose3D& pose = frame_entry.pose();
    Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
    relative_pose_history.pushValue(frame_entry.utc_ts(), camera_from_world);
  }
}

void VideoLabellingManager::pushMsg(const LabelMsg& msg, double ball_radius)
{
  if (!msg.has_frame_index())
    throw std::logic_error(HL_DEBUG + "message has no frame index");
  uint32_t frame_index = msg.frame_index();
  uint64_t utc_ts = getTimeStamp(meta_information, frame_index, true);
  exportLabel(msg, &(labels[frame_index]));
  for (const BallMsg& ball : msg.balls())
  {
    pushBall(utc_ts, ball, ball_radius);
  }
}

void VideoLabellingManager::pushManualPose(int frame_index, const Eigen::Affine3d& camera_from_field)
{
  manual_poses[frame_index] = camera_from_field;
}

void VideoLabellingManager::pushBall(uint64_t utc_ts, const BallMsg& ball, double ball_radius)
{
  if (!ball.has_ball_id())
    throw std::logic_error(HL_DEBUG + "cannot use a ball without ball_id");
  Eigen::Affine3d camera_from_field = getCorrectedCameraPose(utc_ts);
  rhoban::CameraModel camera_model = intrinsicParametersToCameraModel(meta_information.camera_parameters());
  // Height of the ball is determined according to the ball radius
  rhoban_geometry::Plane ball_plane_in_field(Eigen::Vector3d::UnitZ(), ball_radius);
  cv::Point2f img_pos(ball.center().x(), ball.center().y());
  Eigen::Vector3d ball_in_field;
  bool success =
      camera_model.getPosFromPixelAndPlane(img_pos, ball_plane_in_field, &ball_in_field, camera_from_field.inverse());
  if (success)
  {
    int ball_id = ball.ball_id();
    if (balls.count(ball_id) == 0)
    {
      balls[ball_id] = std::unique_ptr<rhoban_utils::HistoryVector3d>(new rhoban_utils::HistoryVector3d(-1));
    }
    balls[ball_id]->pushValue(utc_ts, ball_in_field);
  }
  else
  {
    std::cerr << "Failed to get position of ball at " << img_pos << std::endl;
  }
}

void VideoLabellingManager::importLabels(const MovieLabelCollection& movie, double ball_radius)
{
  if (movie.label_collections_size() > 1)
  {
    throw std::logic_error(HL_DEBUG + " muliple labelers detected, not supported now");
  }
  if (!movie.has_source_id())
  {
    throw std::logic_error(HL_DEBUG + " 'movie' has no source id");
  }
  if (!MessageDifferencer::Equals(movie.source_id(), meta_information.source_id()))
  {
    std::string movie_json, meta_json;
    google::protobuf::util::MessageToJsonString(movie.source_id(), &movie_json);
    google::protobuf::util::MessageToJsonString(meta_information.source_id(), &meta_json);
    throw std::runtime_error(HL_DEBUG + " Mismatch between label source_id " + movie_json +
                             " and meta_information source_id " + meta_json);
  }
  for (const LabelCollection& label_collection : movie.label_collections())
  {
    // First, load all pose and labels
    for (const LabelMsg& label : label_collection.labels())
    {
      if (!label.has_frame_index())
      {
        throw std::runtime_error(HL_DEBUG + " label without frame index received");
      }
      int frame_index = label.frame_index();
      labels[frame_index] = label;
      if (label.field_matches_size() >= 4)
      {
        std::vector<Match2D3DMsg> matches;
        for (const Match2D3DMsg& match : label.field_matches())
        {
          matches.push_back(match);
        }
        Pose3D pose;
        if (ManualPoseSolver::solvePose(matches, meta_information.camera_parameters(), &pose))
        {
          pushManualPose(frame_index, getAffineFromProtobuf(pose));
        }
        else
        {
          std::cout << "Failed to solve pose" << std::endl;
        }
      }
    }
  }
  // Finally update balls in field
  for (const auto& entry : labels)
  {
    uint64_t utc_ts = meta_information.frames(entry.first).utc_ts();
    for (const BallMsg& ball : entry.second.balls())
    {
      pushBall(utc_ts, ball, ball_radius);
    }
  }
}

void VideoLabellingManager::exportLabels(MovieLabelCollection* movie)
{
  movie->mutable_source_id()->CopyFrom(meta_information.source_id());
  LabelCollection* label_collection = movie->add_label_collections();
  label_collection->mutable_labeler_identity()->set_nick_name("unknown");
  for (const auto& frame_entry : labels)
  {
    label_collection->add_labels()->CopyFrom(frame_entry.second);
  }
}

std::map<int, Eigen::Vector3d> VideoLabellingManager::getBalls(uint64_t timestamp)
{
  std::map<int, Eigen::Vector3d> result;
  for (const auto& entry : balls)
  {
    result[entry.first] = entry.second->interpolate(timestamp);
  }
  return result;
}

}  // namespace hl_labelling
