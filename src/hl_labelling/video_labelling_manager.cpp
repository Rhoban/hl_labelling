#include <hl_labelling/video_labelling_manager.h>
#include <hl_communication/labelling_utils.h>
#include <hl_communication/utils.h>
#include <hl_monitoring/manual_pose_solver.h>
#include <hl_labelling/utils.h>

#include <iostream>

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
  if (relative_pose_history.size() == 0 && meta_information.has_default_pose())
  {
    return getAffineFromProtobuf(meta_information.default_pose());
  }
  if (manual_poses.size() == 0)
  {
    return relative_pose_history.interpolate(timestamp);
  }
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

void VideoLabellingManager::exportCorrectedFrame(uint64_t timestamp, hl_communication::CameraMetaInformation* dst)
{
  *(dst->mutable_camera_parameters()) = meta_information.camera_parameters();
  Eigen::Affine3d camera_pose = getCorrectedCameraPose(timestamp);
  setProtobufFromAffine(camera_pose, dst->mutable_pose());
}

const hl_communication::VideoMetaInformation& VideoLabellingManager::getMetaInformation() const
{
  return meta_information;
}

void VideoLabellingManager::syncPoses()
{
  // TODO: currently done automatically
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
    if (frame_entry.has_pose())
    {
      const Pose3D& pose = frame_entry.pose();
      Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
      relative_pose_history.pushValue(frame_entry.utc_ts(), camera_from_world);
    }
  }
}

void VideoLabellingManager::pushMsg(const LabelMsg& msg)
{
  if (!msg.has_frame_index())
    throw std::logic_error(HL_DEBUG + "message has no frame index");
  uint32_t frame_index = msg.frame_index();
  exportLabel(msg, &(labels[frame_index]));
}

void VideoLabellingManager::clearBall(int id)
{
  for (auto& entry : labels)
  {
    google::protobuf::RepeatedPtrField<BallMsg>* repeatedField = entry.second.mutable_balls();

    for (google::protobuf::RepeatedPtrField<BallMsg>::iterator it = repeatedField->begin(); it != repeatedField->end();
         it++)
    {
      if (it->ball_id() == (size_t)id)
      {
        repeatedField->erase(it);
        break;
      }
    }
  }
}

void VideoLabellingManager::clearAllBalls()
{
  for (auto& entry : labels)
  {
    entry.second.clear_balls();
  }
}

void VideoLabellingManager::clearRobot(hl_communication::RobotIdentifier robot_to_delete)
{
  for (auto& entry : labels)
  {
    // hl_communication::RobotMessage rb = copyFrom(entry.second.robots());
    //    entry.second.clear_robots();
    /*   for (auto& entry_robot : entry.second.robots())
    {
      if (entry_robot)
      }*/
    google::protobuf::RepeatedPtrField<RobotMessage>* repeatedField = entry.second.mutable_robots();

    for (google::protobuf::RepeatedPtrField<RobotMessage>::iterator it = repeatedField->begin();
         it != repeatedField->end(); it++)
    {
      if (it->robot_id() == robot_to_delete)
      {
        repeatedField->erase(it);
        break;
      }
    }
  }
}

void VideoLabellingManager::clearAllRobots()
{
  for (auto& entry : labels)
  {
    entry.second.clear_robots();
  }
}

void VideoLabellingManager::pushManualPose(int frame_index, const Eigen::Affine3d& camera_from_field)
{
  manual_poses[frame_index] = camera_from_field;
}

void VideoLabellingManager::importLabels(const MovieLabelCollection& movie)
{
  // TODO: the 'video_meta_information' from MovieLabelCollection should be used, but how?
  if (movie.label_collections_size() > 1)
  {
    throw std::logic_error(HL_DEBUG + " multiple labelers detected, not supported now");
  }
  if (!movie.has_source_id())
  {
    throw std::logic_error(HL_DEBUG + " 'movie' has no source id");
  }
  if (!meta_information.has_source_id())
  {
    // If no meta_information was provided, use info from label collection
    if (movie.has_video_meta_information())
    {
      meta_information.CopyFrom(movie.video_meta_information());
      for (int idx = 0; idx < meta_information.frames_size(); idx++)
      {
        const FrameEntry& frame_entry = meta_information.frames(idx);
        if (frame_entry.has_pose())
        {
          Eigen::Affine3d camera_from_field = getAffineFromProtobuf(frame_entry.pose());
          relative_pose_history.pushValue(frame_entry.utc_ts(), camera_from_field);
        }
      }
    }
    else
    {
      meta_information.mutable_source_id()->CopyFrom(movie.source_id());
    }
  }
  else if (!MessageDifferencer::Equals(movie.source_id(), meta_information.source_id()))
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
}

void VideoLabellingManager::exportLabels(MovieLabelCollection* movie)
{
  movie->mutable_source_id()->CopyFrom(meta_information.source_id());
  exportCorrectedCamera(movie->mutable_video_meta_information());
  LabelCollection* label_collection = movie->add_label_collections();
  label_collection->mutable_labeler_identity()->set_nick_name("unknown");
  for (const auto& frame_entry : labels)
  {
    label_collection->add_labels()->CopyFrom(frame_entry.second);
  }
}

void VideoLabellingManager::exportCorrectedCamera(VideoMetaInformation* dst)
{
  dst->CopyFrom(meta_information);
  for (int idx = 0; idx < dst->frames_size(); idx++)
  {
    FrameEntry* frame = dst->mutable_frames(idx);
    Eigen::Affine3d camera_from_field = getCorrectedCameraPose(frame->utc_ts());
    setProtobufFromAffine(camera_from_field, frame->mutable_pose());
  }
}

std::map<int, std::vector<hl_communication::BallMsg>> VideoLabellingManager::getBallLabels() const
{
  std::map<int, std::vector<hl_communication::BallMsg>> result;
  for (const auto& entry : labels)
  {
    for (const BallMsg& ball : entry.second.balls())
    {
      result[entry.first].push_back(ball);
    }
  }
  return result;
}
std::map<int, std::vector<hl_communication::RobotMessage>> VideoLabellingManager::getRobotLabels() const
{
  std::map<int, std::vector<hl_communication::RobotMessage>> result;
  for (const auto& entry : labels)
  {
    for (const RobotMessage& robot : entry.second.robots())
    {
      result[entry.first].push_back(robot);
    }
  }
  return result;
}

void VideoLabellingManager::summarize(std::ostream* out) const
{
  *out << "metainfo: " << meta_information.frames_size() << " frames" << std::endl;
}

}  // namespace hl_labelling
