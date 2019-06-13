#include <hl_labelling/video_labelling_manager.h>
#include <hl_communication/utils.h>
#include <hl_monitoring/manual_pose_solver.h>

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
    relative_pose_history.pushValue(frame_entry.monotonic_ts(), camera_from_world);
  }
}

void VideoLabellingManager::pushMsg(const LabelMsg& msg)
{
  if (!msg.has_frame_index())
    throw std::logic_error(HL_DEBUG + "message has no frame index");
  if (labels.count(msg.frame_index()) > 0)
    throw std::logic_error(HL_DEBUG + "merge of messages is not implemented yet");
  labels[msg.frame_index()] = msg;
}

void VideoLabellingManager::pushManualPose(int frame_index, const Eigen::Affine3d& camera_from_field)
{
  manual_poses[frame_index] = camera_from_field;
}

void VideoLabellingManager::importLabels(const MovieLabelCollection& movie)
{
  if (movie.label_collections_size() > 1)
  {
    throw std::logic_error(HL_DEBUG + " muliple labelers detected, not supported now");
  }
  if (!movie.has_source_id())
  {
    throw std::logic_error(HL_DEBUG + " 'movie' has no source id");
  }
  // TODO: check video_name
  if (MessageDifferencer::Equals(movie.source_id(), meta_information.source_id()))
  {
    throw std::runtime_error(HL_DEBUG + " Mismatch between label source_id and meta_information source_id");
  }
  for (const LabelCollection& label_collection : movie.label_collections())
  {
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
          manual_poses[frame_index] = getAffineFromProtobuf(pose);
        }
      }
    }
  }
}  // namespace hl_labelling

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

}  // namespace hl_labelling
