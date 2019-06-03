#include <hl_labelling/labelling_window.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/manual_pose_solver.h>
#include <hl_monitoring/utils.h>

#include <iostream>

using namespace hl_communication;
using namespace hl_monitoring;

namespace hl_labelling
{
LabellingWindow::LabellingWindow(std::unique_ptr<hl_monitoring::ReplayImageProvider> provider,
                                 const std::string& window_name)
  : ReplayViewer(std::move(provider), window_name, false), initial_pose_history(-1)
{
  addBinding('c', "Run pose calibration for current frame", [this]() { this->startPoseCalibration(); });
  importMetaData();
}

void LabellingWindow::updateTime()
{
  bool valid_frame = false;
  while (playing && !valid_frame)
  {
    ReplayViewer::updateTime();
    FrameEntry frame = provider->getFrameEntry(now);
    if (frame.has_status() && frame.status() == FrameStatus::MOVING)
    {
      valid_frame = true;
    }
  }
}

void LabellingWindow::paintImg()
{
  ReplayViewer::paintImg();
  int frame_index = provider->getIndex(now);
  uint64_t frame_ts = provider->getTimeStamp(frame_index);
  CameraMetaInformation information = calibrated_img.getCameraInformation();
  setProtobufFromAffine(getCorrectedCameraPose(frame_ts), information.mutable_pose());
  field.tagLines(information, &display_img, cv::Scalar(0, 0, 0), 1, 10);
}

void LabellingWindow::startPoseCalibration()
{
  int frame_index = provider->getIndex(now);
  uint64_t frame_ts = provider->getTimeStamp(frame_index);
  CalibratedImage calib_img = provider->getCalibratedImage(frame_ts);

  ManualPoseSolver solver(calib_img.getImg(), calib_img.getCameraInformation().camera_parameters(), field);
  Pose3D pose;
  std::vector<Match2D3DMsg> matches;
  if (solver.solve(&pose, &matches))
  {
    Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
    labels[frame_index].set_frame_index(frame_index);
    for (const Match2D3DMsg& match : matches)
    {
      labels[frame_index].add_field_matches()->CopyFrom(match);
    }
    std::cout << "Putting in frame " << frame_index << std::endl << camera_from_world.linear() << std::endl;
    manual_poses[frame_index] = camera_from_world;
    //    propagatePoseChange(frame_ts);
  }
}

void LabellingWindow::propagatePoseChange(uint64_t ts)
{
  int prev_idx = getPreviousPoseIdx(ts - 1);
  int next_idx = getNextPoseIdx(ts + 1);
  if (prev_idx < 0)
    prev_idx = 0;
  if (next_idx < 0)
    next_idx = provider->getNbFrames();
  propagatePoseChange(prev_idx, next_idx);
}

void LabellingWindow::propagatePoseChange(int start_idx, int end_idx)
{
  for (int frame_idx = start_idx; frame_idx < end_idx; frame_idx++)
  {
    uint64_t frame_ts = provider->getTimeStamp(frame_idx);
    Eigen::Affine3d pose = getCorrectedCameraPose(frame_ts);
    Pose3D protobuf_pose;
    setProtobufFromAffine(pose, &protobuf_pose);
    provider->setPose(frame_idx, protobuf_pose);
  }
}

int LabellingWindow::getPreviousPoseIdx(uint64_t timestamp)
{
  for (auto iter = manual_poses.rbegin(); iter != manual_poses.rend(); iter++)
  {
    int index = iter->first;
    if (provider->getTimeStamp(index) < timestamp)
    {
      return index;
    }
  }
  return -1;
}

int LabellingWindow::getNextPoseIdx(uint64_t timestamp)
{
  for (auto iter = manual_poses.begin(); iter != manual_poses.end(); iter++)
  {
    int index = iter->first;
    if (provider->getTimeStamp(index) >= timestamp)
    {
      return index;
    }
  }
  return -1;
}

Eigen::Affine3d LabellingWindow::getCorrectedCameraPose(uint64_t timestamp)
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
    prev_ts = provider->getTimeStamp(prev_idx);
    Eigen::Affine3d diff_from_prev = initial_pose_history.getDiff(prev_ts, timestamp);
    pred_from_prev = diff_from_prev * prev_manual_pose;
    if (next_idx < 0)
    {
      return pred_from_prev;
    }
  }
  if (next_idx >= 0)
  {
    Eigen::Affine3d next_manual_pose = manual_poses.at(next_idx);
    next_ts = provider->getTimeStamp(next_idx);
    Eigen::Affine3d diff_from_next = initial_pose_history.getDiff(next_ts, timestamp);
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

void LabellingWindow::importMetaData()
{
  const VideoMetaInformation& meta_information = provider->getMetaInformation();
  for (const FrameEntry& frame_entry : meta_information.frames())
  {
    const Pose3D& pose = frame_entry.pose();
    Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
    initial_pose_history.pushValue(frame_entry.time_stamp(), camera_from_world);
  }
}

void LabellingWindow::importLabels(const MovieLabelCollection& movie)
{
  if (movie.label_collections_size() > 1)
  {
    throw std::logic_error(HL_DEBUG + " muliple labelers detected, not supported now");
  }
  if (!movie.has_camera_name())
  {
    throw std::runtime_error(HL_DEBUG + " movie camera name missing");
  }
  if (movie.camera_name() != window_name)
  {
    throw std::runtime_error(HL_DEBUG + " invalid video name for label: '" + movie.camera_name() +
                             "' while expecting '" + window_name + "'");
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
        if (ManualPoseSolver::solvePose(matches, provider->getMetaInformation().camera_parameters(), &pose))
        {
          std::cout << "Putting in frame " << frame_index << std::endl
                    << getAffineFromProtobuf(pose).linear() << std::endl;
          manual_poses[frame_index] = getAffineFromProtobuf(pose);
        }
      }
    }
  }
  if (manual_poses.size() > 0)
  {
    propagatePoseChange(0, provider->getNbFrames());
  }
}

void LabellingWindow::exportLabels(MovieLabelCollection* movie)
{
  movie->set_camera_name(window_name);
  LabelCollection* label_collection = movie->add_label_collections();
  label_collection->mutable_labeler_identity()->set_nick_name("unknown");
  for (const auto& frame_entry : labels)
  {
    label_collection->add_labels()->CopyFrom(frame_entry.second);
  }
}

}  // namespace hl_labelling
