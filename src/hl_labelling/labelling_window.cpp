#include <hl_labelling/labelling_window.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/manual_pose_solver.h>

#include <iostream>

using namespace hl_communication;
using namespace hl_monitoring;

namespace hl_labelling
{
LabellingWindow::LabellingWindow(std::unique_ptr<hl_monitoring::ReplayImageProvider> provider_,
                                 const std::string& window_name)
  : ReplayViewer(std::move(provider_), window_name, false)
{
  addBinding('c', "Run pose calibration for current frame", [this]() { this->startPoseCalibration(); });
  labelling_manager.importMetaData(provider->getMetaInformation());
}

void LabellingWindow::updateTime()
{
  bool valid_frame = false;
  while (playing && !valid_frame)
  {
    ReplayViewer::updateTime();
    FrameEntry frame = provider->getFrameEntry(now);
    // TODO: depending on labelling mode, it might be interesting to tag STATIC frames as well
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
  setProtobufFromAffine(labelling_manager.getCorrectedCameraPose(frame_ts), information.mutable_pose());
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
    hl_communication::LabelMsg label;
    Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
    label.set_frame_index(frame_index);
    for (const Match2D3DMsg& match : matches)
    {
      label.add_field_matches()->CopyFrom(match);
    }
    labelling_manager.pushMsg(label);
    labelling_manager.pushManualPose(frame_index, camera_from_world);
  }
}

}  // namespace hl_labelling
