#include <hl_labelling/labelling_window.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/manual_pose_solver.h>
#include <robot_model/camera_model.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_communication;
using namespace hl_monitoring;

#define MODE_FIELD 0
#define MODE_BALL 1
#define MODE_ROBOT 2

namespace hl_labelling
{
LabellingWindow::LabellingWindow(std::unique_ptr<hl_monitoring::ReplayImageProvider> provider_,
                                 const std::string& window_name)
  : ReplayViewer(std::move(provider_), window_name, false), mode(0), object_id(0), team_id(0)
{
  addBinding('c', "Run pose calibration for current frame", [this]() { this->startPoseCalibration(); });
  labelling_manager.importMetaData(provider->getMetaInformation());
  cv::createTrackbar("mode", window_name, &mode, 2,
                     [](int new_value, void* ptr) { ((LabellingWindow*)ptr)->updateMode(new_value); }, this);
  cv::createTrackbar("objectID", window_name, &object_id, 6,
                     [](int new_value, void* ptr) { ((LabellingWindow*)ptr)->updateObjectID(new_value); }, this);
  cv::createTrackbar("teamID", window_name, &team_id, 30,
                     [](int new_value, void* ptr) { ((LabellingWindow*)ptr)->updateTeamID(new_value); }, this);
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
  switch (mode)
  {
    case MODE_FIELD:
      field.tagLines(information, &display_img, cv::Scalar(0, 0, 0), 1, 10);
      break;
    case MODE_BALL:
      for (const auto& entry : labelling_manager.getBalls(frame_ts))
      {
        int ball_id = entry.first;
        const Eigen::Vector3d& ball_in_field = entry.second;
        cv::Point2f ball_in_img;
        if (fieldToImg(eigen2CV(ball_in_field), information, &ball_in_img))
        {
          cv::circle(display_img, ball_in_img, 3, cv::Scalar(0, 0, 255), CV_FILLED, cv::LINE_AA);
        }
      }
      break;
  }
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
    labelling_manager.pushMsg(label, field.ball_radius);
    labelling_manager.pushManualPose(frame_index, camera_from_world);
  }
}

void LabellingWindow::treatMouseEvent(int event, int x, int y, int flags)
{
  (void)flags;
  if (mode == MODE_BALL && event == cv::EVENT_LBUTTONDOWN)
  {
    hl_communication::LabelMsg label;
    int frame_index = provider->getIndex(now);
    label.set_frame_index(frame_index);
    BallMsg* ball = label.add_balls();
    ball->mutable_center()->set_x(x);
    ball->mutable_center()->set_y(y);
    ball->set_ball_id(object_id);
    labelling_manager.pushMsg(label, field.ball_radius);
  }
}

void LabellingWindow::updateMode(int new_mode)
{
  mode = new_mode;
}

void LabellingWindow::updateObjectID(int new_id)
{
  object_id = new_id;
}

void LabellingWindow::updateTeamID(int new_id)
{
  object_id = new_id;
}

}  // namespace hl_labelling
