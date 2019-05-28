#include <hl_labelling/labelling_window.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/manual_pose_solver.h>
#include <qt_monitoring/globals.h>

#include <iostream>

using namespace hl_communication;
using namespace hl_monitoring;
using namespace qt_monitoring;

namespace hl_labelling
{
LabellingWindow::LabellingWindow(std::unique_ptr<hl_monitoring::MonitoringManager> manager)
{
  Globals::team_manager = manager->getTeamManager();

  setWindowTitle(tr("LabellingManager"));

  central_widget = new QWidget;
  layout = new QGridLayout;

  timer = new QTimer(this);
  video_widget = new VideoWidget(std::move(manager), timer);

  pose_calibration_button = new QPushButton("CalibratePose");
  connect(pose_calibration_button, SIGNAL(released()), this, SLOT(startPoseCalibration()));

  // TODO: update positions and make it easier to read
  layout->addWidget(pose_calibration_button, 0, 0, 1, 1);
  layout->addWidget(video_widget, 0, 1, 1, 1);

  central_widget->setLayout(layout);
  setCentralWidget(central_widget);

  timer->start(30);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  importMetaData();
  update();
}

void LabellingWindow::update()
{
  video_widget->update();
}

void LabellingWindow::startPoseCalibration()
{
  // TODO: stop video_widget
  std::string source_name = getUniqueSource();
  const ImageProvider& provider = video_widget->getManager().getImageProvider(source_name);
  int frame_index = provider.getIndex(video_widget->getTS());
  uint64_t frame_ts = provider.getTimeStamp(frame_index);
  CalibratedImage calib_img = video_widget->getCalibratedImage(source_name, frame_ts);

  ManualPoseSolver solver(calib_img.getImg(), calib_img.getCameraInformation().camera_parameters(),
                          video_widget->getManager().getField());
  Pose3D pose;
  std::vector<Match2D3DMsg> matches;
  if (solver.solve(&pose, &matches))
  {
    Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
    labels[source_name][frame_index].set_frame_index(frame_index);
    for (const Match2D3DMsg& match : matches)
    {
      labels[source_name][frame_index].add_field_matches()->CopyFrom(match);
    }
    manual_poses[source_name][frame_index] = camera_from_world;
    propagatePoseChange(source_name, frame_ts);
  }
}

void LabellingWindow::propagatePoseChange(const std::string& provider_name, uint64_t ts)
{
  const ImageProvider& provider = video_widget->getManager().getImageProvider(provider_name);
  int prev_idx = getPreviousPoseIdx(provider_name, ts - 1);
  int next_idx = getNextPoseIdx(provider_name, ts + 1);
  if (prev_idx < 0)
    prev_idx = 0;
  if (next_idx < 0)
    next_idx = provider.getNbFrames();
  for (int frame_idx = prev_idx; frame_idx < next_idx; frame_idx++)
  {
    uint64_t frame_ts = provider.getTimeStamp(frame_idx);
    Eigen::Affine3d pose = getCorrectedCameraPose(provider_name, frame_ts);
    Pose3D protobuf_pose;
    setProtobufFromAffine(pose, &protobuf_pose);
    video_widget->setPose(provider_name, frame_idx, protobuf_pose);
  }
}

int LabellingWindow::getPreviousPoseIdx(const std::string& provider_name, uint64_t timestamp)
{
  const ImageProvider& provider = video_widget->getManager().getImageProvider(provider_name);
  for (auto iter = manual_poses[provider_name].rbegin(); iter != manual_poses[provider_name].rend(); iter++)
  {
    int index = iter->first;
    if (provider.getTimeStamp(index) < timestamp)
    {
      return index;
    }
  }
  return -1;
}

int LabellingWindow::getNextPoseIdx(const std::string& provider_name, uint64_t timestamp)
{
  const ImageProvider& provider = video_widget->getManager().getImageProvider(provider_name);
  for (auto iter = manual_poses[provider_name].begin(); iter != manual_poses[provider_name].end(); iter++)
  {
    int index = iter->first;
    if (provider.getTimeStamp(index) >= timestamp)
    {
      return index;
    }
  }
  return -1;
}

Eigen::Affine3d LabellingWindow::getCorrectedCameraPose(const std::string& provider_name, uint64_t timestamp)
{
  int prev_idx = getPreviousPoseIdx(provider_name, timestamp);
  int next_idx = getNextPoseIdx(provider_name, timestamp);
  const std::map<int, Eigen::Affine3d>& provider_manual_poses = manual_poses[provider_name];
  const ImageProvider& provider = video_widget->getManager().getImageProvider(provider_name);
  if (prev_idx < 0 && next_idx < 0)
  {
    std::cout << "No previous nor next idx" << std::endl;
    return Eigen::Affine3d::Identity();
  }
  Eigen::Affine3d pred_from_prev, pred_from_next;
  uint64_t prev_ts(0), next_ts(0);
  if (prev_idx >= 0)
  {
    Eigen::Affine3d prev_manual_pose = provider_manual_poses.at(prev_idx);
    prev_ts = provider.getTimeStamp(prev_idx);
    Eigen::Affine3d diff_from_prev = initial_pose_histories[provider_name]->getDiff(prev_ts, timestamp);
    pred_from_prev = diff_from_prev * prev_manual_pose;
    std::cout << "prev_ts, timestamp: " << prev_ts << ", " << timestamp << std::endl;
    std::cout << "diff_from_prev: " << diff_from_prev.linear() << std::endl;
    if (next_idx < 0)
    {
      std::cout << "pred_from_prev" << std::endl;
      return pred_from_prev;
    }
  }
  if (next_idx >= 0)
  {
    Eigen::Affine3d next_manual_pose = provider_manual_poses.at(next_idx);
    next_ts = provider.getTimeStamp(next_idx);
    Eigen::Affine3d diff_from_next = initial_pose_histories[provider_name]->getDiff(next_ts, timestamp);
    std::cout << "diff_from_next: " << diff_from_next.linear() << std::endl;
    pred_from_next = diff_from_next * next_manual_pose;
    if (prev_idx < 0)
    {
      std::cout << "pred_from_next" << std::endl;
      return pred_from_next;
    }
  }
  uint64_t dt = next_ts - prev_ts;
  double w_prev = 1 - (timestamp - prev_ts) / (double)dt;
  return rhoban_utils::averageFrames(pred_from_prev, pred_from_next, 1 - w_prev);
}

std::string LabellingWindow::getUniqueSource()
{
  std::set<std::string> active_sources = video_widget->getStreamSelector().getActiveSources();
  if (active_sources.size() != 1)
  {
    throw std::logic_error(HL_DEBUG + " several sources are active");
  }
  return *active_sources.begin();
}

const VideoMetaInformation& LabellingWindow::getMetaInformation(const std::string& provider_name)
{
  return video_widget->getManager().getImageProvider(provider_name).getMetaInformation();
}

void LabellingWindow::importMetaData()
{
  for (const std::string& provider_name : video_widget->getManager().getImageProvidersNames())
  {
    const VideoMetaInformation& meta_information = getMetaInformation(provider_name);
    initial_pose_histories[provider_name].reset(new rhoban_utils::HistoryPose(-1));
    for (const FrameEntry& frame_entry : meta_information.frames())
    {
      const Pose3D& pose = frame_entry.pose();
      Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
      initial_pose_histories[provider_name]->pushValue(frame_entry.time_stamp(), camera_from_world);
    }
  }
}

void LabellingWindow::importLabels(const GameLabelCollection& game)
{
  for (const MovieLabelCollection& movie : game.movies())
  {
    if (movie.label_collections_size() > 1)
    {
      throw std::logic_error(HL_DEBUG + " muliple labelers detected, not supported now");
    }
    if (!movie.has_camera_name())
    {
      throw std::runtime_error(HL_DEBUG + " movie camera name missing");
    }
    for (const LabelCollection& label_collection : movie.label_collections())
    {
      for (const LabelMsg& label : label_collection.labels())
      {
        if (!label.has_frame_index())
        {
          throw std::runtime_error(HL_DEBUG + " label without frame index received");
        }
        const std::string& camera_name = movie.camera_name();
        int frame_index = label.frame_index();
        labels[camera_name][frame_index] = label;
        if (label.field_matches_size() >= 4)
        {
          std::vector<Match2D3DMsg> matches;
          for (const Match2D3DMsg& match : label.field_matches())
          {
            matches.push_back(match);
          }
          Pose3D pose;
          ManualPoseSolver::solvePose(matches, getMetaInformation(camera_name).camera_parameters(), &pose);
          manual_poses[camera_name][frame_index] = getAffineFromProtobuf(pose);
        }
      }
    }
  }
}

void LabellingWindow::exportLabels(GameLabelCollection* game)
{
  for (const auto& video_entry : labels)
  {
    std::string video_name = video_entry.first;
    MovieLabelCollection* movie = game->add_movies();
    movie->set_camera_name(video_name);
    LabelCollection* label_collection = movie->add_label_collections();
    label_collection->mutable_labeler_identity()->set_nick_name("unknown");
    for (const auto& frame_entry : video_entry.second)
    {
      label_collection->add_labels()->CopyFrom(frame_entry.second);
    }
  }
}

}  // namespace hl_labelling
