#include <hl_monitoring/manual_pose_solver.h>
#include <hl_monitoring/monitoring_manager.h>
#include <rhoban_utils/history/history.h>

#include <Eigen/Geometry>
#include <opencv2/highgui.hpp>

using namespace hl_monitoring;

Eigen::Affine3d getAffineFromProtobuf(const Pose3D& pose)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d src_in_dst = Eigen::Vector3d::Zero();
  switch (pose.rotation_size())
  {
    case 0:
      break;
    case 3:
    {
      // Rodrigues vector case
      Eigen::Vector3d rotation_axis(pose.rotation(0), pose.rotation(1), pose.rotation(2));
      double angle = 0;
      if (rotation_axis.norm() > std::pow(10, -6))
      {
        angle = rotation_axis.norm();
        rotation_axis.normalize();
      }
      else
      {
        rotation_axis = Eigen::Vector3d::UnitZ();
      }
      rotation = Eigen::AngleAxisd(angle, rotation_axis);
      break;
    }
    case 4:
    {
      Eigen::Quaterniond q(pose.rotation(0), pose.rotation(1), pose.rotation(2), pose.rotation(3));
      rotation = Eigen::Matrix3d(q);
      break;
    }
    default:
      throw std::runtime_error(DEBUG_INFO + " invalid size for rotation: " + std::to_string(pose.rotation_size()));
  }
  switch (pose.translation_size())
  {
    case 0:
      break;
    case 3:
      for (int dim = 0; dim < 3; dim++)
      {
        src_in_dst(dim) = pose.translation(dim);
      }
      break;
    default:
      throw std::runtime_error(DEBUG_INFO + " invalid size for src_in_dst: " + std::to_string(pose.translation_size()));
  }
  return Eigen::Translation3d(src_in_dst) * Eigen::Affine3d(rotation);
}

// From kid_size:CameraState
void setProtobufFromAffine(const Eigen::Affine3d& affine, hl_monitoring::Pose3D* pose)
{
  Eigen::Vector3d src_in_dst = affine * Eigen::Vector3d::Zero();
  pose->clear_rotation();
  pose->clear_translation();
  Eigen::Quaterniond q(affine.linear());
  pose->add_rotation(q.w());
  pose->add_rotation(q.x());
  pose->add_rotation(q.y());
  pose->add_rotation(q.z());
  pose->add_translation(src_in_dst(0));
  pose->add_translation(src_in_dst(1));
  pose->add_translation(src_in_dst(2));
}

class RhobanLog
{
public:
  RhobanLog() : initial_pose_history(-1)
  {
    manager.loadConfig("replay.json");
    const VideoMetaInformation& meta_information = getImageProvider().getMetaInformation();
    for (const FrameEntry& frame_entry : meta_information.frames())
    {
      const Pose3D& pose = frame_entry.pose();
      Eigen::Affine3d camera_from_world = getAffineFromProtobuf(pose);
      initial_pose_history.pushValue(frame_entry.time_stamp(), camera_from_world);
    }
  }

  const ImageProvider& getImageProvider()
  {
    return manager.getImageProvider("camera");
  }

  Eigen::Affine3d getCameraPose(uint64_t timestamp)
  {
    bool has_prev(false), has_next(false);
    std::map<uint64_t, Pose3D>::iterator next_pose_it = manual_poses.upper_bound(timestamp);
    uint64_t prev_ts(0), next_ts(0);
    if (next_pose_it != manual_poses.end())
    {
      has_next = true;
      next_ts = next_pose_it->first;
      if (next_pose_it != manual_poses.begin())
      {
        next_pose_it--;
        prev_ts = next_pose_it->first;
        has_prev = true;
      }
    }
    else if (manual_poses.size() > 0)
    {
      prev_ts = manual_poses.rbegin()->first;
      has_prev = true;
    }
    if (!has_prev && !has_next)
    {
      return Eigen::Affine3d::Identity();
    }
    Eigen::Affine3d pred_from_prev, pred_from_next;
    if (has_prev)
    {
      Eigen::Affine3d prev_manual_pose = getAffineFromProtobuf(manual_poses.at(prev_ts));
      Eigen::Affine3d diff_from_prev = initial_pose_history.getDiff(prev_ts, timestamp);
      pred_from_prev = diff_from_prev * prev_manual_pose;
      if (!has_next)
      {
        return pred_from_prev;
      }
    }
    if (has_next)
    {
      Eigen::Affine3d next_manual_pose = getAffineFromProtobuf(manual_poses.at(next_ts));
      Eigen::Affine3d diff_from_next = initial_pose_history.getDiff(next_ts, timestamp);
      pred_from_next = diff_from_next * next_manual_pose;
      if (!has_prev)
      {
        return pred_from_next;
      }
    }
    uint64_t dt = next_ts - prev_ts;
    double w_prev = 1 - (timestamp - prev_ts) / (double)dt;
    return rhoban_utils::averageFrames(pred_from_prev, pred_from_next, 1 - w_prev);
  }

  void pushManualPose(uint64_t time_stamp, const Pose3D& pose)
  {
    manual_poses[time_stamp] = pose;
  }

  MonitoringManager manager;
  rhoban_utils::HistoryPose initial_pose_history;
  std::map<uint64_t, Pose3D> manual_poses;
};

int main()
{
  RhobanLog log;

  bool active = true;
  int idx = 0;
  while (active)
  {
    int64_t time_stamp = log.getImageProvider().getTimeStamp(idx);
    CalibratedImage calibrated_img = log.manager.getCalibratedImage("camera", time_stamp);
    cv::Mat display_img = calibrated_img.getImg().clone();
    CameraMetaInformation camera_meta = calibrated_img.getCameraInformation();
    Eigen::Affine3d pose = log.getCameraPose(time_stamp);
    setProtobufFromAffine(pose, camera_meta.mutable_pose());
    log.manager.getField().tagLines(camera_meta, &display_img, cv::Scalar(0, 0, 0), 2.0, 20);
    cv::imshow("display_img", display_img);
    char key = cv::waitKey(time_stamp);
    switch (key)
    {
      case 'q':
        active = false;
        break;
      case 'c':
      {
        ManualPoseSolver pose_solver(calibrated_img.getImg(), camera_meta.camera_parameters(), log.manager.getField());
        Pose3D manual_pose;
        bool success = pose_solver.solve(&manual_pose);
        if (success)
        {
          log.pushManualPose(time_stamp, manual_pose);
        }
        break;
      }
      case 'n':
        idx = std::min((int)log.getImageProvider().getNbFrames(), idx + 1);
        break;
      case 'p':
        idx = std::max(0, idx - 1);
        break;
      default:
        std::cout << "Key '" << key << "' is not handled" << std::endl;
    }
  }
}
