#include <hl_labelling/gtkmm/labelling_display_widget.h>
#include <hl_labelling/label_drawer.h>
#include <robot_model/camera_model.h>

#include <opencv2/imgproc.hpp>

namespace hl_labelling
{
LabellingDisplayWidget::LabellingDisplayWidget(LabellingManager* manager, const hl_monitoring::Field& field_)
  : MultiCameraWidget(), labelling_manager(manager), field(field_)
{
}

void LabellingDisplayWidget::annotateImg(const std::string& name)
{
  MultiCameraWidget::annotateImg(name);
  // TODO refactor -> cf personal note (LH)
  uint64_t timestamp = sources[name].timestamp;
  hl_communication::VideoSourceID source_id = sources[name].source_id;
  cv::Mat display_img = sources[name].display_image;
  LabelDrawer label_drawer;
  hl_communication::LabelMsg history_label = labelling_manager->getHistoryBasedLabel(timestamp);
  if (labelling_manager->hasSource(source_id))
  {
    int frame_idx = getFrameIndex(source_id);
    hl_communication::CameraMetaInformation camera_information;
    labelling_manager->exportCorrectedFrame(source_id, timestamp, &camera_information);
    field.tagLines(camera_information, &display_img, cv::Scalar(0, 0, 0), 1.0, 10);
    label_drawer.drawNatural(camera_information, history_label, &display_img);
    hl_communication::LabelMsg frame_label = labelling_manager->getLabel(source_id, frame_idx);
    label_drawer.drawNatural(camera_information, frame_label, &display_img);
    // Draw robots
    for (const auto& entry : labelling_manager->getRobots(timestamp))
    {
      cv::Scalar robot_color = cv::Scalar(255, 0, 255);
      const Eigen::Vector3d& robot_in_field = entry.second;
      cv::Point2f robot_in_img;
      if (fieldToImg(eigen2CV(robot_in_field), camera_information, &robot_in_img))
      {
        cv::circle(display_img, robot_in_img, 3, robot_color, CV_FILLED, cv::LINE_AA);
      }
      std::string text = std::to_string(entry.first.team_id()) + "," + std::to_string(entry.first.robot_id());
      cv::putText(display_img, text, robot_in_img, cv::FONT_HERSHEY_PLAIN, 1.0, robot_color, 1, cv::LINE_AA);
    }
  }
  else if (isTopViewID(source_id))
  {
    label_drawer.drawTopView(manager.getField(), top_view_drawer, history_label, &display_img);
  }
  else
  {
    std::cout << "No labelling manager available for " << source_id << std::endl;
    return;
  }
}
void LabellingDisplayWidget::addProvider(std::unique_ptr<hl_monitoring::ImageProvider> provider)
{
  labelling_manager->importMetaData(provider->getMetaInformation());
  MultiCameraWidget::addProvider(std::move(provider));
}
}  // namespace hl_labelling
