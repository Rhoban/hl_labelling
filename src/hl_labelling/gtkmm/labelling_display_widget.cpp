#include <hl_labelling/gtkmm/labelling_display_widget.h>
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
  if (labelling_manager->hasSource(source_id))
  {
    hl_communication::CameraMetaInformation camera_information;
    labelling_manager->exportCorrectedFrame(source_id, timestamp, &camera_information);
    field.tagLines(camera_information, &display_img, cv::Scalar(0, 0, 0), 1.0, 10);
    // Draw balls
    for (const auto& entry : labelling_manager->getBalls(timestamp))
    {
      int ball_id = entry.first;
      cv::Scalar ball_color = cv::Scalar(0, 0, 255);
      const Eigen::Vector3d& ball_in_field = entry.second;
      cv::Point2f ball_in_img;
      if (fieldToImg(eigen2CV(ball_in_field), camera_information, &ball_in_img))
      {
        cv::circle(display_img, ball_in_img, 2, ball_color, CV_FILLED, cv::LINE_AA);
      }
      cv::putText(display_img, std::to_string(ball_id), ball_in_img, cv::FONT_HERSHEY_PLAIN, 1.0, ball_color, 1,
                  cv::LINE_AA);
    }
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
    for (const auto& entry : labelling_manager->getBalls(timestamp))
    {
      int ball_id = entry.first;
      cv::Scalar ball_color = cv::Scalar(0, 0, 255);
      const Eigen::Vector3d& ball_in_field = entry.second;
      cv::Point ball_in_img = top_view_drawer.getImgFromField(manager.getField(), eigen2CV(ball_in_field));
      cv::circle(display_img, ball_in_img, 2, ball_color, CV_FILLED, cv::LINE_AA);
      cv::putText(display_img, std::to_string(ball_id), ball_in_img, cv::FONT_HERSHEY_PLAIN, 1.0, ball_color, 1,
                  cv::LINE_AA);
    }
  }
  else
  {
    std::cout << "No labelling manager available for " << source_id << std::endl;
  }
}
void LabellingDisplayWidget::addProvider(std::unique_ptr<hl_monitoring::ImageProvider> provider)
{
  labelling_manager->importMetaData(provider->getMetaInformation());
  MultiCameraWidget::addProvider(std::move(provider));
}
}  // namespace hl_labelling
