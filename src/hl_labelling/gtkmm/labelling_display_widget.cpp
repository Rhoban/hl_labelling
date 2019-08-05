#include <hl_labelling/gtkmm/labelling_display_widget.h>

namespace hl_labelling
{
LabellingDisplayWidget::LabellingDisplayWidget(LabellingManager* manager, const hl_monitoring::Field& field_)
  : MultiCameraWidget(), labelling_manager(manager), field(field_)
{
}

void LabellingDisplayWidget::annotateImg(const std::string& name)
{
  uint64_t timestamp = timestamp_by_image[name];
  hl_communication::VideoSourceID source_id = source_ids[name];
  hl_communication::CameraMetaInformation camera_information;
  if (labelling_manager->hasSource(source_id))
  {
    labelling_manager->exportCorrectedFrame(source_id, timestamp, &camera_information);
    field.tagLines(camera_information, &display_images[name], cv::Scalar(0, 0, 0), 1.0, 10);
  }
}
}  // namespace hl_labelling
