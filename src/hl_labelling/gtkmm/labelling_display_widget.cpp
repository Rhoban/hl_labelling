#include <hl_labelling/gtkmm/labelling_display_widget.h>

namespace hl_labelling
{
LabellingDisplayWidget::LabellingDisplayWidget(LabellingManager* manager, const hl_monitoring::Field& field_)
  : MultiCameraWidget(), labelling_manager(manager), field(field_)
{
}

void LabellingDisplayWidget::annotateImg(const std::string& name)
{
  uint64_t timestamp = sources[name].timestamp;
  hl_communication::VideoSourceID source_id = sources[name].source_id;
  if (labelling_manager->hasSource(source_id))
  {
    hl_communication::CameraMetaInformation camera_information;
    labelling_manager->exportCorrectedFrame(source_id, timestamp, &camera_information);
    field.tagLines(camera_information, &sources[name].display_image, cv::Scalar(0, 0, 0), 1.0, 10);
  }
}
}  // namespace hl_labelling
