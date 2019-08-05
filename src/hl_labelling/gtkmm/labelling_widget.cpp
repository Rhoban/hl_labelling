#include <hl_labelling/gtkmm/labelling_widget.h>

namespace hl_labelling
{
LabellingWidget::LabellingWidget(const hl_monitoring::Field& field)
  : manager(field.ball_radius), labelling_bar(&manager), display_area(&manager, field)
{
  add(labelling_bar);
  labelling_bar.show();
  add(display_area);
  display_area.show();
  display_area.registerClickHandler(
      [this](const std::string& source_name, const cv::Point2f& img_pos) { this->mouseClick(source_name, img_pos); });
}
LabellingWidget::~LabellingWidget()
{
}

void LabellingWidget::mouseClick(const std::string& source_name, const cv::Point2f& img_pos)
{
  std::cout << "Click on " << source_name << " at " << img_pos << std::endl;
}
}  // namespace hl_labelling
