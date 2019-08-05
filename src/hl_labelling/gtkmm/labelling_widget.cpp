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
}
LabellingWidget::~LabellingWidget()
{
}
}  // namespace hl_labelling
