#pragma once

#include <hl_labelling/gtkmm/labelling_display_widget.h>
#include <hl_labelling/gtkmm/labelling_bar.h>

namespace hl_labelling
{
class LabellingWidget : public Gtk::VBox
{
public:
  LabellingWidget(const hl_monitoring::Field& field);
  virtual ~LabellingWidget();

protected:
  LabellingManager manager;
  /**
   * Access to label manager and configuration of labelling mode
   */
  LabellingBar labelling_bar;

  /**
   * Source of information from the video point of view
   */
  LabellingDisplayWidget display_area;
};
}  // namespace hl_labelling
