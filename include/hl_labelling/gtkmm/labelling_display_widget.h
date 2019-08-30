#pragma once

#include <hl_monitoring/field.h>
#include <hl_monitoring/gtkmm/multi_camera_widget.h>
#include <hl_labelling/labelling_manager.h>

namespace hl_labelling
{
class LabellingDisplayWidget : public hl_monitoring::MultiCameraWidget
{
public:
  LabellingDisplayWidget(LabellingManager* manager, const hl_monitoring::Field& field);

  void annotateImg(const std::string& name) override;

protected:
  void addProvider(std::unique_ptr<hl_monitoring::ImageProvider> provider) override;

private:
  /**
   * Access to the labelling manager concerned by this labelling bar.
   * LabellingBar do not have ownership on this
   */
  LabellingManager* labelling_manager;
  /**
   * Access to the field allows to draw on the image
   */
  hl_monitoring::Field field;
};
}  // namespace hl_labelling
