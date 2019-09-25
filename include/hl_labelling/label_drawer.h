#pragma once

#include <hl_monitoring/drawers/drawer.h>

namespace hl_labelling
{
class LabelDrawer : public hl_monitoring::Drawer<hl_communication::LabelMsg>
{
  void draw(hl_monitoring::FieldToImgConverter c, const hl_communication::LabelMsg& data, cv::Mat* out) override;
};
}  // namespace hl_labelling
