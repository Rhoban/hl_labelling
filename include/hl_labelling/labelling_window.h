#pragma once
#include <hl_labelling/video_labelling_manager.h>
#include <hl_monitoring/replay_viewer.h>

namespace hl_labelling
{
class LabellingWindow : public hl_monitoring::ReplayViewer
{
public:
  LabellingWindow(std::unique_ptr<hl_monitoring::ReplayImageProvider> manager, const std::string& window_name);

  void paintImg() override;
  void startPoseCalibration();

  VideoLabellingManager labelling_manager;

protected:
  void updateTime() override;
};
}  // namespace hl_labelling
