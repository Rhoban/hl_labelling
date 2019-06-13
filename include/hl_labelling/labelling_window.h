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

  void treatMouseEvent(int event, int x, int y, int flags) override;

  void updateMode(int new_mode);
  void updateObjectID(int new_id);
  static void onModeTrackbarUpdate(int new_pos, void* ptr);
  static void onObjectIDTrackbarUpdate(int new_id, void* ptr);

  int mode;

  int object_id;
};
}  // namespace hl_labelling
