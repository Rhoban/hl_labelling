#pragma once
#include <hl_labelling/labelling_manager.h>
#include <hl_monitoring/replay_viewer.h>

namespace hl_labelling
{
class LabellingWindow : public hl_monitoring::ReplayViewer
{
public:
  LabellingWindow(std::unique_ptr<hl_monitoring::ReplayImageProvider> manager, const std::string& window_name,
                  bool moving_frames_only);

  void paintImg() override;
  void startPoseCalibration();

  LabellingManager labelling_manager;

protected:
  void updateTime() override;

  void treatMouseEvent(int event, int x, int y, int flags) override;

  void updateTagMode(int new_mode);
  void updateViewMode(int new_mode);
  void updateObjectID(int new_id);
  void updateTeamID(int new_id);

  /**
   * Control the features painted on the image
   */
  int view_mode;

  /**
   * Control the features which are tagged
   */
  int tag_mode;

  /**
   * For both robots and balls
   */
  int object_id;

  /**
   * For robots
   */
  int team_id;

  /**
   * When enabled, all the 'static' and shaking frames are ignored
   */
  bool moving_frames_only;
};
}  // namespace hl_labelling
