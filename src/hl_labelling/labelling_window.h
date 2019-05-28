#include <qt_monitoring/video_widget.h>

#include <hl_communication/labelling.pb.h>
#include <rhoban_utils/history/history.h>

#include <QGridLayout>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>

namespace hl_labelling
{
class LabellingWindow : public QMainWindow
{
  Q_OBJECT

public:
  LabellingWindow(std::unique_ptr<hl_monitoring::MonitoringManager> manager);

  void importLabels(const hl_communication::GameLabelCollection& game);
  void exportLabels(hl_communication::GameLabelCollection* game);

public slots:
  void update();
  void startPoseCalibration();

private:
  /**
   * Update the pose of all the frames between previous and next position from ts
   * 'ts' should be the exact timestamp of the provider which has just been updated
   */
  void propagatePoseChange(const std::string& provider_name, uint64_t ts);
  /**
   * Returns index of the previous pose, return -1 if there is no previous pose
   */
  int getPreviousPoseIdx(const std::string& provider_name, uint64_t timestamp);
  /**
   * Returns index of the next pose, return -1 if there is no next pose
   */
  int getNextPoseIdx(const std::string& provider_name, uint64_t timestamp);
  /**
   * Return the
   */
  Eigen::Affine3d getCorrectedCameraPose(const std::string& provider_name, uint64_t timestamp);
  /**
   * If exactly one source is available in video_widget, returns its name.
   * Otherwise, throw an error
   */
  std::string getUniqueSource();
  const hl_monitoring::VideoMetaInformation& getMetaInformation(const std::string& provider_name);
  /**
   * Reads the metadata from manager to populate the histories
   */
  void importMetaData();
  qt_monitoring::VideoWidget* video_widget;

  QPushButton* pose_calibration_button;

  QWidget* central_widget;
  QGridLayout* layout;

  /**
   * Timer ticking updates of the main_window
   */
  QTimer* timer;

  /**
   * Represent the information initially present in the VideoMetaInformation
   */
  std::map<std::string, std::unique_ptr<rhoban_utils::HistoryPose>> initial_pose_histories;

  /**
   * The existing labels ordered first by camera_source and then by frame_index
   */
  std::map<std::string, std::map<int, hl_communication::LabelMsg>> labels;

  /**
   * Poses obtained through manual estimation
   */
  std::map<std::string, std::map<int, Eigen::Affine3d>> manual_poses;
};
}  // namespace hl_labelling
