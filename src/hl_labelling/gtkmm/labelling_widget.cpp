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
  display_area.registerClickHandler([this](const hl_communication::VideoSourceID& source_id,
                                           const cv::Point2f& img_pos) { this->mouseClick(source_id, img_pos); });
  labelling_bar.signal_collection_changed().connect(sigc::mem_fun(this, &LabellingWidget::on_label_collection_update));
}

LabellingWidget::~LabellingWidget()
{
}

void LabellingWidget::mouseClick(const hl_communication::VideoSourceID& source_id, const cv::Point2f& img_pos)
{
  const hl_communication::VideoSourceID& detailed_source_id = display_area.getDetailedSourceID(source_id);
  hl_communication::LabelMsg label;
  label.set_frame_index(display_area.getFrameIndex(detailed_source_id));
  // int team_id = labelling_bar.getLabellingChooser().getTeamID();
  int obj_id = labelling_bar.getLabellingChooser().getObjID();
  switch (labelling_bar.getLabellingChooser().getObjectType())
  {
    case LabellingChooser::Ball:
    {
      hl_communication::BallMsg* ball = label.add_balls();
      ball->set_ball_id(obj_id);
      ball->mutable_center()->set_x(img_pos.x);
      ball->mutable_center()->set_y(img_pos.y);
      manager.push(detailed_source_id, label);
      display_area.step(false);
      std::cout << "adding a ball" << std::endl;
      break;
    }
    case LabellingChooser::Field:
      std::cout << "adding a field tag" << std::endl;
      break;
    case LabellingChooser::Robot:
      std::cout << "Robots are not supported now" << std::endl;
      break;
    default:
      std::cout << "unkown object type" << std::endl;
  }
  std::string source_name = display_area.getName(detailed_source_id);
  std::cout << "Click on " << source_name << " at " << img_pos << std::endl;
}

void LabellingWidget::on_label_collection_update()
{
  display_area.step(false);
}
}  // namespace hl_labelling
