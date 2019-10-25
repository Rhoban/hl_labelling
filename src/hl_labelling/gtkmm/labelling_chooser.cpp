#include <hl_labelling/gtkmm/labelling_chooser.h>

#include <hl_communication/game_controller_utils.h>
#include <string>
#include <map>
#include <sstream>
#include <iostream>

namespace hl_labelling
{
LabellingChooser::LabellingChooser()
{
  tree_content = Gtk::ListStore::create(tree_model);
  set_model(tree_content);
  pack_start(tree_model.col_name);
  addRow(ObjectType::Field);
  int max_balls = 6;
  for (int i = 0; i < max_balls; i++)
    addRow(ObjectType::Ball, -1, i);
  // int max_robots = 6;
  // for (int team : { 0, 1 })
  //   for (int robot_id = 1; robot_id <= max_robots; robot_id++)
  //     addRow(ObjectType::Robot, team, robot_id);
}

LabellingChooser::ObjectType LabellingChooser::getObjectType() const
{
  try
  {
    return getActiveRow()[tree_model.obj_type];
  }
  catch (const std::out_of_range& exc)
  {
    return ObjectType::None;
  }
}
int LabellingChooser::getTeamID() const
{
  try
  {
    return getActiveRow()[tree_model.team_id];
  }
  catch (const std::out_of_range& exc)
  {
    return -1;
  }
}
int LabellingChooser::getObjID() const
{
  try
  {
    return getActiveRow()[tree_model.obj_id];
  }
  catch (const std::out_of_range& exc)
  {
    return -1;
  }
}

LabellingChooser::TeamColor LabellingChooser::getTeamColor() const
{
  try
  {
    int team_color_code = getActiveRow()[tree_model.team_color];
    std::cout << "TCC: " << team_color_code << std::endl;
    return hl_communication::getTeamColor(team_color_code);
  }
  catch (const std::out_of_range& exc)
  {
    return TeamColor::UNKNOWN;
  }
}

void LabellingChooser::updateRobots(const hl_communication::RobotColorMap& robot_colors)
{
  // TODO: remove old robots lines
  for (const auto& entry : robot_colors)
  {
    int team_id = entry.first.team_id();
    int robot_id = entry.first.robot_id();
    std::cout << "Adding a robot: " << entry.first << " -> " << entry.second << std::endl;
    addRow(ObjectType::Robot, team_id, robot_id, entry.second);
  }
}

std::string LabellingChooser::objectTypeToStr(ObjectType type)
{
  switch (type)
  {
    case Field:
      return "Field";
    case Ball:
      return "Ball";
    case Robot:
      return "Robot";
    default:
      return "???";
  }
}
LabellingChooser::TreeModel::TreeModel()
{
  add(col_id);
  add(col_name);
  add(obj_type);
  add(team_id);
  add(team_color);
  add(obj_id);
}
void LabellingChooser::addRow(ObjectType obj_type, int team_id, int obj_id, TeamColor color)
{
  static int id = 0;
  Gtk::TreeModel::Row row = *tree_content->append();
  row[tree_model.obj_type] = obj_type;
  row[tree_model.team_id] = team_id;
  row[tree_model.obj_id] = obj_id;
  std::cout << "color: " << color << std::endl;
  std::cout << "(int)color: " << ((int)color) << std::endl;
  row[tree_model.team_color] = (int)color;
  std::ostringstream oss;
  oss << objectTypeToStr(obj_type);
  if (team_id >= 0)
    oss << "(team: " << team_id << ")";
  if (obj_id >= 0)
    oss << "(obj: " << obj_id << ")";
  row[tree_model.col_id] = id++;
  row[tree_model.col_name] = oss.str();
}

Gtk::TreeModel::Row LabellingChooser::getActiveRow() const
{
  Gtk::TreeModel::iterator iter = get_active();
  if (iter)
    return *iter;
  throw std::out_of_range("Invalid iterator");
}
}  // namespace hl_labelling
