#include <hl_labelling/gtkmm/labelling_chooser.h>

namespace hl_labelling
{
LabellingChooser::LabellingChooser()
{
  tree_content = Gtk::ListStore::create(tree_model);
  set_model(tree_content);
  pack_start(tree_model.col_name);
  addRow(ObjectType::Field);
  int max_balls = 6;
  int max_robots = 6;
  for (int i = 0; i < max_balls; i++)
    addRow(ObjectType::Ball, i);
  for (int team : { 0, 1 })
    for (int robot_id = 1; robot_id <= max_robots; robot_id++)
      addRow(ObjectType::Robot, team, robot_id);
}

LabellingChooser::ObjectType LabellingChooser::getObjectType() const
{
  return getActiveRow()[tree_model.obj_type];
}
int LabellingChooser::getTeamID() const
{
  return getActiveRow()[tree_model.team_id];
}
int LabellingChooser::getObjID() const
{
  return getActiveRow()[tree_model.obj_id];
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
  add(obj_id);
}
void LabellingChooser::addRow(ObjectType obj_type, int team_id, int obj_id)
{
  static int id = 0;
  Gtk::TreeModel::Row row = *tree_content->append();
  row[tree_model.obj_type] = obj_type;
  row[tree_model.team_id] = team_id;
  row[tree_model.obj_id] = obj_id;
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
