#pragma once

#include <gtkmm-2.4/gtkmm.h>

namespace hl_labelling
{
/**
 * A labelling chooser contains one or multiple combo-box allowing to choose which item is being labelled
 */
class LabellingChooser : public Gtk::ComboBox
{
public:
  enum ObjectType
  {
    Field,
    Ball,
    Robot,
    None
  };
  LabellingChooser();

  /**
   * Return ObjectType::None if no labelling mode is selected
   */
  ObjectType getObjectType() const;
  /**
   * Return -1 if no labelling mode is selected or if team_id is not relevant for current object
   */
  int getTeamID() const;
  /**
   * Return -1 if no labelling mode is selected or if obj_id is not relevant for current object
   */
  int getObjID() const;

  static std::string objectTypeToStr(ObjectType type);

private:
  class TreeModel : public Gtk::TreeModel::ColumnRecord
  {
  public:
    TreeModel();
    Gtk::TreeModelColumn<int> col_id;
    Gtk::TreeModelColumn<std::string> col_name;
    Gtk::TreeModelColumn<ObjectType> obj_type;
    Gtk::TreeModelColumn<int> team_id;
    Gtk::TreeModelColumn<int> obj_id;
  };
  /**
   * Add a labelling option:
   * - If team_id is not specified or negative: the labelling option is considered as independent from team
   * - If obj_id is not specified or negative: there are no element for obj_id
   */
  void addRow(ObjectType obj_type, int team_id = -1, int obj_id = -1);

  Gtk::TreeModel::Row getActiveRow() const;

  TreeModel tree_model;
  Glib::RefPtr<Gtk::ListStore> tree_content;
};

}  // namespace hl_labelling
