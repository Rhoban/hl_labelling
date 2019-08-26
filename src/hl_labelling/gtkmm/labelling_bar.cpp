#include <hl_labelling/gtkmm/labelling_bar.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/field.h>
#include <hl_monitoring/gtkmm/dialogs.h>
#include <string>

namespace hl_labelling
{
LabellingBar::LabellingBar(LabellingManager* manager)
  : labelling_manager(manager), load_labels_button("Load labels"), save_labels_button("Save labels")
{
  add(labelling_chooser);
  labelling_chooser.show();
  load_labels_button.signal_clicked().connect(sigc::mem_fun(*this, &LabellingBar::on_load_labels));
  add(load_labels_button);
  load_labels_button.show();
  save_labels_button.signal_clicked().connect(sigc::mem_fun(*this, &LabellingBar::on_save_labels));
  add(save_labels_button);
  save_labels_button.show();
}

LabellingBar::~LabellingBar()
{
}

void LabellingBar::on_load_labels()
{
  std::string label_file;
  Gtk::Window* window = (Gtk::Window*)get_toplevel();
  if (hl_monitoring::requestProtobufFile(window, &label_file))
  {
    labelling_manager->importLabels(label_file);
  }
  labelling_manager->summarize(&std::cout);
}

void LabellingBar::on_save_labels()
{
  std::string path;
  Gtk::Window* window = (Gtk::Window*)get_toplevel();
  if (hl_monitoring::requestSavePath(window, &path))
  {
    hl_communication::GameLabelCollection labels;
    labelling_manager->exportLabels(&labels);
    hl_communication::writeToFile(path, labels);
  }
}
}  // namespace hl_labelling
