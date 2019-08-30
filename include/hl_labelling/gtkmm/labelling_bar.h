#pragma once

#include <hl_labelling/gtkmm/labelling_chooser.h>
#include <hl_labelling/labelling_manager.h>

#include <gtkmm-2.4/gtkmm.h>

namespace hl_labelling
{
class LabellingBar : public Gtk::HBox
{
public:
  LabellingBar(LabellingManager* manager);
  virtual ~LabellingBar();

  const LabellingChooser& getLabellingChooser() const;

  /**
   * This signal is emitted every time the labelling_manager collection is changed by the labelling bar
   */
  sigc::signal<void> signal_collection_changed();

protected:
  /**
   * Open a dialog box to choose an input label file
   */
  void on_load_labels();
  /**
   * Open a dialog box to choose an output label file
   */
  void on_save_labels();

  /**
   * Access to the labelling manager concerned by this labelling bar.
   * LabellingBar do not have ownership on this
   */
  LabellingManager* labelling_manager;
  LabellingChooser labelling_chooser;
  Gtk::Button load_labels_button;
  Gtk::Button save_labels_button;

  sigc::signal<void> collection_change_signal;
};
}  // namespace hl_labelling
