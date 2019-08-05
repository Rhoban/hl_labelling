#include <hl_labelling/gtkmm/labelling_widget.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_monitoring;
using namespace hl_labelling;

class TestWindow : public Gtk::Window
{
public:
  TestWindow() : labelling_widget(Field())
  {
    set_border_width(10);
    add(labelling_widget);
    labelling_widget.show();
    labelling_widget.set_size_request(800, 600);
  }

private:
  LabellingWidget labelling_widget;
};

int main(int argc, char* argv[])
{
  Gtk::Main kit(argc, argv);
  TestWindow window;
  window.set_default_size(200, 200);
  Gtk::Main::run(window);
}
