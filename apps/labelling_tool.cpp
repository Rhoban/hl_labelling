#include <hl_communication/utils.h>
#include <hl_labelling/labelling_window.h>
#include <tclap/CmdLine.h>
#include <QApplication>
#include <locale>

using namespace hl_monitoring;
using namespace hl_labelling;

int main(int argc, char** argv)
{
  TCLAP::CmdLine cmd("labelling_tool", ' ', "0.0");
  TCLAP::ValueArg<std::string> input_arg("i", "input", "Initial labelling", false, "labelling.pb", "input", cmd);
  TCLAP::ValueArg<std::string> output_arg("o", "output", "Output file for labelling", true, "labelling.pb", "output",
                                          cmd);
  cmd.parse(argc, argv);

  QApplication app(argc, argv);
  std::setlocale(LC_ALL, "C");  // Qt tend to change the locale which causes issues for parsing numbers

  std::unique_ptr<MonitoringManager> manager(new MonitoringManager());
  manager->loadConfig("replay.json");
  LabellingWindow window(std::move(manager));
  if (input_arg.isSet())
  {
    hl_communication::GameLabelCollection labels;
    hl_communication::readFromFile(input_arg.getValue(), &labels);
    window.importLabels(labels);
  }
  window.show();
  int return_code = app.exec();
  if (output_arg.isSet())
  {
    hl_communication::GameLabelCollection labels;
    window.exportLabels(&labels);
    hl_communication::writeToFile(output_arg.getValue(), labels);
  }
  return return_code;
}
