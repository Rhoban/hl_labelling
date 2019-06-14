#include <hl_communication/utils.h>
#include <hl_labelling/labelling_window.h>
#include <tclap/CmdLine.h>
#include <locale>

using namespace hl_monitoring;
using namespace hl_labelling;

int main(int argc, char** argv)
{
  TCLAP::CmdLine cmd("labelling_tool", ' ', "0.0");
  TCLAP::ValueArg<std::string> input_arg("i", "input", "Initial labelling", false, "labelling.pb", "input", cmd);
  TCLAP::ValueArg<std::string> output_arg("o", "output", "Output file for labelling", true, "labelling.pb", "output",
                                          cmd);
  TCLAP::ValueArg<std::string> video_arg("v", "video", "Name of the video to be labelled", true, "camera", "camera",
                                         cmd);
  TCLAP::ValueArg<std::string> metadata_arg("m", "metadata", "Metadata of the video to be labelled", true, "metadata",
                                            "metadata", cmd);
  TCLAP::SwitchArg clear_balls_arg("", "clear-balls", "Clear all balls from the input", cmd);
  TCLAP::SwitchArg moving_only_arg("", "moving-only", "Keep only frames with status 'moving'", cmd);
  cmd.parse(argc, argv);

  std::unique_ptr<ReplayImageProvider> image_provider(
      new ReplayImageProvider(video_arg.getValue(), metadata_arg.getValue()));
  LabellingWindow window(std::move(image_provider), "calibration_tool", moving_only_arg.getValue());
  if (input_arg.isSet())
  {
    hl_communication::MovieLabelCollection labels;
    hl_communication::readFromFile(input_arg.getValue(), &labels);
    window.labelling_manager.importLabels(labels, window.field.ball_radius);
    if (clear_balls_arg.getValue())
    {
      window.labelling_manager.clearBalls();
    }
  }
  window.run();
  hl_communication::MovieLabelCollection labels;
  window.labelling_manager.exportLabels(&labels);
  hl_communication::writeToFile(output_arg.getValue(), labels);
}
