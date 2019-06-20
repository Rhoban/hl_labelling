#include <hl_communication/utils.h>
#include <hl_labelling/labelling_window.h>
#include <tclap/CmdLine.h>
#include <locale>

using namespace hl_monitoring;
using namespace hl_labelling;

int main(int argc, char** argv)
{
  TCLAP::CmdLine cmd("labelling_tool", ' ', "0.0");
  TCLAP::MultiArg<std::string> input_arg("i", "input", "Existing label files", false, "labelling.pb", cmd);
  TCLAP::ValueArg<std::string> output_arg("o", "output", "Output file for labelling", true, "labelling.pb", "output",
                                          cmd);
  TCLAP::ValueArg<std::string> video_arg("v", "video", "Name of the video to be labelled", true, "camera", "camera",
                                         cmd);
  TCLAP::ValueArg<std::string> metadata_arg("m", "metadata", "Metadata of the video to be labelled", true, "metadata",
                                            "metadata", cmd);
  TCLAP::SwitchArg clear_balls_arg("", "clear-balls", "Clear all balls from the input", cmd);
  TCLAP::SwitchArg moving_only_arg("", "moving-only", "Keep only frames with status 'moving'", cmd);
  TCLAP::SwitchArg merge_arg("", "merge", "Place all labels in output file, not only the one from current video", cmd);
  TCLAP::SwitchArg verbose_arg("", "verbose", "Use log output", cmd);
  cmd.parse(argc, argv);

  std::unique_ptr<ReplayImageProvider> image_provider(
      new ReplayImageProvider(video_arg.getValue(), metadata_arg.getValue()));
  hl_communication::VideoSourceID source_id = image_provider->getMetaInformation().source_id();
  LabellingWindow window(std::move(image_provider), "calibration_tool", moving_only_arg.getValue());
  for (const std::string& label_path : input_arg.getValue())
  {
    hl_communication::MovieLabelCollection labels;
    hl_communication::readFromFile(label_path, &labels);
    window.labelling_manager.importLabels(labels);
  }
  window.labelling_manager.sync();
  // if (clear_balls_arg.getValue())
  //{
  //  window.labelling_manager.clearBalls();
  //}
  if (verbose_arg.getValue())
  {
    window.labelling_manager.summarize(&std::cout);
  }
  window.run();
  if (merge_arg.getValue())
  {
    hl_communication::GameLabelCollection game_labels;
    window.labelling_manager.exportLabels(&game_labels);
    hl_communication::writeToFile(output_arg.getValue(), game_labels);
  }
  else
  {
    hl_communication::MovieLabelCollection video_labels;
    window.labelling_manager.exportLabels(source_id, &video_labels);
    hl_communication::writeToFile(output_arg.getValue(), video_labels);
  }
}
