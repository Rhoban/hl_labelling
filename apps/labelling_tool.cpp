#include <hl_communication/utils.h>
#include <hl_labelling/labelling_window.h>
#include <rhoban_utils/util.h>
#include <tclap/CmdLine.h>
#include <locale>

#include <iostream>

using namespace hl_monitoring;
using namespace hl_labelling;

void importLabels(const std::string& path, LabellingManager* label_manager, bool video_input)
{
  if (video_input)
  {
    hl_communication::MovieLabelCollection labels;
    hl_communication::readFromFile(path, &labels);
    label_manager->importLabels(labels);
  }
  else
  {
    hl_communication::GameLabelCollection labels;
    hl_communication::readFromFile(path, &labels);
    label_manager->importLabels(labels);
  }
}

int main(int argc, char** argv)
{
  TCLAP::CmdLine cmd("labelling_tool", ' ', "0.0");
  TCLAP::MultiArg<std::string> input_arg("i", "input", "Existing label files", false, "labelling.pb", cmd);
  TCLAP::ValueArg<std::string> output_arg("o", "output", "Output file for labelling", false, "labelling.pb", "output",
                                          cmd);
  TCLAP::ValueArg<std::string> edit_arg("e", "edit", "Input/Output file for labelling", false, "labelling.pb", "output",
                                        cmd);
  TCLAP::ValueArg<std::string> video_arg("v", "video", "Name of the video to be labelled", false, "camera", "camera",
                                         cmd);
  TCLAP::ValueArg<std::string> metadata_arg("m", "metadata", "Metadata of the video to be labelled", false, "metadata",
                                            "metadata", cmd);
  TCLAP::ValueArg<std::string> robot_prefix_arg("", "robot-prefix",
                                                "Uses robot_prefix to find the appropriate files for metadata and "
                                                "video",
                                                false, "log_path", "log_path", cmd);

  TCLAP::SwitchArg clear_all_balls_arg("", "clear-all-balls", "Clear all balls from the input", cmd);
  TCLAP::ValueArg<int> clear_ball_arg("", "clear-ball", "Clear a ball from the input, arg is the id", false, 0,
                                      "clear_option", cmd);
  TCLAP::SwitchArg clear_all_robots_arg("", "clear-all-robots", "Clear all robots from the input", cmd);
  TCLAP::ValueArg<std::string> clear_robot_arg(
      "", "clear-robot", "Clear a robot from the input, write : teamId:robotID", false, "0:0", "clear_option", cmd);

  TCLAP::SwitchArg all_frames_arg("", "all-frames", "Keep all frames, not only the 'moving' ones", cmd);
  TCLAP::SwitchArg verbose_arg("", "verbose", "Use log output", cmd);
  TCLAP::SwitchArg video_output_arg("", "video_output",
                                    "Place all labels in output file, not only the one from current video", cmd);
  TCLAP::SwitchArg video_input_arg("", "video-input", "Input labels are 'merged labels'", cmd);
  cmd.parse(argc, argv);

  // First of all, check if -o has a risk of overriding a file
  if (output_arg.isSet() && rhoban_utils::file_exists(output_arg.getValue()))
  {
    std::cerr << "File '" << output_arg.getValue() << " already exists, use edit option" << std::endl;
    exit(-1);
  }

  // Check that edit or output is specified (tagging makes little sense if no output is allowed)
  // Could be done using xor arguments
  if (!output_arg.isSet() && !edit_arg.isSet())
  {
    std::cerr << "Flags output or edit should be used" << std::endl;
    exit(-1);
  }

  // Could be done using xor arguments
  if (!(video_arg.isSet() && metadata_arg.isSet()) && !robot_prefix_arg.isSet())
  {
    std::cerr << "Either specify 'video & metadata' or 'robot_prefix'" << std::endl;
    exit(-1);
  }
  std::string video_path, metadata_path;
  if (robot_prefix_arg.isSet())
  {
    std::string robot_prefix = robot_prefix_arg.getValue();
    if (robot_prefix[video_path.size() - 1] != '/')
      robot_prefix += "/";
    video_path = robot_prefix + "video.avi";
    metadata_path = robot_prefix + "camera_from_world.pb";
  }
  else
  {
    video_path = video_arg.getValue();
    metadata_path = metadata_arg.getValue();
  }

  std::unique_ptr<ReplayImageProvider> image_provider(new ReplayImageProvider(video_path, metadata_path));
  hl_communication::VideoSourceID source_id = image_provider->getMetaInformation().source_id();
  LabellingWindow window(std::move(image_provider), "calibration_tool", !all_frames_arg.getValue());

  if (edit_arg.isSet())
  {
    importLabels(edit_arg.getValue(), &window.labelling_manager, video_input_arg.getValue());
  }

  for (const std::string& label_path : input_arg.getValue())
  {
    importLabels(label_path, &window.labelling_manager, video_input_arg.getValue());
  }

  if (!clear_robot_arg.getValue().empty())
  {
    std::string s = clear_robot_arg.getValue();
    std::string delimiter = ":";

    if (s.find(delimiter) == std::string::npos)
      throw std::logic_error(HL_DEBUG + "please write with the format : 'teamID:robotId'");

    std::string token = s.substr(0, s.find(delimiter));
    s.erase(0, s.find(delimiter) + delimiter.length());
    std::cout << "clear robot id : " << s << " team : " << token << std::endl;
    hl_communication::RobotIdentifier rb;
    rb.set_team_id(stoi(token));
    rb.set_robot_id(stoi(s));
    window.labelling_manager.clearRobot(rb);
  }
  if (clear_all_robots_arg.getValue())
  {
    window.labelling_manager.clearAllRobots();
  }
  if (clear_ball_arg.getValue())
  {
    std::cout << "clear ball id : " << clear_ball_arg.getValue() << std::endl;
    window.labelling_manager.clearBall(clear_ball_arg.getValue());
  }

  if (clear_all_balls_arg.getValue())
  {
    window.labelling_manager.clearAllBalls();
  }
  if (verbose_arg.getValue())
  {
    window.labelling_manager.summarize(&std::cout);
  }

  window.labelling_manager.sync();

  window.run();
  bool write_data = false;
  std::string output_path = "";
  if (output_arg.isSet())
  {
    output_path = output_arg.getValue();
  }
  else if (edit_arg.isSet())
  {
    output_path = edit_arg.getValue();
  }

  if (video_output_arg.getValue())
  {
    hl_communication::MovieLabelCollection video_labels;
    window.labelling_manager.exportLabels(source_id, &video_labels);
    hl_communication::writeToFile(output_path, video_labels);
  }
  else
  {
    hl_communication::GameLabelCollection game_labels;
    window.labelling_manager.exportLabels(&game_labels);
    hl_communication::writeToFile(output_path, game_labels);
  }
}
