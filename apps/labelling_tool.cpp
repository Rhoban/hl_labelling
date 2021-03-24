#include <hl_communication/utils.h>
#include <hl_labelling/labelling_window.h>
#include <rhoban_utils/util.h>
#include <tclap/CmdLine.h>
#include <locale>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <deque>
#include <map>

using namespace hl_monitoring;
using namespace hl_labelling;

void importLabels(const std::string& path, LabellingManager* label_manager, bool video_input,
                  unsigned int min_frames_between_labels = 0)
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

    if (min_frames_between_labels != 0)
    {
      for (int movie_index = 0; movie_index < labels.movies_size(); movie_index++)
      {
        auto* mut_movie = labels.mutable_movies(movie_index);
        auto collections_copy = mut_movie->label_collections();
        mut_movie->clear_label_collections();

        for (const auto& labels_collection : collections_copy)
        {
          auto new_collection = mut_movie->add_label_collections();
          int last_frame_index = -1;
          for (const auto& label : labels_collection.labels())
          {
            if ((label.frame_index() - last_frame_index > min_frames_between_labels) || last_frame_index == -1)
            {
              auto* l = new_collection->add_labels();
              l->MergeFrom(label);
              last_frame_index = label.frame_index();
            }
          }
        }
      }
    }
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

  TCLAP::SwitchArg export_csv_arg(
      "", "csv", "Produce a csv in output file containing the data an estimate trajectory based on labels.", cmd);
  TCLAP::ValueArg<unsigned int> min_frames_between_two_labels_arg("", "min-frames-between-labels",
                                                                  "discard some labels to have a given minimum "
                                                                  "number "
                                                                  "of "
                                                                  "frames "
                                                                  "between two of them",
                                                                  false, 0, "uint", cmd);
  TCLAP::ValueArg<unsigned int> first_frame_arg("", "csv-first-frame", "Starting frame index to export into csv file",
                                                false, 0, "uint", cmd);
  TCLAP::ValueArg<unsigned int> nb_frames_arg("", "csv-nb-frames", "Total frames to export into csv file", false, 0,
                                              "uint", cmd);

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
    importLabels(edit_arg.getValue(), &window.labelling_manager, video_input_arg.getValue(),
                 min_frames_between_two_labels_arg.getValue());
  }

  for (const std::string& label_path : input_arg.getValue())
  {
    importLabels(label_path, &window.labelling_manager, video_input_arg.getValue(),
                 min_frames_between_two_labels_arg.getValue());
  }

  if (!clear_robot_arg.isSet())
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
  if (clear_ball_arg.isSet())
  {
    std::cout << "clear ball id : " << clear_ball_arg.getValue() << std::endl;
    window.labelling_manager.clearBall(clear_ball_arg.getValue());
  }

  if (clear_all_balls_arg.getValue())
  {
    window.labelling_manager.clearAllBalls();
  }

  window.labelling_manager.sync();

  if (verbose_arg.getValue())
  {
    window.labelling_manager.summarize(&std::cout);
    window.labelling_manager.analyze(&std::cout);
  }

  std::string output_path = "";
  if (output_arg.isSet())
  {
    output_path = output_arg.getValue();
  }
  else if (edit_arg.isSet())
  {
    output_path = edit_arg.getValue();
  }

  if (export_csv_arg.isSet())
  {
    std::string current_mode = "LINEAR_INTERPOLATION";
    std::vector<cv::Point3f> field_points;

    std::vector<cv::Point2f> projected_field_points;
    auto manual_poses = window.labelling_manager.managers[source_id].getManualPoses();

    std::ofstream csv(output_path, std::ios::out);
    auto labels = window.labelling_manager.managers[source_id].getLabels();

    // construct header
    csv << "time_us,interpolation_mode,frame_id,cam_status,is_annotation_frame"
        << ",camera_estimate_translation_x,camera_estimate_translation_y,camera_estimate_translation_z"
        //        <<
        //        ",camera_estimate_rotation_q0,camera_estimate_rotation_q1,camera_estimate_rotation_q3,camera_estimate_"
        << ",camera_estimate_roll,camera_estimate_pitch,camera_estimate_yaw";
    for (auto&& [type, p] : window.field.getPointsOfInterest())
    {
      csv << ',' << type << "_x";
      csv << ',' << type << "_y";
      field_points.push_back(p);
    }

    csv << '\n';  // END header

    cv::Mat camera_matrix, distortion_coeffs, rvec, tvec;
    cv::Size size;

    intrinsicToCV(window.getMetaInformation().camera_parameters(), &camera_matrix, &distortion_coeffs, &size);

    // extract data based on labels
    uint nb_frames = window.provider->getNbFrames();
    if (nb_frames_arg.getValue() != 0)
      nb_frames = std::min(nb_frames, nb_frames_arg.getValue());

    std::cout << "Export CSV, " << nb_frames << " frames...\n";

    for (uint frame_id = first_frame_arg.getValue(); frame_id < nb_frames; ++frame_id)
    {
      // load current frame data
      auto time = window.getMetaInformation().frames(frame_id).utc_ts();
      auto camera_status = window.getMetaInformation().frames(frame_id).status();

      hl_communication::CameraMetaInformation camera_information = window.calibrated_img.getCameraInformation();
      auto cam_pos = window.labelling_manager.getCameraPose(source_id, time);

      Eigen::Vector3d src_in_dst = cam_pos * Eigen::Vector3d::Zero();
      auto pose = camera_information.mutable_pose();
      pose->clear_rotation();
      pose->clear_translation();
      Eigen::Vector3d t = Eigen::Affine3d(cam_pos.rotation().inverse()) * src_in_dst;

      Eigen::Quaterniond q(cam_pos.linear());
      //      pose->add_rotation(q.w());
      //      pose->add_rotation(q.x());
      //      pose->add_rotation(q.y());
      //      pose->add_rotation(q.z());
      auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      pose->add_rotation(euler[0]);
      pose->add_rotation(euler[1]);
      pose->add_rotation(euler[2]);
      pose->add_translation(-t(0));
      pose->add_translation(-t(1));
      pose->add_translation(-t(2));
      bool is_annotation_frame = labels.begin()->second.frame_index() == frame_id;
      if (is_annotation_frame)
      {
        labels.erase(labels.begin());
      }

      // write current frame data
      csv << time << ',' << current_mode << ',' << frame_id << ',' << camera_status << ',' << is_annotation_frame;
      // cam translation
      csv << ',' << camera_information.pose().translation(0) << ',' << camera_information.pose().translation(1) << ','
          << camera_information.pose().translation(2);
      // cam rotation
      csv << ',' << camera_information.pose().rotation(0) << ',' << camera_information.pose().rotation(1) << ','
          << camera_information.pose().rotation(2);  // << ',' << camera_information.pose().rotation(3);

      // reprojected field points
      pose3DToCV(camera_information.pose(), &rvec, &tvec);
      for (auto& p : field_points)
      {
        if (not hl_communication::isPointValidForCorrection(p, rvec, tvec, camera_matrix, distortion_coeffs))
        {
          std::cerr << "error invalid point\n";
          continue;
        }
      }
      // keep points also if outside

      cv::projectPoints(field_points, rvec, tvec, camera_matrix, distortion_coeffs, projected_field_points);

      for (auto& p : projected_field_points)
        csv << ',' << p.x << ',' << p.y;

      projected_field_points.clear();
      csv << '\n';  // END frame

      std::cout << "frame " << (frame_id + 1) - first_frame_arg.getValue() << "/"
                << nb_frames - first_frame_arg.getValue() << " done.\n";
    }

    csv.close();
    exit(0);
  }

  window.run();
  bool write_data = false;

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
