#include <hl_labelling/label_drawer.h>

#include <hl_monitoring/drawers/player_drawer.h>
#include <hl_monitoring/drawers/text_drawer.h>

#include <opencv2/imgproc.hpp>
#include <string>
#include <iostream>

using namespace hl_communication;
using namespace hl_monitoring;

namespace hl_labelling
{
void LabelDrawer::draw(FieldToImgConverter converter, const hl_communication::LabelMsg& data, cv::Mat* out)
{
  int ball_radius_px = 5;
  for (const BallMsg& ball : data.balls())
  {
    cv::Scalar color(0, 0, 0);
    cv::Point2f ball_center_in_img;
    bool valid = true;
    if (ball.has_center())
    {
      std::cout << "Drawing a ball with center" << std::endl;
      ball_center_in_img = protobufToCV(ball.center());
      color = cv::Scalar(0, 0, 255);
    }
    else if (ball.has_center_in_field())
    {
      valid = converter(protobufToCV(ball.center_in_field()), &ball_center_in_img);
      color = cv::Scalar(0, 165, 255);
    }
    else
    {
      valid = false;
    }
    if (valid)
    {
      // TODO: temporary, to be replaced by a sphere of appropriate radius
      cv::circle(*out, ball_center_in_img, ball_radius_px, color, CV_FILLED, cv::LineTypes::LINE_AA);
      if (ball.has_ball_id())
      {
        cv::putText(*out, std::to_string(ball.ball_id()), ball_center_in_img, cv::FONT_HERSHEY_PLAIN, 1.0,
                    cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      }
    }
  }
  for (const RobotMessage& robot : data.robots())
  {
    cv::Scalar color(255, 0, 255);
    if (robot.has_robot_color())
    {
      switch (robot.robot_color())
      {
        case TeamColor::BLUE:
          color = cv::Scalar(255, 0, 0);
          break;
        case TeamColor::RED:
          color = cv::Scalar(0, 0, 255);
          break;
        case TeamColor::CONFLICT:
          color = cv::Scalar(0, 255, 255);
          break;
        default:
          color = cv::Scalar(255, 0, 255);
      }
    }

    cv::Point2f robot_center_in_img;
    bool valid = true;
    if (robot.has_ground_position())
    {
      robot_center_in_img = protobufToCV(robot.ground_position());
      int robot_radius_px = 5;
      cv::circle(*out, robot_center_in_img, robot_radius_px, color, CV_FILLED, cv::LineTypes::LINE_AA);
    }
    else if (robot.has_robot_in_field())
    {
      valid = converter(protobufToCV(robot.robot_in_field()), &robot_center_in_img);
      hl_communication::RobotMsg fake_msg;
      fake_msg.mutable_robot_id()->CopyFrom(robot.robot_id());
      hl_communication::WeightedPose* w_pose = fake_msg.mutable_perception()->add_self_in_field();
      w_pose->set_probability(1.0);
      hl_communication::PositionDistribution* pos = w_pose->mutable_pose()->mutable_position();
      pos->set_x(robot.robot_in_field().x());
      pos->set_y(robot.robot_in_field().y());
      hl_monitoring::PlayerDrawer player_drawer;
      player_drawer.setColor(color);
      player_drawer.draw(converter, fake_msg, out);
    }
  }
  for (const Match2D3DMsg& match : data.field_matches())
  {
    cv::Scalar color(0, 0, 0);
    cv::Point2f img_pos = protobufToCV(match.img_pos());
    int marker_size = 15;
    int marker_thickness = 2;
    cv::drawMarker(*out, img_pos, color, cv::MarkerTypes::MARKER_CROSS, marker_size, marker_thickness);
  }
}
}  // namespace hl_labelling
