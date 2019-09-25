#include <hl_labelling/label_drawer.h>

#include <opencv2/imgproc.hpp>

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
}
}  // namespace hl_labelling
