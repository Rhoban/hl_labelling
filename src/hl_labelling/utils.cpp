#include <hl_labelling/utils.h>
#include <hl_communication/utils.h>

using namespace hl_communication;

namespace hl_labelling
{
rhoban::CameraModel intrinsicParametersToCameraModel(const hl_communication::IntrinsicParameters& intrinsic)
{
  cv::Mat camera_matrix, distortion_coefficients;
  cv::Size img_size;
  intrinsicToCV(intrinsic, &camera_matrix, &distortion_coefficients, &img_size);
  return rhoban::CameraModel(camera_matrix, distortion_coefficients, img_size);
}

}  // namespace hl_labelling
