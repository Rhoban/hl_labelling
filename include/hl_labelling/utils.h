#pragma once

#include <hl_communication/camera.pb.h>
#include <robot_model/camera_model.h>

namespace hl_labelling
{
rhoban::CameraModel intrinsicParametersToCameraModel(const hl_communication::IntrinsicParameters& intrinsic);
}
