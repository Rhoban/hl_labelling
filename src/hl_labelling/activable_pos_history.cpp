#include <hl_labelling/activable_pos_history.h>

namespace hl_labelling
{
ActivablePosHistory::ActivablePosHistory() : position(-1), activity(-1, rhoban_utils::InterpolatePolicy::Last)
{
}
bool ActivablePosHistory::isActive(double t) const
{
  return activity.interpolate(t);
}

Eigen::Vector3d ActivablePosHistory::getPosition(double t) const
{
  return position.interpolate(t);
}

void ActivablePosHistory::pushPosition(double t, const Eigen::Vector3d& pos)
{
  activity.pushValue(t, true, false);
  position.pushValue(t, pos, false);
}

void ActivablePosHistory::disable(double t)
{
  activity.pushValue(t, false, false);
}

}  // namespace hl_labelling
