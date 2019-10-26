#pragma once

#include <rhoban_utils/history/history.h>

namespace hl_labelling
{
/**
 * Represents 3D Position of an object along with its validity
 *
 * - Position is interpolated linearly between specified values
 * - Activity uses the most recent value
 */
class ActivablePosHistory
{
public:
  ActivablePosHistory();

  bool isActive(double t) const;

  Eigen::Vector3d getPosition(double t) const;

  /**
   * Add a position to the history, automatically consider it as active as well
   */
  void pushPosition(double t, const Eigen::Vector3d& pos);

  /**
   * Set the robot as inactive from 't' until next valid position
   */
  void disable(double t);

private:
  /**
   * Position of the object
   */
  rhoban_utils::HistoryVector3d position;
  /**
   * Check whether the object is active or not
   */
  rhoban_utils::HistoryBool activity;
};
}  // namespace hl_labelling
