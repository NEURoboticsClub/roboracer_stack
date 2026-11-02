#include "algorithm.hpp"

// ===========================================================================
// YOUR ALGORITHM GOES HERE. This is the only file you need to edit.
//
// `ranges` is one lidar scan: ranges[i] is the distance in meters seen by beam
// i, at angle ind2angle(i). The beam count differs between the sim and the real
// car, so use ind2angle() rather than hard-coding indices.
//
// The scan is passed through raw: a beam with no return can read 0, inf or nan.
//
// To keep state between scans, use static variables in this file. The scan
// callback waits for this function, so a slow algorithm lowers the command rate.
//
// See algorithm.hpp for what the returned command means.
// ===========================================================================
DriveCommand computeDrive(const std::vector<float> &ranges)
{
  (void)ranges; // remove once you use it

  DriveCommand cmd;
  cmd.steering = 0.0f; // straight
  cmd.throttle = 0.0f; // stopped - raise this to move
  return cmd;
}
