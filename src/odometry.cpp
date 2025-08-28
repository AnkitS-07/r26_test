#include "odometry.h"
#include <cmath>
#include <ctime>
#include <iterator>
#include <numeric>

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  // Linear velocity (m/s) = (wheel circumference * revolutions per second)
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  // atan2 returns radians, convert to degrees
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
  MotionCommand res = {0.0, 0.0}; // store total time and total angle

  if (path.size() < 2) return res; // no motion needed

  double total_distance = 0.0;
  double total_angle = 0.0;

  // compute distances + turning angles
  for (size_t i = 1; i < path.size(); i++) {
    // distance between consecutive points
    total_distance += distance(path[i - 1].first, path[i - 1].second,
                               path[i].first, path[i].second);

    // compute angle change (except for the first movement)
    if (i >= 2) {
      double prev_angle = angle(path[i - 2].first, path[i - 2].second,
                                path[i - 1].first, path[i - 1].second);
      double curr_angle = angle(path[i - 1].first, path[i - 1].second,
                                path[i].first, path[i].second);

      double dtheta = curr_angle - prev_angle;
      // normalize to [-180,180] range
      while (dtheta > 180) dtheta -= 360;
      while (dtheta < -180) dtheta += 360;

      total_angle += fabs(dtheta);
    }
  }

  // total time = distance / linear velocity
  double total_time = (linear_vel > 0) ? (total_distance / linear_vel) : 0.0;

  res.time = total_time;
  res.angle = total_angle;

  return res;
}
