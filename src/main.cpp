#include "gridmap.h"
#include "odometry.h"
#include "planning.h"
#include "ublox_reader.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {

  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <gps_data_file> <output_file>" << endl;
    return 1;
  }

  string gps_data = argv[1];
  string odom_commands = argv[2];

  // decode GPS data
  auto result = readUbloxFile(gps_data);
  if ((int)result.first.lat == 0 && (int)result.first.lon == 0 &&
      (int)result.second.lat == 0 && (int)result.second.lon == 0) {
    cout << "Error: Invalid GPS Coordinates" << endl;
    return 1;
  }

  cout << "Start -> Lat: " << result.first.lat << " Lon: " << result.first.lon << endl;
  cout << "Goal  -> Lat: " << result.second.lat << " Lon: " << result.second.lon << endl;

  // Initialize grid
  GPS origin = {result.first.lat, result.first.lon};
  Gridmapper grid(origin, 1.0, 10, 10);

  pair<int, int> start = grid.gpstogrid(result.first);
  pair<int, int> goal = grid.gpstogrid(result.second);

  cout << "Start (grid) -> (" << start.first << "," << start.second << ")" << endl;
  cout << "Goal  (grid) -> (" << goal.first << "," << goal.second << ")" << endl;

  // Planner
  Planner planner(grid.getGrid());
  auto path = planner.pathplanning(start, goal);

  cout << "Planned Path:" << endl;
  for (auto &p : path) {
    cout << "(" << p.first << "," << p.second << ") ";
  }
  cout << endl;

  // Odometry
  Odometry odo(0.05, 120); // wheel radius, rpm
  MotionCommand commands = odo.computeCommands(path);

  ofstream result_file(odom_commands);
  if (!result_file.is_open()) {
    cerr << "Error: cannot open file " << odom_commands << endl;
    return 1;
  }

  result_file << commands.time_sec << endl;
  result_file << commands.angle_deg << endl;
  result_file.close();

  return 0;
}


