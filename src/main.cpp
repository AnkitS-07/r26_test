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
        cerr << "Usage: " << argv[0] << " <gps_data_file> <odom_output_file>" << endl;
        return 1;
    }

    string gps_data_file = argv[1];
    string odom_output_file = argv[2];

    // Decode GPS data from file
    auto result = readUbloxFile(gps_data_file);
    if ((int)result.first.lat == 0 && (int)result.first.lon == 0 &&
        (int)result.second.lat == 0 && (int)result.second.lon == 0) {
        cerr << "Error: Invalid GPS Coordinates" << endl;
        return 1;
    }

    cout << "Start -> Lat: " << result.first.lat << " Lon: " << result.first.lon << endl;
    cout << "Goal  -> Lat: " << result.second.lat << " Lon: " << result.second.lon << endl;

    // Initialize Gridmapper with start as origin
    GPS origin = {result.first.lat, result.first.lon};
    double cellsize = 1.0; // meters per grid cell
    int rows = 10, cols = 10;
    Gridmapper grid(origin, cellsize, rows, cols);

    // Convert start and goal GPS to grid coordinates
    pair<int, int> start = grid.gpstogrid(result.first);
    pair<int, int> goal = grid.gpstogrid(result.second);

    cout << "Start (grid) -> (" << start.first << "," << start.second << ")" << endl;
    cout << "Goal  (grid) -> (" << goal.first << "," << goal.second << ")" << endl;

    // Path planning
    Planner planner(grid.getGrid());
    auto path = planner.pathplanning(start, goal);

    // Print planned path
    cout << "Planned Path:" << endl;
    for (auto &p : path) {
        cout << "(" << p.first << "," << p.second << ") ";
    }
    cout << endl;

    // Odometry commands
    cout << "\nOdometry Commands" << endl;
    double wheel_radius = 0.05; // meters
    double rpm = 120;           // wheel speed
    Odometry odo(wheel_radius, rpm);
    auto commands = odo.computeCommands(path);

    cout << "Total Time: " << commands.time << " s" << endl;
    cout << "Total Angle: " << commands.angle << " degrees" << endl;

    // Write result to file
    ofstream result_file(odom_output_file);
    if (!result_file.is_open()) {
        cerr << "Error: cannot open file " << odom_output_file << endl;
        return 1;
    }

    result_file << commands.time << endl << commands.angle << endl;
    result_file.close();

    return 0;
}

