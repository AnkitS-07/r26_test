#include <iostream>
#include "ublox_reader.h"
#include "planning.h"
#include "odometry.h"

using namespace std;

int main() {
    cout << "=== Rover Navigation Test ===" << endl;

    // --- Task 1: UBX Data Decode ---
    UbloxReader reader;
    if (!reader.readUBX("sample.ubx")) {   // Assume input file is sample.ubx
        cerr << "Error: Could not read UBX file" << endl;
        return 1;
    }

    double lat = reader.getLatitude();
    double lon = reader.getLongitude();
    cout << "Decoded GPS Coordinates: Lat = " << lat << ", Lon = " << lon << endl;

    // --- Task 2: Path Planning ---
    Planner planner;
    pair<int, int> start = {0, 0};
    pair<int, int> goal = {5, 5};

    vector<pair<int, int>> path = planner.pathplanning(start, goal);
    cout << "Planned Path:" << endl;
    for (auto &p : path) {
        cout << "(" << p.first << "," << p.second << ") ";
    }
    cout << endl;

    // --- Task 3: Odometry Commands ---
    Odometry odo;
    vector<string> commands = odo.computeCommands(path);

    cout << "Generated Commands:" << endl;
    for (auto &cmd : commands) {
        cout << cmd << endl;
    }

    cout << "=== Test Completed ===" << endl;
    return 0;
}

