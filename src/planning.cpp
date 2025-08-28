#include <iostream>
#include <vector>
#include <string>
#include <utility>  
#include <fstream>
#include <sstream>

using namespace std;
vector<vector<bool>> readUbloxFile(const string& filename) {
    vector<vector<bool>> grid(10, vector<bool>(10, false));

    grid[2][3] = true;
    grid[5][5] = true;
    grid[7][1] = true;

    cout << "Reading ublox file: " << filename << endl;
    cout << "Created a dummy 10x10 grid with obstacles.\n";
    return grid;
}

class Planner {
private:
    vector<vector<bool>> map;

public:
    Planner(const vector<vector<bool>>& inputMap) : map(inputMap) {
        cout << "Planner initialized with map of size " 
             << map.size() << "x" << map[0].size() << endl;
    }

    vector<pair<int,int>> pathplanning(pair<int,int> start, pair<int,int> goal) {
        cout << "Planning path from (" << start.first << "," << start.second 
             << ") to (" << goal.first << "," << goal.second << ")..." << endl;

        // Dummy straight-line path for now
        vector<pair<int,int>> path;
        int x = start.first, y = start.second;
        while (x != goal.first || y != goal.second) {
            if (x < goal.first) x++;
            else if (x > goal.first) x--;

            if (y < goal.second) y++;
            else if (y > goal.second) y--;

            path.push_back({x,y});
        }

        cout << "Path found with " << path.size() << " steps.\n";
        return path;
    }
};

class Odometry {
private:
    double wheelBase;
    double wheelRadius;

public:
    Odometry(double base, double radius) 
        : wheelBase(base), wheelRadius(radius) {
        cout << "Odometry initialized (wheelBase=" << wheelBase 
             << ", wheelRadius=" << wheelRadius << ")\n";
    }

    void computeCommands(vector<pair<int,int>>& path) {
        cout << "Computing commands for path of length " << path.size() << "...\n";
        for (auto& p : path) {
            cout << " -> Move to (" << p.first << "," << p.second << ")\n";
        }
    }
};



