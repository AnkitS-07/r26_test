#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

class Odometry {
public:
    vector<string> computeCommands(const vector<pair<int,int>>& path) {
        vector<string> commands;

        if (path.empty()) return commands;

        for (size_t i = 1; i < path.size(); i++) {
            int dx = path[i].first - path[i-1].first;
            int dy = path[i].second - path[i-1].second;

            if (dx == 1 && dy == 0) commands.push_back("Move Down");
            else if (dx == -1 && dy == 0) commands.push_back("Move Up");
            else if (dx == 0 && dy == 1) commands.push_back("Move Right");
            else if (dx == 0 && dy == -1) commands.push_back("Move Left");
        }

        return commands;
    }
};
