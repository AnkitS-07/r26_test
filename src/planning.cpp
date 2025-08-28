#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <cmath>
using namespace std;

// ---------------- GPS STRUCT ----------------
struct GPS {
    double latitude;
    double longitude;
    GPS(double lat = 0, double lon = 0) : latitude(lat), longitude(lon) {}
};

// ---------------- GRIDMAPPER CLASS ----------------
class Gridmapper {
public:
    int width, height;

    Gridmapper(int w = 10, int h = 10) : width(w), height(h) {}

    pair<int, int> gpstogrid(const GPS& gps) {
        // Dummy conversion
        int x = static_cast<int>(gps.latitude) % width;
        int y = static_cast<int>(gps.longitude) % height;
        return {x, y};
    }

    GPS gridtogps(const pair<int, int>& cell) {
        return GPS(cell.first, cell.second);
    }

    bool isValidCell(const pair<int, int>& cell) {
        return (cell.first >= 0 && cell.first < width &&
                cell.second >= 0 && cell.second < height);
    }
};

// ---------------- PLANNING CLASS ----------------
class Planning {
private:
    Gridmapper& gm;

public:
    Planning(Gridmapper& gridmapper) : gm(gridmapper) {}

    bool findPath(const GPS& start, const GPS& goal, vector<pair<int, int>>& path) {
        auto startCell = gm.gpstogrid(start);
        auto goalCell = gm.gpstogrid(goal);

        queue<pair<int, int>> q;
        map<pair<int, int>, pair<int, int>> parent;
        vector<vector<bool>> visited(gm.width, vector<bool>(gm.height, false));

        q.push(startCell);
        visited[startCell.first][startCell.second] = true;

        int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
        bool found = false;

        while (!q.empty()) {
            auto cur = q.front();
            q.pop();

            if (cur == goalCell) {
                found = true;
                break;
            }

            for (auto& d : dirs) {
                pair<int, int> next = {cur.first + d[0], cur.second + d[1]};
                if (gm.isValidCell(next) && !visited[next.first][next.second]) {
                    visited[next.first][next.second] = true;
                    parent[next] = cur;
                    q.push(next);
                }
            }
        }

        if (!found) return false;

        // Reconstruct path
        path.clear();
        for (auto cur = goalCell; cur != startCell; cur = parent[cur]) {
            path.push_back(cur);
        }
        path.push_back(startCell);
        reverse(path.begin(), path.end());
        return true;
    }

    void printPath(const vector<pair<int, int>>& path) {
        cout << "Path: ";
        for (auto& p : path) {
            cout << "(" << p.first << "," << p.second << ") ";
        }
        cout << endl;
    }
};



