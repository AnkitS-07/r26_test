#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>

using namespace std;

struct Node {
    int x, y;
    double f, g;
    Node(int x, int y, double g, double f) : x(x), y(y), g(g), f(f) {}
};

// For priority queue (min-heap)
struct Compare {
    bool operator()(const Node &a, const Node &b) {
        return a.f > b.f;
    }
};

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
    rows = grid.size();
    cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
    return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
    vector<pair<int, int>> path;

    if (!isvalid(start.first, start.second) || !isvalid(goal.first, goal.second)) {
        return path; // empty, invalid inputs
    }
    if (start == goal) {
        path.push_back(start);
        return path;
    }

    // 8 possible moves
    vector<pair<int,int>> directions = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    priority_queue<Node, vector<Node>, Compare> open;
    vector<vector<double>> g_score(rows, vector<double>(cols, numeric_limits<double>::infinity()));
    vector<vector<pair<int,int>>> parent(rows, vector<pair<int,int>>(cols, {-1,-1}));

    int sx = start.first, sy = start.second;
    int gx = goal.first, gy = goal.second;

    g_score[sx][sy] = 0.0;
    open.emplace(sx, sy, 0.0, heuristic(sx, sy, gx, gy));

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        int cx = current.x;
        int cy = current.y;

        if (cx == gx && cy == gy) {
            // reconstruct path
            pair<int,int> cur = {cx, cy};
            while (cur.first != -1) {
                path.push_back(cur);
                cur = parent[cur.first][cur.second];
            }
            reverse(path.begin(), path.end());
            return path;
        }

        for (auto d : directions) {
            int nx = cx + d.first;
            int ny = cy + d.second;
            if (!isvalid(nx, ny)) continue;

            double step_cost = (abs(d.first) + abs(d.second) == 2) ? sqrt(2.0) : 1.0;
            double tentative_g = g_score[cx][cy] + step_cost;

            if (tentative_g < g_score[nx][ny]) {
                g_score[nx][ny] = tentative_g;
                double f = tentative_g + heuristic(nx, ny, gx, gy);
                open.emplace(nx, ny, tentative_g, f);
                parent[nx][ny] = {cx, cy};
            }
        }
    }

    // If we reach here → no path found
    return {};
}
