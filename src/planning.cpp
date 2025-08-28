#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <algorithm>
using namespace std;

// A simple grid-based Planner class
class Planner {
public:
    Planner(int w, int h) : width(w), height(h) {
        grid.resize(height, vector<int>(width, 0));
    }

    // Block a cell (make it an obstacle)
    void setObstacle(int x, int y) {
        if (inBounds(x, y)) {
            grid[y][x] = 1; // 1 = obstacle
        }
    }

    // Path planning function (BFS for shortest path)
    vector<pair<int, int>> pathplanning(pair<int, int> start, pair<int, int> goal) {
        vector<pair<int, int>> path;

        if (!inBounds(start.first, start.second) || 
            !inBounds(goal.first, goal.second)) {
            cout << "Invalid start or goal position!" << endl;
            return path;
        }

        vector<vector<int>> visited(height, vector<int>(width, 0));
        vector<vector<pair<int,int>>> parent(height, vector<pair<int,int>>(width, {-1, -1}));

        queue<pair<int,int>> q;
        q.push(start);
        visited[start.second][start.first] = 1;

        // 4 possible moves (up, down, left, right)
        int dx[4] = {1, -1, 0, 0};
        int dy[4] = {0, 0, 1, -1};

        bool found = false;

        while (!q.empty()) {
            auto [x, y] = q.front(); q.pop();

            if (x == goal.first && y == goal.second) {
                found = true;
                break;
            }

            for (int i = 0; i < 4; i++) {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (inBounds(nx, ny) && !visited[ny][nx] && grid[ny][nx] == 0) {
                    visited[ny][nx] = 1;
                    parent[ny][nx] = {x, y};
                    q.push({nx, ny});
                }
            }
        }

        if (found) {
            // Backtrack to build path
            pair<int,int> cur = goal;
            while (cur != make_pair(-1, -1)) {
                path.push_back(cur);
                cur = parent[cur.second][cur.first];
            }
            reverse(path.begin(), path.end());
        } else {
            cout << "No path found!" << endl;
        }

        return path;
    }

private:
    int width, height;
    vector<vector<int>> grid;

    bool inBounds(int x, int y) {
        return (x >= 0 && x < width && y >= 0 && y < height);
    }
};




