#include <iostream>
#include <vector>
#include <queue>
#include <map>
using namespace std;

class Planner {
public:
    vector<pair<int,int>> pathplanning(vector<vector<int>>& grid, pair<int,int> start, pair<int,int> goal) {
        int n = grid.size();
        int m = grid[0].size();

        vector<vector<bool>> visited(n, vector<bool>(m, false));
        map<pair<int,int>, pair<int,int>> parent;

        queue<pair<int,int>> q;
        q.push(start);
        visited[start.first][start.second] = true;

        int dx[4] = {1, -1, 0, 0};
        int dy[4] = {0, 0, 1, -1};

        while (!q.empty()) {
            auto [x, y] = q.front(); q.pop();
            if (make_pair(x, y) == goal) break;

            for (int k = 0; k < 4; k++) {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx >= 0 && nx < n && ny >= 0 && ny < m &&
                    !visited[nx][ny] && grid[nx][ny] == 0) {
                    visited[nx][ny] = true;
                    parent[{nx, ny}] = {x, y};
                    q.push({nx, ny});
                }
            }
        }

        vector<pair<int,int>> path;
        if (!visited[goal.first][goal.second]) {
            cerr << "No path found!" << endl;
            return path;
        }

        // Reconstruct path
        for (pair<int,int> at = goal; at != start; at = parent[at])
            path.push_back(at);
        path.push_back(start);

        reverse(path.begin(), path.end());
        return path;
    }
};


