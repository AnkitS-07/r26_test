vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
    vector<pair<int, int>> path; 
    if (!isvalid(start.first, start.second) || !isvalid(goal.first, goal.second))
        return path; // invalid start/goal

    // Directions: up, down, left, right
    vector<pair<int,int>> directions = {{-1,0},{1,0},{0,-1},{0,1}};

    // g cost (distance from start)
    vector<vector<double>> g(rows, vector<double>(cols, 1e9));
    g[start.first][start.second] = 0;

    // f cost (g + heuristic)
    vector<vector<double>> f(rows, vector<double>(cols, 1e9));
    f[start.first][start.second] = heuristic(start.first, start.second, goal.first, goal.second);

    // parent matrix for path reconstruction
    vector<vector<pair<int,int>>> parent(rows, vector<pair<int,int>>(cols, {-1,-1}));

    // Min-heap: (f_score, (x,y))
    using Node = pair<double, pair<int,int>>;
    priority_queue<Node, vector<Node>, greater<Node>> openSet;
    openSet.push({f[start.first][start.second], start});

    while (!openSet.empty()) {
        auto current = openSet.top().second;
        openSet.pop();

        if (current == goal) {
            // Reconstruct path
            while (current != make_pair(-1,-1)) {
                path.push_back(current);
                current = parent[current.first][current.second];
            }
            reverse(path.begin(), path.end());
            return path;
        }

        for (auto &dir : directions) {
            int nx = current.first + dir.first;
            int ny = current.second + dir.second;

            if (!isvalid(nx, ny)) continue;

            double tentative_g = g[current.first][current.second] + 1.0; // cost to move = 1
            if (tentative_g < g[nx][ny]) {
                parent[nx][ny] = current;
                g[nx][ny] = tentative_g;
                f[nx][ny] = tentative_g + heuristic(nx, ny, goal.first, goal.second);
                openSet.push({f[nx][ny], {nx, ny}});
            }
        }
    }

    return path; // return empty if no path found
}
