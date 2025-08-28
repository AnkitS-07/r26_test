#include "planning.h"
#include <queue>
#include <iostream>
#include <unordered_map>

using namespace std;

// Node structure for BFS
struct Node {
    int row, col;
    Node* parent;
    Node(int r, int c, Node* p = nullptr) : row(r), col(c), parent(p) {}
};

// Directions (up, down, left, right)
const int dR[4] = {-1, 1, 0, 0};
const int dC[4] = {0, 0, -1, 1};

Planning::Planning(Gridmapper& gridmapper) : gm(gridmapper) {}

bool Planning::findPath(const GPS& start, const GPS& goal, vector<pair<int,int>>& path) {
    auto startCell = gm.gpstogrid(start);
    auto goalCell = gm.gpstogrid(goal);

    int rows = gm.getRows();
    int cols = gm.getCols();
    auto grid = gm.getGrid();

    if (!gm.isvalid(startCell.first, startCell.second) || 
        !gm.isvalid(goalCell.first, goalCell.second)) {
        cerr << "Invalid start or goal!" << endl;
        return false;
    }

    if (grid[startCell.first][startCell.second] || grid[goalCell.first][goalCell.second]) {
        cerr << "Start or goal is blocked!" << endl;
        return false;
    }

    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    queue<Node*> q;

    Node* startNode = new Node(startCell.first, startCell.second);
    q.push(startNode);
    visited[startCell.first][startCell.second] = true;

    Node* goalNode = nullptr;

    while (!q.empty()) {
        Node* current = q.front();
        q.pop();

        if (current->row == goalCell.first && current->col == goalCell.second) {
            goalNode = current;
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nr = current->row + dR[i];
            int nc = current->col + dC[i];

            if (gm.isvalid(nr, nc) && !visited[nr][nc] && !grid[nr][nc]) {
                visited[nr][nc] = true;
                q.push(new Node(nr, nc, current));
            }
        }
    }

    if (!goalNode) {
        cerr << "No path found!" << endl;
        return false;
    }

    // Backtrack path
    Node* temp = goalNode;
    while (temp) {
        path.push_back({temp->row, temp->col});
        temp = temp->parent;
    }
    reverse(path.begin(), path.end());

    return true;
}

void Planning::printPath(const vector<pair<int,int>>& path) {
    cout << "Path (row, col):" << endl;
    for (auto& p : path) {
        cout << "(" << p.first << "," << p.second << ") ";
    }
    cout << endl;
}


