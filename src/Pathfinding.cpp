#include "Pathfinding.h"
#include "queue.h"

// Representation of the grid (0: open, 1: blocked)
int intersections[GRID_ROWS][GRID_COLS] = {
    {0, 1, 0, 1, 1, 1},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
};

int jPaths[GRID_ROWS][GRID_COLS - 1] = {
    {0, 0, 0, 1, 1},
    {0, 1, 0, 1, 1},
    {0, 0, 1, 0, 1},
};

int iPaths[GRID_ROWS - 1][GRID_COLS] = {
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
};

Pathfinding::Node::Node(int i, int j, int g, int h, Node* parent) 
    : i(i), j(j), g(g), h(h), parent(parent) {}

int Pathfinding::Node::f() const {
    return g + h;
}

int Pathfinding::heuristic(int i1, int j1, int i2, int j2) {
    return abs(i1 - i2) + abs(j1 - j2);
}

void Pathfinding::getNeighbors(int i, int j, int* neighbors, int& count) {
    count = 0;

    // Move Left (check if left is open and not blocked)
    if (j > 0 && jPaths[i][j - 1] == 0 && intersections[i][j - 1] == 0) {
        neighbors[count++] = (i * GRID_COLS + (j - 1));
    }

    // Move Right (check if right is open and not blocked)
    if (j < GRID_COLS - 1 && jPaths[i][j] == 0 && intersections[i][j + 1] == 0) {
        neighbors[count++] = (i * GRID_COLS + (j + 1));
    }

    // Move Up (check if up is open and not blocked)
    if (i > 0 && iPaths[i - 1][j] == 0 && intersections[i - 1][j] == 0) {
        neighbors[count++] = ((i - 1) * GRID_COLS + j);
    }

    // Move Down (check if down is open and not blocked)
    if (i < GRID_ROWS - 1 && iPaths[i][j] == 0 && intersections[i + 1][j] == 0) {
        neighbors[count++] = ((i + 1) * GRID_COLS + j);
    }
}

void Pathfinding::findPath(int i_start, int j_start, int i_end, int j_end) {
    Queue<Node*> openSet; // Queue to hold nodes to process
    bool visited[GRID_ROWS][GRID_COLS] = {false};
    int neighbors[4];  // Max 4 neighbors for each node
    int neighborCount = 0;

    if (intersections[i_start][j_start] == 1 || intersections[i_end][j_end] == 1) {
        Serial.println("Start or End is blocked.");
        return;
    }

    // Add start node to openSet
    Node* startNode = new Node(i_start, j_start, 0, heuristic(i_start, j_start, i_end, j_end), nullptr);
    openSet.enqueue(startNode);

    while (!openSet.isEmpty()) {
        Node* current = openSet.dequeue();

        if (current->i == i_end && current->j == j_end) {
            // Reconstruct path
            Serial.println("Path found:");
            Node* temp = current;
            while (temp) {
                Serial.print("(");
                Serial.print(temp->i);
                Serial.print(",");
                Serial.print(temp->j);
                Serial.print(") ");
                temp = temp->parent;
            }
            Serial.println();
            return;
        }

        visited[current->i][current->j] = true;

        // Get neighbors and process them
        getNeighbors(current->i, current->j, neighbors, neighborCount);
        for (int i = 0; i < neighborCount; i++) {
            int ni = neighbors[i] / GRID_COLS;
            int nj = neighbors[i] % GRID_COLS;

            if (!visited[ni][nj]) {
                int g = current->g + 1;
                int h = heuristic(ni, nj, i_end, j_end);
                Node* neighborNode = new Node(ni, nj, g, h, current);
                openSet.enqueue(neighborNode);
            }
        }
    }

    Serial.println("No path found.");
}
