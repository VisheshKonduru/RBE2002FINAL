#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <Arduino.h>

// Grid dimensions
const int GRID_ROWS = 3;
const int GRID_COLS = 6;

// Representation of the grid (0: open, 1: blocked)
extern int intersections[GRID_ROWS][GRID_COLS];
extern int jPaths[GRID_ROWS][GRID_COLS - 1];
extern int iPaths[GRID_ROWS - 1][GRID_COLS];

class Pathfinding {
private:
    struct Node {
        int i, j, g, h;
        Node* parent;

        Node(int i, int j, int g, int h, Node* parent);
        int f() const;
    };

    // Heuristic function: Manhattan Distance
    int heuristic(int i1, int j1, int i2, int j2);

    // Get valid neighbors based on the grid and path arrays
    void getNeighbors(int i, int j, int* neighbors, int& count);

public:
    // Function to find the path using A* algorithm
    void findPath(int i_start, int j_start, int i_end, int j_end);
};

#endif // PATHFINDING_H
