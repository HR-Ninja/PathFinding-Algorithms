#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime> // Added for random seed

// Constants for grid dimensions
constexpr int ROWS = 10;
constexpr int COLS = 10;

// Define cell types
enum class CellType { EMPTY, WALL, START, END, PATH, VISITED };

// Define cell structure
struct Cell {
    int row;
    int col;
    CellType type;
};

// Function to print grid
void printGrid(const std::vector<std::vector<Cell>>& grid) {
    system("cls");
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            char symbol;
            switch (grid[i][j].type) {
            case CellType::EMPTY:
                symbol = '.';
                break;
            case CellType::WALL:
                symbol = '#';
                break;
            case CellType::START:
                symbol = 'S';
                break;
            case CellType::END:
                symbol = 'E';
                break;
            case CellType::PATH:
                symbol = '*';
                break;
            case CellType::VISITED:
                symbol = '-';
                break;
            }
            std::cout << symbol << " ";
        }
        std::cout << std::endl;
    }
}

// Function to pause execution for a specified duration
void sleep(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// Function to perform Breadth-First Search (BFS)
bool bfs(std::vector<std::vector<Cell>>& grid, int startRow, int startCol, int endRow, int endCol) {
    std::queue<std::pair<int, int>> q;
    q.push({ startRow, startCol });

    while (!q.empty()) {
        int row = q.front().first;
        int col = q.front().second;
        q.pop();

        if (row == endRow && col == endCol) // Check if we reached the end
            return true;

        const std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };
        for (size_t i = 0; i < directions.size(); ++i) {
            int dx = directions[i].first;
            int dy = directions[i].second;

            int newRow = row + dx;
            int newCol = col + dy;

            if (newRow >= 0 && newRow < ROWS && newCol >= 0 && newCol < COLS &&
                grid[newRow][newCol].type != CellType::WALL && grid[newRow][newCol].type != CellType::VISITED) {
                grid[newRow][newCol].type = CellType::VISITED;
                printGrid(grid);
                sleep(500); // Pause for 500 milliseconds
                if (newRow == endRow && newCol == endCol) // Check if the end point is reached
                    return true;
                q.push({ newRow, newCol });
            }
        }
    }
    return false; // If end is not reached
}

// Function to perform Depth-First Search (DFS)
bool dfs(std::vector<std::vector<Cell>>& grid, int startRow, int startCol, int endRow, int endCol) {
    std::stack<std::pair<int, int>> stk;
    stk.push({ startRow, startCol });

    while (!stk.empty()) {
        int row = stk.top().first;
        int col = stk.top().second;
        stk.pop();

        if (row == endRow && col == endCol) // Check if we reached the end
            return true;

        const std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };
        for (size_t i = 0; i < directions.size(); ++i) {
            int dx = directions[i].first;
            int dy = directions[i].second;

            int newRow = row + dx;
            int newCol = col + dy;

            if (newRow >= 0 && newRow < ROWS && newCol >= 0 && newCol < COLS &&
                grid[newRow][newCol].type != CellType::WALL && grid[newRow][newCol].type != CellType::VISITED) {
                grid[newRow][newCol].type = CellType::VISITED;
                printGrid(grid);
                sleep(500); // Pause for 500 milliseconds
                if (newRow == endRow && newCol == endCol) // Check if the end point is reached
                    return true;
                stk.push({ newRow, newCol });
            }
        }
    }
    return false; // If end is not reached
}

// Function to generate random obstacles in the grid
void generateObstacles(std::vector<std::vector<Cell>>& grid, int numObstacles) {
    srand(time(nullptr)); // Seed the random number generator
    int obstaclesPlaced = 0;
    while (obstaclesPlaced < numObstacles) {
        int row = rand() % ROWS;
        int col = rand() % COLS;
        if (grid[row][col].type == CellType::EMPTY) {
            grid[row][col].type = CellType::WALL;
            ++obstaclesPlaced;
        }
    }
}

int main() {
    std::vector<std::vector<Cell>> grid(ROWS, std::vector<Cell>(COLS, { 0, 0, CellType::EMPTY }));

    // Set start and end points
    int startRow = 1;
    int startCol = 1;
    int endRow = ROWS - 2;
    int endCol = COLS - 2;
    grid[startRow][startCol].type = CellType::START;
    grid[endRow][endCol].type = CellType::END;

    // Generate random obstacles
    generateObstacles(grid, 15); // Adjust the number of obstacles as needed

    // Print initial grid
    printGrid(grid);

    // Choose algorithm
    std::cout << "Select pathfinding algorithm:\n1. Breadth-First Search (BFS)\n2. Depth-First Search (DFS)\nEnter choice: ";
    int choice;
    std::cin >> choice;

    bool reachedEnd = false;
    switch (choice) {
    case 1:
        reachedEnd = bfs(grid, startRow, startCol, endRow, endCol);
        if (reachedEnd) {
            std::cout << "End reached by BFS!\n";
        }
        else {
            std::cout << "End not reached by BFS.\n";
        }
        break;
    case 2:
        reachedEnd = dfs(grid, startRow, startCol, endRow, endCol);
        if (reachedEnd) {
            std::cout << "End reached by DFS!\n";
        }
        else {
            std::cout << "End not reached by DFS.\n";
        }
        break;
    default:
        std::cout << "Invalid choice.\n";
    }

    // Print final grid
    printGrid(grid);

    return 0;
}