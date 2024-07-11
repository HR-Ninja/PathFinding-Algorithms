#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <chrono>
#include <thread>
#include <random>
#include <limits>

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
    std::cout << "\033[H\033[J"; // ANSI escape code to clear screen
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

// Helper function to check if the cell is valid for visiting
bool isValidCell(const std::vector<std::vector<Cell>>& grid, int newRow, int newCol) {
    return newRow >= 0 && newRow < ROWS && newCol >= 0 && newCol < COLS &&
        grid[newRow][newCol].type != CellType::WALL && grid[newRow][newCol].type != CellType::VISITED;
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
        for (const auto& dir : directions) {
            int newRow = row + dir.first;
            int newCol = col + dir.second;

            if (isValidCell(grid, newRow, newCol)) {
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
        for (const auto& dir : directions) {
            int newRow = row + dir.first;
            int newCol = col + dir.second;

            if (isValidCell(grid, newRow, newCol)) {
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
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> disRow(0, ROWS - 1);
    std::uniform_int_distribution<> disCol(0, COLS - 1);

    int obstaclesPlaced = 0;
    while (obstaclesPlaced < numObstacles) {
        int row = disRow(gen);
        int col = disCol(gen);
        if (grid[row][col].type == CellType::EMPTY) {
            grid[row][col].type = CellType::WALL;
            ++obstaclesPlaced;
        }
    }
}

int main() {
    while (true) {
        std::vector<std::vector<Cell>> grid(ROWS, std::vector<Cell>(COLS, { 0, 0, CellType::EMPTY }));

        // Initialize the grid cells with their coordinates
        for (int i = 0; i < ROWS; ++i) {
            for (int j = 0; j < COLS; ++j) {
                grid[i][j] = { i, j, CellType::EMPTY };
            }
        }

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
        std::cout << "Select pathfinding algorithm:\n1. Breadth-First Search (BFS)\n2. Depth-First Search (DFS)\n3. Exit\nEnter choice: ";
        int choice;
        while (!(std::cin >> choice) || (choice < 1 || choice > 3)) {
            std::cin.clear(); // Clear the error flag
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
            std::cout << "Invalid choice. Please enter 1 for BFS, 2 for DFS, or 3 to Exit: ";
        }

        if (choice == 3) {
            break; // Exit the program
        }

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
        }

        // Print final grid
        printGrid(grid);

        std::cout << "Press Enter to return to the main menu...";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();
    }

    return 0;
}
