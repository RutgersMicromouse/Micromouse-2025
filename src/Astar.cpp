#include "Astar.h"
#include "API.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <stack>
#include <vector>

// Direction constants
#define N 0
#define NE 1
#define E 2
#define SE 3
#define S 4
#define SW 5
#define W 6
#define NW 7


std::string getdir(int dir){
    switch(dir){
        case 0:
            return "N";
        case 1:
            return "NE";
        case 2:
            return "E";
        case 3:
            return "SE";
        case 4:
            return "S";
        case 5:
            return "SW";
        case 6:
            return "W";
        case 7:
            return "NW";
        default:
            return "BAD";
    }
    return NULL;
}
struct PointHash {
    size_t operator()(const point& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

struct PointEqual {
    bool operator()(const point& a, const point& b) const {
        return a.x == b.x && a.y == b.y;
    }
};

typedef struct {
    point curr;
    point parent;
    uint8_t direction;
    float g_weight;
    float h_weight;
} meta_data;

typedef struct {
    point location;
    uint8_t direction;
} astar_mouse;

// Structure to store movement commands
typedef struct {
    uint8_t direction;
    int distance;
} move_command;

astar_mouse redieeem;
std::unordered_map<point, meta_data, PointHash, PointEqual> mazeMap;
std::vector<move_command> path_commands;

struct Compare {
    bool operator()(const meta_data& a, const meta_data& b) {
        return (a.g_weight + a.h_weight) >= (b.g_weight + a.h_weight); // Min-queue
    }
};

std::priority_queue<meta_data, std::vector<meta_data>, Compare> queue;
point destination;
float Kh = 1;

float get_heuristic(point x) {
    float length = destination.x - x.x;
    float height = destination.y - x.y;
    return Kh * sqrt(length * length + height * height);
}

void executeAction(int relativeDirection, int dist) {
    switch (relativeDirection) {
        case 0: // Forward
            API::moveForwardHalf(dist);
            break;
        case 1: // Right 45°
            API::turnRight45();
            API::moveForwardHalf(dist);
            break;
        case 2: // Right 90°
            API::turnRight();
            API::moveForwardHalf(dist);
            break;
        case 3: // Right 135°
            API::turnRight();
            API::turnRight45();
            API::moveForwardHalf(dist);
            break;
        case 4: // Turn around (180°)
            API::turnRight();
            API::turnRight();
            API::moveForwardHalf(dist);
            break;
        case 5: // Left 135°
            API::turnLeft();
            API::turnLeft45();
            API::moveForwardHalf(dist);
            break;
        case 6: // Left 90°
            API::turnLeft();
            API::moveForwardHalf(dist);
            break;
        case 7: // Left 45°
            API::turnLeft45();
            API::moveForwardHalf(dist);
            break;
        default:
            exit(1); // Invalid relative direction
    }
}

void realturn(uint8_t direction, int distance) {
    // Compute the relative direction (0-7)
    int relativeDirection = (direction - redieeem.direction + 8) % 8;
    
    // Execute the corresponding action
    executeAction(relativeDirection, distance);
    
    // Update the current direction
    redieeem.direction = direction;
}

void Astar(uint8_t x, uint8_t y) {
    redieeem.location.x = mouse.location.x;
    redieeem.location.y = mouse.location.y;
    redieeem.direction = mouse.direction;
    destination.x = x;
    destination.y = y;
    
    meta_data start = {{1, 1}, {1, 1}, static_cast<uint8_t>(100), 0.0, 0.0};
    queue.push(start);
    mazeMap[start.curr] = start;
    uint16_t nodes_explored = 0;
    
    while (!queue.empty()) {
        meta_data point = queue.top();
        nodes_explored++;
        queue.pop();
        
        if (point.curr.x == destination.x && point.curr.y == destination.y) break;
        
        uint8_t x = point.curr.x;
        uint8_t y = point.curr.y;
        API::setColor(x/2, y/2, 'y');
        
        auto process_neighbor = [&](int nx, int ny, uint8_t dir) {
            if (nx >= 0 && nx < 33 && ny >= 0 && ny < 33 && maze[nx][ny].visited) {
                meta_data neighbor;
                neighbor.curr = {static_cast<uint8_t>(nx), static_cast<uint8_t>(ny)};
                neighbor.parent = point.curr;
                neighbor.direction = dir;
                
                // Base movement cost
                neighbor.g_weight = point.g_weight + 0.5;
                
                // Add turning penalty if direction changes
                if (point.direction != 100 && neighbor.direction != point.direction) {
                    neighbor.g_weight += 1;
                }
                
                neighbor.h_weight = get_heuristic(neighbor.curr);
                
                if (mazeMap.find(neighbor.curr) == mazeMap.end() || neighbor.g_weight < mazeMap[neighbor.curr].g_weight) {
                    mazeMap[neighbor.curr] = neighbor;
                    queue.push(neighbor);
                }
            }
        };
        
        // Process all neighbors
        process_neighbor(x, y + 1, N);
        process_neighbor(x + 1, y + 1, NE);
        process_neighbor(x + 1, y, E);
        process_neighbor(x + 1, y - 1, SE);
        process_neighbor(x, y - 1, S);
        process_neighbor(x - 1, y - 1, SW);
        process_neighbor(x - 1, y, W);
        process_neighbor(x - 1, y + 1, NW);
    }
    
    std::cerr << "A star end" << std::endl;
    fprintf(stderr, "a star: nodes expanded: %d\n", nodes_explored);
    
    if (mazeMap.find(destination) == mazeMap.end()) {
        std::cerr << "No path found to destination." << std::endl;
        return;
    }
    
    // Reconstruct the path and group directions
    std::vector<uint8_t> raw_path;
    meta_data back_boi = mazeMap[destination];
    
    while (back_boi.curr.x != 1 || back_boi.curr.y != 1) {
        raw_path.push_back(back_boi.direction);
        if (mazeMap.find(back_boi.parent) == mazeMap.end()) {
            std::cerr << "Error: Backtracking failed, parent not found!" << std::endl;
            return;
        }
        back_boi = mazeMap[back_boi.parent];
    }
    
    // Reverse the path and group same directions
    if (!raw_path.empty()) {
        uint8_t current_direction = raw_path.back();
        int distance_count = 1;
        
        for (int i = raw_path.size() - 2; i >= 0; i--) {
            if (raw_path[i] == current_direction) {
                // Same direction, increase distance
                distance_count++;
            } else {
                // Direction changed, add the current group to commands
                path_commands.push_back({current_direction, distance_count});
                current_direction = raw_path[i];
                distance_count = 1;
            }
        }
        
        // Add the final group
        path_commands.push_back({current_direction, distance_count});
    }
    
    // Execute the grouped movement commands
    for (const auto& cmd : path_commands) {
        std::cerr<<getdir(cmd.direction)<<static_cast<int>(cmd.distance)<<std::endl;
        realturn(cmd.direction, cmd.distance);
    }
}