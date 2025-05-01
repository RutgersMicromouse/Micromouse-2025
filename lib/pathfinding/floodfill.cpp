#include "./floodfill.h"


#include "API.h"
#include<queue>
#include<stack>
#include<cstdio>
static int steps = 0;
typedef struct
{
  uint8_t x;
  uint8_t y;
} point;

typedef struct{
  point location;
  uint8_t direction;
} mouse_t;

typedef struct{
  float weight;
  point parent;
  bool visited;
} cell;


cell maze[33][33]; // Center is 14, 14
mouse_t mouse;
point dest;
int i = 0;
static void stall(){
    API::turnLeft(mouse.direction);
    API::turnLeft(mouse.direction);
    API::turnLeft(mouse.direction);
    API::turnLeft(mouse.direction);
}

uint16_t floodfill_expand = 0;
#define N 0
#define E 2
#define S 4
#define W 6

static void fill_maze(){
    char buf[50];
    for(int i = 1; i < 33; i += 2){
        for(int j = 1; j < 33; j += 2){
            snprintf(buf,sizeof(buf),"%.1lf",maze[i][j].weight);
            API::setText(i/2,j/2,buf);
        }
    }
}



static bool hasnorthwall(point p) {
    if (p.y == 32) {
        return true; // Edge of the maze, treat as a wall.
    }
    return maze[p.x][p.y + 1].weight == -2;
}

static bool hassouthwall(point p) {
    if (p.y == 0) {
        return true; // Edge of the maze, treat as a wall.
    }
    return maze[p.x][p.y - 1].weight == -2;
}

static bool haseastwall(point p) {
    if (p.x == 32) {
        return true; // Edge of the maze, treat as a wall.
    }
    return maze[p.x + 1][p.y].weight == -2;
}

static bool haswestwall(point p) {
    if (p.x == 0) {
        return true; // Edge of the maze, treat as a wall.
    }
    return maze[p.x - 1][p.y].weight == -2;
}

static void reflood() {
    std::queue<point> myqueue;
    // std::stack<point> myqueue;
    myqueue.push(dest);
    // Reset weights except for blocked cells
    for (int i = 0; i < 33; i++) {
        for (int j = 0; j < 33; j++) {
            if (maze[i][j].weight != -2) {
                maze[i][j].weight = INFINITY;
            }
        }
    }

    maze[dest.x][dest.y].weight = 0;

    while (!myqueue.empty()) {
        point temp = myqueue.front();
        // point temp = myqueue.top();
        myqueue.pop();

        // Look at neighbors
        if (temp.y + 1 < 33 && !hasnorthwall(temp) && maze[temp.x][temp.y].weight + 0.5 < maze[temp.x][temp.y + 1].weight) {
            maze[temp.x][temp.y + 1].weight = maze[temp.x][temp.y].weight + 0.5;
            myqueue.push({temp.x, static_cast<uint8_t>(temp.y + 1)});
        }
        if (temp.y > 0 && !hassouthwall(temp) && maze[temp.x][temp.y].weight + 0.5 < maze[temp.x][temp.y - 1].weight) {
            maze[temp.x][temp.y - 1].weight = maze[temp.x][temp.y].weight + 0.5;
            myqueue.push({temp.x, static_cast<uint8_t>(temp.y - 1)});
        }
        if (temp.x + 1 < 33 && !haseastwall(temp) && maze[temp.x][temp.y].weight + 0.5 < maze[temp.x + 1][temp.y].weight) {
            maze[temp.x + 1][temp.y].weight = maze[temp.x][temp.y].weight + 0.5;
            myqueue.push({static_cast<uint8_t>(temp.x + 1), temp.y});
        }
        if (temp.x > 0 && !haswestwall(temp) && maze[temp.x][temp.y].weight + 0.5 < maze[temp.x - 1][temp.y].weight) {
            maze[temp.x - 1][temp.y].weight = maze[temp.x][temp.y].weight + 0.5;
            myqueue.push({static_cast<uint8_t>(temp.x - 1), temp.y});
        }
    }

    fill_maze();
}



void initialize_maze(uint8_t x, uint8_t y,bool reset){
    dest.x = x;
    dest.y = y;
    std::queue<point> myqueue;

    if (reset) {
        for (int i = 0; i < 33; i++) {
            for (int j = 0; j < 33; j++) {
                maze[i][j].weight = -1;
            }
        }
        mouse.location.x = mouse.location.y = 1;
        mouse.direction = N;
    }
    else{
        for (int i = 0; i < 33; i++) {
            for (int j = 0; j < 33; j++) {
                if(maze[i][j].weight != -2){
                    maze[i][j].weight = -1;
                }
            }
        }  
    }

    myqueue.push(dest);
    maze[dest.x][dest.y].weight = 0.0;

    while (!myqueue.empty()) {
        point temp = myqueue.front();
        myqueue.pop();

        // Look at neighbors
        if (!hasnorthwall(temp) && maze[temp.x][temp.y + 1].weight == -1) {
            maze[temp.x][temp.y + 1].weight = maze[temp.x][temp.y].weight + 0.5f;
            myqueue.push({temp.x, static_cast<uint8_t>(temp.y + 1)});
        }
        if (!hassouthwall(temp) && maze[temp.x][temp.y - 1].weight == -1) {
            maze[temp.x][temp.y - 1].weight = maze[temp.x][temp.y].weight + 0.5f;
            myqueue.push({temp.x, static_cast<uint8_t>(temp.y - 1)});
        }
        if (!haseastwall(temp) && maze[temp.x + 1][temp.y].weight == -1) {
            maze[temp.x + 1][temp.y].weight = maze[temp.x][temp.y].weight + 0.5f;
            myqueue.push({static_cast<uint8_t>(temp.x + 1), temp.y});
        }
        if (!haswestwall(temp) && maze[temp.x - 1][temp.y].weight == -1) {
            maze[temp.x - 1][temp.y].weight = maze[temp.x][temp.y].weight + 0.5f;
            myqueue.push({static_cast<uint8_t>(temp.x - 1), temp.y});
        }
    }

    // Print the maze weights

    for(int i = 0; i < 33; i++){
        maze[i][0].weight = -2;
        maze[i][32].weight = -2;
        maze[0][i].weight = -2;
        maze[32][i].weight = -2;
        API::setWall(i/2,0,'s');
        API::setWall(i/2,15,'n');
        API::setWall(0,i/2,'w');
        API::setWall(15,i/2,'e');
    }

    // for (int i = 0; i < 33; i++) {
    //     for (int j = 0; j < 33; j++) {
    //         printf("%.2f  ", maze[i][j].weight);
    //     }
    //     printf("\n");
    // }
    fill_maze();
}

static void setwalls(mouse_t mike) {
    uint8_t x = mike.location.x >> 1;
    uint8_t y = mike.location.y >> 1;
    if (mike.direction == N) {
        if (API::wallLeft()) {
            maze[mike.location.x - 1][mike.location.y].weight = -2;
            maze[mike.location.x - 1][mike.location.y + 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y - 1].weight = -2;
            API::setWall(x, y, 'w');
        }
        if (API::wallRight()) {
            maze[mike.location.x + 1][mike.location.y].weight = -2;
            maze[mike.location.x + 1][mike.location.y + 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y - 1].weight = -2;
            API::setWall(x, y, 'e');
        }
        if (API::wallFront()) {
            maze[mike.location.x][mike.location.y + 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y + 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y + 1].weight = -2;
            API::setWall(x, y, 'n');
        }
    } else if (mike.direction == S) {
        if (API::wallLeft()) {
            maze[mike.location.x + 1][mike.location.y].weight = -2;
            maze[mike.location.x + 1][mike.location.y - 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y + 1].weight = -2;
            API::setWall(x, y, 'e');
        }
        if (API::wallRight()) {
            maze[mike.location.x - 1][mike.location.y].weight = -2;
            maze[mike.location.x - 1][mike.location.y - 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y + 1].weight = -2;
            API::setWall(x, y, 'w');
        }
        if (API::wallFront()) {
            maze[mike.location.x][mike.location.y - 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y - 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y - 1].weight = -2;
            API::setWall(x, y, 's');
        }
    } else if (mike.direction == E) {
        if (API::wallLeft()) {
            maze[mike.location.x][mike.location.y + 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y + 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y + 1].weight = -2;
            API::setWall(x, y, 'n');
        }
        if (API::wallRight()) {
            maze[mike.location.x][mike.location.y - 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y - 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y - 1].weight = -2;
            API::setWall(x, y, 's');
        }
        if (API::wallFront()) {
            maze[mike.location.x + 1][mike.location.y].weight = -2;
            maze[mike.location.x + 1][mike.location.y + 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y - 1].weight = -2;
            API::setWall(x, y, 'e');
        }
    } else if (mike.direction == W) {
        if (API::wallLeft()) {
            maze[mike.location.x][mike.location.y - 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y - 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y - 1].weight = -2;
            API::setWall(x, y, 's');
        }
        if (API::wallRight()) {
            maze[mike.location.x][mike.location.y + 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y + 1].weight = -2;
            maze[mike.location.x + 1][mike.location.y + 1].weight = -2;
            API::setWall(x, y, 'n');
        }
        if (API::wallFront()) {
            maze[mike.location.x - 1][mike.location.y].weight = -2;
            maze[mike.location.x - 1][mike.location.y + 1].weight = -2;
            maze[mike.location.x - 1][mike.location.y - 1].weight = -2;
            API::setWall(x, y, 'w');
        }
    }

}

static void realturn(uint8_t direction){
    // Serial.printf("direction: %d\n",direction);
    // while(1); 
    if(mouse.direction == N){
        if(direction == N){
            API::moveForward(1,mouse.direction);
            mouse.location.y += 2;
        }
        else if(direction == E){
            API::turnRight(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.x += 2;
        }
        else if(direction == S){
            API::turnRight(mouse.direction);
            API::turnRight(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.y -= 2;
        }
        else if(direction  == W){
            API::turnLeft(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.x -= 2;
        }
    }
    else if(mouse.direction  == E){
        if(direction  == E){
            API::moveForward(1,mouse.direction);
            mouse.location.x += 2;
        }
        else if(direction == S){
            API::turnRight(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.y -= 2;
        }
        else if(direction  == W){
            API::turnRight(mouse.direction);
            API::turnRight(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.x -= 2;
        }
        else if(direction  == N){
            API::turnLeft(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.y += 2;
        }
    }
    else if(mouse.direction == S){
        if(direction == S){
            API::moveForward(1,mouse.direction);
            mouse.location.y -= 2;
        }
        else if(direction  == W){
            API::turnRight(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.x -= 2;
        }
        else if(direction  == N){
            API::turnRight(mouse.direction);
            API::turnRight(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.y += 2;
        }
        else if(direction  == E){
            API::turnLeft(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.x += 2;
        }
    }
    else if(mouse.direction  == W){
        if(direction  == W){
            API::moveForward(1,mouse.direction);
            mouse.location.x -= 2;
        }
        else if(direction  == N){
            API::turnRight(mouse.direction);
            
            API::moveForward(1,mouse.direction);
            mouse.location.y += 2;
        }
        else if(direction  == E){
            API::turnRight(mouse.direction);
            API::turnRight(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.x += 2;
        }
        else if(direction == S){
            API::turnLeft(mouse.direction);
            API::moveForward(1,mouse.direction);
            mouse.location.y -= 2;
        }
    }
    mouse.direction = direction;
}

static void set_visited(point p){
    if(!maze[p.x][p.y].visited){
        floodfill_expand++;
    }
    maze[p.x][p.y].visited = true;
}
void floodfill() {
    // while(1){
    //     Serial.printf("Left Wall: %d\t Front Wall: %d\t Right Wall: %d\n", API::wallLeft(), API::wallFront(), API::wallRight());

    // }
    while (mouse.location.x != dest.x || mouse.location.y != dest.y) {
        
        setwalls(mouse);
        set_visited(mouse.location);
THING:

        float max_weight = INFINITY;
        char dec_direction;
        API::setColor(mouse.location.x/2,mouse.location.y/2,'g');
        // Check north
        if (mouse.location.y < 31 && !hasnorthwall(mouse.location) && maze[mouse.location.x][mouse.location.y + 2].weight < max_weight) {
            max_weight = maze[mouse.location.x][mouse.location.y + 2].weight;
            maze[mouse.location.x][mouse.location.y + 1].visited = 1;
            dec_direction = N;
        }

        // Check south
        if (mouse.location.y > 1 && !hassouthwall(mouse.location) && maze[mouse.location.x][mouse.location.y - 2].weight < max_weight) {
            max_weight = maze[mouse.location.x][mouse.location.y - 2].weight;
            maze[mouse.location.x][mouse.location.y - 1].visited = 1;
            dec_direction = S;
        }

        // Check east
        if (mouse.location.x < 31 && !haseastwall(mouse.location) && maze[mouse.location.x + 2][mouse.location.y].weight < max_weight) {
            max_weight = maze[mouse.location.x + 2][mouse.location.y].weight;
            maze[mouse.location.x + 1][mouse.location.y].visited = 1;
            dec_direction = E;
        }

        // Check west
        if (mouse.location.x > 1 && !haswestwall(mouse.location) && maze[mouse.location.x - 2][mouse.location.y].weight < max_weight) {
            max_weight = maze[mouse.location.x - 2][mouse.location.y].weight;
            maze[mouse.location.x - 1][mouse.location.y].visited = 1;
            dec_direction = W;
        }
        if(max_weight != maze[mouse.location.x][mouse.location.y].weight - 1){
            reflood();
            goto THING;
        }
        else{
            Serial.printf("direction: %d\n",dec_direction);
            realturn(dec_direction);
            floodfill_expand++;
        }
    }
}