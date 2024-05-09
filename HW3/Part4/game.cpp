#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
#include <queue>
#include <cmath>
#include <chrono>
#include "game.h"
#include <climits>
#include <algorithm>

using namespace std;

struct Edge {
    int to;
    float weight;
};

struct Node {
    int id;
    int g_score;
    double f_score; 

    bool operator>(const Node& other) const {
        return f_score > other.f_score;
    }
};

double heuristic(const sf::Vector2f& from, const sf::Vector2f& to) {
    // Calculating Euclidean distance between "from" node and "to" node as heuristic
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    return sqrt(dx * dx + dy * dy);
}

std::vector<int> reversedVector(const std::vector<int>& vec) {
    return std::vector<int>(vec.rbegin(), vec.rend());
}

// Constants for the environment
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int GRID_SIZE = 20; // Size of each grid cell
const int OBSTACLE_SIZE = 4; // Size of each room in grids
const int OBSTACLE1_X = 5; // X position of room 1 (grid index)
const int OBSTACLE1_Y = 4; // Y position of room 1 (grid index)
const int OBSTACLE2_X = 18; // X position of room 2 (grid index)
const int OBSTACLE2_Y = 22; // Y position of room 2 (grid index)
const int OBSTACLE3_X = 31; // X position of room 2 (grid index)
const int OBSTACLE3_Y = 4; // Y position of room 2 (grid index)
const int WALL_WIDTH = 1; // Size of each room in grids
const int WALL_HEIGHT = 12; // Size of each room in grids
const int WALL1_X = 0; // X position of room 1 (grid index)
const int WALL1_Y = 0; 

void drawObstaclesAndRooms(sf::RenderWindow& window) {
    
    sf::RectangleShape obstacle1(sf::Vector2f(GRID_SIZE * OBSTACLE_SIZE, GRID_SIZE * OBSTACLE_SIZE));
    obstacle1.setPosition(GRID_SIZE * OBSTACLE1_X, GRID_SIZE * OBSTACLE1_Y);
    obstacle1.setFillColor(sf::Color::Transparent);
    obstacle1.setOutlineColor(sf::Color::Black);
    obstacle1.setOutlineThickness(2);
    window.draw(obstacle1);

    sf::RectangleShape obstacle2(sf::Vector2f(GRID_SIZE * OBSTACLE_SIZE, GRID_SIZE * OBSTACLE_SIZE));
    obstacle2.setPosition(GRID_SIZE * OBSTACLE2_X, GRID_SIZE * OBSTACLE2_Y);
    obstacle2.setFillColor(sf::Color::Transparent);
    obstacle2.setOutlineColor(sf::Color::Black);
    obstacle2.setOutlineThickness(2);
    window.draw(obstacle2);

    sf::RectangleShape obstacle3(sf::Vector2f(GRID_SIZE * OBSTACLE_SIZE, GRID_SIZE * OBSTACLE_SIZE));
    obstacle3.setPosition(GRID_SIZE * OBSTACLE3_X, GRID_SIZE * OBSTACLE3_Y);
    obstacle3.setFillColor(sf::Color::Transparent);
    obstacle3.setOutlineColor(sf::Color::Black);
    obstacle3.setOutlineThickness(2);
    window.draw(obstacle3);

    sf::RectangleShape wall1(sf::Vector2f(GRID_SIZE * WALL_WIDTH, GRID_SIZE * WALL_HEIGHT));
    wall1.setPosition(GRID_SIZE * WALL1_X, GRID_SIZE * WALL1_Y);
    wall1.setFillColor(sf::Color::Transparent);
    wall1.setOutlineColor(sf::Color::Black);
    wall1.setOutlineThickness(1);
    window.draw(wall1);

    sf::RectangleShape wall2(sf::Vector2f(GRID_SIZE * WALL_HEIGHT, GRID_SIZE * WALL_WIDTH));
    wall2.setPosition(GRID_SIZE * (WALL1_X+1), GRID_SIZE * (WALL1_Y));
    wall2.setFillColor(sf::Color::Transparent);
    wall2.setOutlineColor(sf::Color::Black);
    wall2.setOutlineThickness(1);
    // wall2.setRotation(270);
    window.draw(wall2);

    sf::RectangleShape wall3(sf::Vector2f(GRID_SIZE * WALL_WIDTH, GRID_SIZE * WALL_HEIGHT));
    wall3.setPosition(GRID_SIZE * (WALL1_X+13), GRID_SIZE * (WALL1_Y));
    wall3.setFillColor(sf::Color::Transparent);
    wall3.setOutlineColor(sf::Color::Black);
    wall3.setOutlineThickness(1);
    window.draw(wall3);

    sf::RectangleShape wall4(sf::Vector2f(GRID_SIZE * WALL_WIDTH, GRID_SIZE * WALL_HEIGHT));
    wall4.setPosition(GRID_SIZE * (WALL1_X+13), GRID_SIZE * (WALL1_Y + 18));
    wall4.setFillColor(sf::Color::Transparent);
    wall4.setOutlineColor(sf::Color::Black);
    wall4.setOutlineThickness(1);
    window.draw(wall4);

    sf::RectangleShape wall5(sf::Vector2f(GRID_SIZE * WALL_WIDTH, GRID_SIZE * WALL_HEIGHT));
    wall5.setPosition(GRID_SIZE * (WALL1_X+26), GRID_SIZE * (WALL1_Y + 18));
    wall5.setFillColor(sf::Color::Transparent);
    wall5.setOutlineColor(sf::Color::Black);
    wall5.setOutlineThickness(1);
    window.draw(wall5);

    sf::RectangleShape wall6(sf::Vector2f(GRID_SIZE * WALL_HEIGHT, GRID_SIZE * WALL_WIDTH));
    wall6.setPosition(GRID_SIZE * (WALL1_X+14), GRID_SIZE * (WALL1_Y + 29));
    wall6.setFillColor(sf::Color::Transparent);
    wall6.setOutlineColor(sf::Color::Black);
    wall6.setOutlineThickness(1);
    // wall6.setRotation(90);
    window.draw(wall6);

    sf::RectangleShape wall7(sf::Vector2f(GRID_SIZE * WALL_WIDTH, GRID_SIZE * WALL_HEIGHT));
    wall7.setPosition(GRID_SIZE * (WALL1_X+26), GRID_SIZE * (WALL1_Y));
    wall7.setFillColor(sf::Color::Transparent);
    wall7.setOutlineColor(sf::Color::Black);
    wall7.setOutlineThickness(1);
    window.draw(wall7);

    sf::RectangleShape wall8(sf::Vector2f(GRID_SIZE * WALL_HEIGHT, GRID_SIZE * WALL_WIDTH));
    wall8.setPosition(GRID_SIZE * (WALL1_X+27), GRID_SIZE * (WALL1_Y));
    wall8.setFillColor(sf::Color::Transparent);
    wall8.setOutlineColor(sf::Color::Black);
    wall8.setOutlineThickness(1);
    // wall8.setRotation(270);
    window.draw(wall8);

    sf::RectangleShape wall9(sf::Vector2f(GRID_SIZE * WALL_WIDTH, GRID_SIZE * WALL_HEIGHT));
    wall9.setPosition(GRID_SIZE * (WALL1_X+39), GRID_SIZE * (WALL1_Y));
    wall9.setFillColor(sf::Color::Transparent);
    wall9.setOutlineColor(sf::Color::Black);
    wall9.setOutlineThickness(1);
    window.draw(wall9);
}


bool isObstacle(const sf::Vector2f& position) {
    
    int x = position.x;
    int y = position.y;

    // Checking if the current grid cell is an obstacle
    if ((x >= OBSTACLE1_Y * GRID_SIZE && y >= OBSTACLE1_X* GRID_SIZE && x < (OBSTACLE1_Y* GRID_SIZE + GRID_SIZE * OBSTACLE_SIZE) && y < (OBSTACLE1_X* GRID_SIZE + GRID_SIZE * OBSTACLE_SIZE)) ||
        (x >= OBSTACLE2_Y * GRID_SIZE&& y >= OBSTACLE2_X * GRID_SIZE&& x < (OBSTACLE2_Y* GRID_SIZE + GRID_SIZE * OBSTACLE_SIZE) && y < (OBSTACLE2_X* GRID_SIZE + GRID_SIZE * OBSTACLE_SIZE)) ||
        (x >= OBSTACLE3_Y * GRID_SIZE&& y >= OBSTACLE3_X* GRID_SIZE && x < (OBSTACLE3_Y* GRID_SIZE + GRID_SIZE * OBSTACLE_SIZE) && y < (OBSTACLE3_X* GRID_SIZE + GRID_SIZE * OBSTACLE_SIZE))) {
        return true;
    }

    // Returning false for non-obstacle cells
    return false;
}

// Function for detecting room boundaries
bool isRoomBoundary(const sf::Vector2f& position) {
    
    int x = position.x;
    int y = position.y;

    // Checking if the current grid cell is a room boundary
    if ((y >= WALL1_Y* GRID_SIZE && x >= WALL1_X* GRID_SIZE && y < (WALL1_Y* GRID_SIZE + GRID_SIZE * WALL_WIDTH) && x < (WALL1_X* GRID_SIZE + GRID_SIZE * WALL_HEIGHT)) ||
        (y >= (WALL1_Y+1)* GRID_SIZE && x >= WALL1_X* GRID_SIZE && y < ((WALL1_Y+1)* GRID_SIZE + GRID_SIZE * WALL_HEIGHT) && x < (WALL1_X* GRID_SIZE + GRID_SIZE * WALL_WIDTH)) ||
        (y >= (WALL1_Y+13)* GRID_SIZE && x >= WALL1_X* GRID_SIZE && y < ((WALL1_Y+13)* GRID_SIZE + GRID_SIZE * WALL_WIDTH) && x < (WALL1_X* GRID_SIZE + GRID_SIZE * WALL_HEIGHT)) || 
        (y >= (WALL1_Y+13)* GRID_SIZE && x >= (WALL1_X + 18)* GRID_SIZE && y < ((WALL1_Y+13)* GRID_SIZE + GRID_SIZE * WALL_WIDTH) && x < ((WALL1_X+18)* GRID_SIZE + GRID_SIZE * WALL_HEIGHT)) ||
        (y >= (WALL1_Y+26)* GRID_SIZE && x >= (WALL1_X + 18)* GRID_SIZE && y < ((WALL1_Y+26)* GRID_SIZE + GRID_SIZE * WALL_WIDTH) && x < ((WALL1_X+18)* GRID_SIZE + GRID_SIZE * WALL_HEIGHT)) ||
        (y >= (WALL1_Y+14)* GRID_SIZE && x >= (WALL1_X + 29)* GRID_SIZE && y < ((WALL1_Y+14)* GRID_SIZE + GRID_SIZE * WALL_HEIGHT) && x < ((WALL1_X+29) * GRID_SIZE+ GRID_SIZE * WALL_WIDTH)) ||
        (y >= (WALL1_Y+26)* GRID_SIZE && x >= (WALL1_X)* GRID_SIZE && y < ((WALL1_Y+26)* GRID_SIZE + GRID_SIZE * WALL_WIDTH) && x < (WALL1_X* GRID_SIZE + GRID_SIZE * WALL_HEIGHT)) ||
        (y >= (WALL1_Y+27)* GRID_SIZE && x >= (WALL1_X)* GRID_SIZE && y < ((WALL1_Y+27)* GRID_SIZE + GRID_SIZE * WALL_HEIGHT) && x < (WALL1_X* GRID_SIZE + GRID_SIZE * WALL_WIDTH)) || 
        (y >= (WALL1_Y+39)* GRID_SIZE && x >= (WALL1_X)* GRID_SIZE && y < ((WALL1_Y+39)* GRID_SIZE + GRID_SIZE * WALL_WIDTH) && x < (WALL1_X* GRID_SIZE + GRID_SIZE * WALL_HEIGHT)) ) {
        return true;
    }

    // Returning false for non-room boundary cells
    return false;
}


std::vector<std::vector<bool>> createGraph() {
    int cellsX = WINDOW_WIDTH / GRID_SIZE;
    int cellsY = WINDOW_HEIGHT / GRID_SIZE;

    std::vector<std::vector<bool>> graph(cellsY+1, std::vector<bool>(cellsX, true));

    // Detecting obstacles and mark corresponding nodes as false
    for (int x = 0; x < cellsY; x++) {
        for (int y = 0; y < cellsX; y++) {
            sf::Vector2f position(x * GRID_SIZE, y * GRID_SIZE);
            if (isObstacle(position)) {
                graph[x][y] = false;
            }
        }
    }


    // Detecting room boundaries and mark corresponding nodes as false
    for (int x = 0; x < cellsY; x++) {
        for (int y = 0; y < cellsX; y++) {
            sf::Vector2f position(x * GRID_SIZE, y * GRID_SIZE);
            if (isRoomBoundary(position)) {
                graph[x][y] = false;
            }
        }
    }

    return graph;
}

std::map<int, sf::Vector2f> createCoordinates(const std::vector<std::vector<bool>>& graph){
    std::map<int, sf::Vector2f> coordinates;
    int cellsX = WINDOW_WIDTH / GRID_SIZE;
    int cellsY = WINDOW_HEIGHT / GRID_SIZE;
    int i = 0;
    for (int x = 0; x < cellsY; x++) {
        for (int y = 0; y < cellsX; y++) {
            coordinates[i] = sf::Vector2f(x * GRID_SIZE, y * GRID_SIZE);
            i++;
        }
    }

    return coordinates;
}

std::vector<std::vector<Edge>> createCostGraph(const std::vector<std::vector<bool>>& graph) {

    int cellsX = WINDOW_WIDTH / GRID_SIZE;
    int cellsY = WINDOW_HEIGHT / GRID_SIZE;

    // Defining the sideways and diagonal movement costs
    const float sidewaysCost = 1.0f;
    const float diagonalCost = std::sqrt(2.0f);

            
    // Initializing the cost graph with appropriate sizes
    std::vector<std::vector<Edge>> costGraph(cellsX * cellsY);

    for (int x = 0; x < cellsY; x++) {
        for (int y = 0; y < cellsX; y++) {
            std::vector<sf::Vector2i> neighbors = {
                {x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}, // Sideways neighbors
                {x - 1, y - 1}, {x - 1, y + 1}, {x + 1, y - 1}, {x + 1, y + 1} // Diagonal neighbors
            };
            // Checking if the current cell is blocked (obstacle or room boundary)
            if (!graph[x][y]) {
                for (const auto& neighbor : neighbors) {
                int nx = neighbor.x;
                int ny = neighbor.y;

                // Checking if the neighbor is within the bounds of the graph
                if (nx >= 0 && nx < cellsY && ny >= 0 && ny < cellsX) {
                    Edge edgeInstance;
                    edgeInstance.to = nx * cellsX + ny;
                    edgeInstance.weight = 0.f;
                    costGraph[x * cellsX + y].push_back(edgeInstance);
                }
                }
            }
            else{
            for (const auto& neighbor : neighbors) {
                int nx = neighbor.x;
                int ny = neighbor.y;

                if (nx >= 0 && nx < cellsY && ny >= 0 && ny < cellsX) {
                    if (!graph[nx][ny]) {
                        Edge edgeInstance;
                        edgeInstance.to = nx * cellsX + ny;
                        edgeInstance.weight = 0.f;
                        costGraph[x * cellsX + y].push_back(edgeInstance);
                    }
                    else{

                    float neighborCost = (std::abs(nx - x) + std::abs(ny - y) == 1) ? sidewaysCost : diagonalCost;

                    Edge edgeInstance;
                    edgeInstance.to = nx * cellsX + ny;
                    edgeInstance.weight = neighborCost;
                    costGraph[x * cellsX + y].push_back(edgeInstance);
                    }
                }
            }
            }
        }
    }

    return costGraph;
}

vector<int> astar(const vector<vector<Edge>>& graph, int start, int goal, const map<int, sf::Vector2f>& coordinates) {
    const int INF = INT_MAX; 
    

    priority_queue<Node, vector<Node>, greater<Node>> pq;
    vector<int> g_scores(graph.size(), INF);
    vector<int> parents(graph.size(), -1);

    pq.push({start, 0, heuristic(coordinates.at(start), coordinates.at(goal))});
    g_scores[start] = 0;

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.id == goal) {
            
            vector<int> path;
            int node = goal;
            int totalCost = 0; // Initialize total cost to zero
            while (node != -1) {
                path.push_back(node);
                node = parents[node];
            }
            cout << "Shortest path from " << start << " to " << goal << ": ";
            for (int i = path.size() - 1; i >= 0; --i) {
                cout << path[i];
                if (i != 0)
                    cout << " -> ";
                if (i > 0) {
                    // Calculate the cost of the edge from current node to its parent
                    for (const Edge& edge : graph[path[i]]) {
                        if (edge.to == path[i - 1]) {
                            totalCost += edge.weight; // Add the edge weight to the total cost
                            break;
                        }
                    }
                }
            }
            cout << endl;
            cout << "Total cost: " << totalCost << endl; // Print the total cost
            return path;
        }

        for (const Edge& edge : graph[current.id]) {
            int neighbor = edge.to;
            
            // Skip processing zero-cost edges (representing obstacles)
            if (edge.weight == 0)
                continue;
                
            int tentative_g_score = current.g_score + edge.weight;
            if (tentative_g_score < g_scores[neighbor]) {
                parents[neighbor] = current.id;
                g_scores[neighbor] = tentative_g_score;
                double f_score = tentative_g_score + heuristic(coordinates.at(neighbor), coordinates.at(goal)); // Change to double
                pq.push({neighbor, tentative_g_score, f_score});
            }
        }
    }

    cout << "No path exists from " << start << " to " << goal << endl;
    return {};
}

bool isValid(int x, int y, const std::vector<std::vector<bool>>& graph) {
    int cellsX = graph.size();
    int cellsY = graph[0].size();
    return (x >= 0 && x < cellsX && y >= 0 && y < cellsY && graph[x][y]);
}
// float euclideanDistance(const sf::Vector2f& v1, const sf::Vector2f& v2) {
//     float dx = v2.x - v1.x;
//     float dy = v2.y - v1.y;
//     return std::sqrt(dx * dx + dy * dy);
// }

int main() {

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Indoor Environment");
    const std::vector<std::vector<bool>> g = createGraph();
    std::vector<std::vector<Edge>> graph = createCostGraph(g);
    std::map<int, sf::Vector2f> coordinates = createCoordinates(g);
    int cellsX = WINDOW_WIDTH / GRID_SIZE;
    int cellsY = WINDOW_HEIGHT / GRID_SIZE;

    const float movementSpeed = 100.0f; 
    const float tolerance = 5.0f; 


    
    sf::Texture texture;
    if (!texture.loadFromFile("boid.png")) {
        cout << "Could not load the image" << endl;
        return 1;
    }
    sf::Vector2i currentPos={2,1};
    KinematicData character;
    KinematicData target;
    SteeringData characterSteering;
    character.position = sf::Vector2f(40.f, 20.f);
    character.orientation = 0.f;
    character.velocity = sf::Vector2f(0.f, 0.f);
    character.angularVelocity = 0.f;
    characterSteering.linear=sf::Vector2f(0.f, 0.f);
    characterSteering.angular=0.f;
    sf::Sprite mainCharacter;
    mainCharacter.setTexture(texture);
    int index =0;
    float scaleX = static_cast<float>(GRID_SIZE) / mainCharacter.getLocalBounds().width;
    float scaleY = static_cast<float>(GRID_SIZE) / mainCharacter.getLocalBounds().height;

    // Set the scale of the character sprite or shape
    mainCharacter.setScale(sf::Vector2f(GRID_SIZE / mainCharacter.getLocalBounds().width,
                                      GRID_SIZE / mainCharacter.getLocalBounds().height));
    mainCharacter.setPosition(sf::Vector2f(currentPos.x * GRID_SIZE, currentPos.y * GRID_SIZE));

    texture.setSmooth(true);
    texture.setRepeated(true);
    auto clock_func = []() { return std::chrono::high_resolution_clock::now(); };
    auto previous_time = clock_func();
    vector<int> path;
    sf::Clock timer;
    int currentPathIndex = 1;
    // Main loop
    while (window.isOpen()) {

        // Handling events
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            } else if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                    sf::Vector2i targetPos = {(mousePos.x / GRID_SIZE), (mousePos.y / GRID_SIZE)};
                    currentPos = {(character.position.x / GRID_SIZE), (character.position.y / GRID_SIZE)};
                    cout<<targetPos.x<<" "<<targetPos.y<<" "<<endl;
                    path =  astar(graph, currentPos.y * cellsX + currentPos.x, targetPos.y * cellsX + targetPos.x, coordinates);
                    index=0;
                    path = reversedVector(path);
                    // for (int num : path) {
                    //     std::cout << num << " ";
                    // }  
                }
            }
        }
        
        
        auto current_time = clock_func();
        auto delta_time = current_time - previous_time;

        if (delta_time.count() < 0) {
            delta_time = std::chrono::duration<long long>::zero();
        }

        auto delta_time_in_seconds = std::chrono::duration_cast<std::chrono::duration<float>>(delta_time);
        float deltaTime = delta_time_in_seconds.count();

        

        window.clear(sf::Color::White);
    
    //     for (int x = 0; x <= WINDOW_WIDTH; x += GRID_SIZE) {
    //         sf::Vertex line[] = {
    //         sf::Vertex(sf::Vector2f(x, 0),sf::Color::Black),
    //         sf::Vertex(sf::Vector2f(x, WINDOW_HEIGHT),sf::Color::Black)
    //     };
    //     window.draw(line, 2, sf::Lines);
    //     }
    //     for (int y = 0; y <= WINDOW_HEIGHT; y += GRID_SIZE) {
    //         sf::Vertex line[] = {
    //         sf::Vertex(sf::Vector2f(0, y),sf::Color::Black),
    //         sf::Vertex(sf::Vector2f(WINDOW_WIDTH, y),sf::Color::Black)
    //     };
    //     window.draw(line, 2, sf::Lines); 
    // }
        AlignBehavior alignBehavior(7.5f, 1.5f, 0.5f, 0.1f);
        ArriveBehavior arriveBehavior(25.0f, 7.f, 3.f, 0.1f);
        if(!path.empty()){
            // cout<<path[0]<<endl;
            if(index< path.size()){
                target.position.x = coordinates[path[index]].y;
                target.position.y = coordinates[path[index]].x;
                characterSteering.linear = arriveBehavior.calculateSteering(character, target);
                sf::Vector2f angularOutput = alignBehavior.calculateSteering(character,target);
                characterSteering.angular = angularOutput.y;
                // cout<<target.position.x<<" "<<target.position.x<<" "<<endl;
                cout<<"Euclidean"<<euclideanDistance(target.position, character.position)<<endl;
                if(euclideanDistance(target.position, character.position)<9.f){
                    index++;
                }
            }
            else{
                index = 0;
                path.clear();
                characterSteering.linear = sf::Vector2f(0.f,0.f);
                characterSteering.angular = 0.f;
                character.velocity = sf::Vector2f(0.f,0.f);
                character.angularVelocity = 0.f;
            }
        }
        character.update(characterSteering, deltaTime, 50.f, alignBehavior.maxRotation);
        previous_time = current_time;
        mainCharacter.setPosition(character.position);
        mainCharacter.setRotation(character.orientation * 180.0 / M_PI);

        window.draw(mainCharacter);


        drawObstaclesAndRooms(window);
        window.display();
    }
    

    return 0;
}






