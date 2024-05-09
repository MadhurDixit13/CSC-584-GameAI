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

bool checkNearObstacle(sf::Vector2f position, const std::vector<std::vector<bool>>& graph){
    int cellsX = WINDOW_WIDTH / GRID_SIZE;
    int cellsY = WINDOW_HEIGHT / GRID_SIZE;
    int x = position.y/GRID_SIZE;
    int y = position.x/GRID_SIZE;

    std::vector<sf::Vector2i> neighbors = {
        {x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}, // Sideways neighbors
        {x - 1, y - 1}, {x - 1, y + 1}, {x + 1, y - 1}, {x + 1, y + 1} // Diagonal neighbors
    };
    // Checking if the current cell is blocked (obstacle or room boundary)
    
    for (const auto& neighbor : neighbors) {
        int nx = neighbor.x;
        int ny = neighbor.y;

                // Checking if the neighbor is within the bounds of the graph
        if (nx >= 0 && nx < cellsY && ny >= 0 && ny < cellsX) {
            if (!graph[nx][ny]) {
                return true;
            }
        }
        else{
            cout<<"Character pos x"<<position.x<<"Character pos y"<<position.y<<endl;
            cout<<"YO"<<endl;
            return true;
        }
        
    }
    return false;

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
    character.position = sf::Vector2f(40.f, 40.f);
    character.orientation = 0.f;
    character.velocity = sf::Vector2f(0.f, 0.f);
    character.angularVelocity = 0.f;
    characterSteering.linear=sf::Vector2f(0.f, 0.f);
    target.position = sf::Vector2f(40.f, 40.f);
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
    EnvironmentState state;
    std::unique_ptr<DecisionNode> arrive_node = std::make_unique<ArriveNode>();
    std::unique_ptr<DecisionNode> wander_node = std::make_unique<WanderNode>();
    std::unique_ptr<DecisionNode> root_node = std::make_unique<RootNode>(std::move(arrive_node), std::move(wander_node));
    // Main loop
    while (window.isOpen()) {
        state.wander = false;
        state.pathFind = false;
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
    
        // for (int x = 0; x <= WINDOW_WIDTH; x += GRID_SIZE) {
        //     sf::Vertex line[] = {
        //     sf::Vertex(sf::Vector2f(x, 0),sf::Color::Black),
        //     sf::Vertex(sf::Vector2f(x, WINDOW_HEIGHT),sf::Color::Black)
        // };
        // window.draw(line, 2, sf::Lines);
        // }
        // for (int y = 0; y <= WINDOW_HEIGHT; y += GRID_SIZE) {
        //     sf::Vertex line[] = {
        //     sf::Vertex(sf::Vector2f(0, y),sf::Color::Black),
        //     sf::Vertex(sf::Vector2f(WINDOW_WIDTH, y),sf::Color::Black)
        // };
        // window.draw(line, 2, sf::Lines); 
        // }
        // AlignBehavior alignBehavior(4.5f, 1.8f, 0.4f, 0.03f);
        // ArriveBehavior arriveBehavior(50.0f, 8.f, 0.1f, 0.08f);
        
        // cout<<"Inside Reached Target"<<endl;
        // characterSteering = wander.calculateSteering(character, target);
        // character.update(characterSteering, deltaTime, wander.maxVelocity, wander.maxRotation);
        // target.position = character.position;
                
       if(!path.empty()){
            // cout<<path[0]<<endl;
            if(index< path.size()){
                target.position.x = coordinates[path[index]].y;
                target.position.y = coordinates[path[index]].x;
                state.pathFind = true;
               
            }
            else{
                index = 0;
                path.clear();
                character.position = target.position;
                characterSteering.linear = sf::Vector2f(0.f,0.f);
                characterSteering.angular = 0.f;
                character.velocity = sf::Vector2f(0.f,0.f);
                character.angularVelocity = 0.f;
            } 
        }
        else{
            state.wander= true;
        }
        characterSteering = root_node->decide(state, character, target, characterSteering);
        if(state.pathFind){
            character.update(characterSteering, deltaTime, 50.f, 4.5f); 
             if(euclideanDistance(target.position, character.position)<7.f){
                    index++;
                }
        }else if (state.wander){
            character.update(characterSteering, deltaTime, 50.f, 3.8f); 
            if (checkNearObstacle(character.position, g)){
                target.position = sf::Vector2f(400.f, 300.f);
                    sf::Vector2i targetPos = {(target.position.x / GRID_SIZE), (target.position.y / GRID_SIZE)};
                    currentPos = {(character.position.x / GRID_SIZE), (character.position.y / GRID_SIZE)};
                    cout<<targetPos.x<<" "<<targetPos.y<<" "<<endl;
                    path =  astar(graph, currentPos.y * cellsX + currentPos.x, targetPos.y * cellsX + targetPos.x, coordinates);
                    index=0;
                    path = reversedVector(path);
                    state.wander = false;
            }
        } 
        //  character.update(characterSteering, deltaTime, 50.f, 4.5f); 
        
        // characterSteering = wander.calculateSteering(character, target);
        
        // character.update(characterSteering, deltaTime, 50.f, alignBehavior.maxRotation);
        previous_time = current_time;
        mainCharacter.setPosition(character.position);
        mainCharacter.setRotation(character.orientation * 180.0 / M_PI);
        cout<<character.position.x<<" "<< character.position.y<<endl;
        window.draw(mainCharacter);


        drawObstaclesAndRooms(window);
        window.display();
    }
      

    return 0;
}






