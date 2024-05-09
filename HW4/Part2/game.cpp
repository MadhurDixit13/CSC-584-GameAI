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

void setupGame(Selector*& mainSelector, KinematicData& character, KinematicData& enemy, SteeringData& enemySteering, KinematicData& targetEnemy) {
    // Initialize composite nodes
    Sequence* followCharacterSequence = new Sequence();
    Selector* followCharacterSelector = new Selector();
    Sequence* eatSequence = new Sequence();
    Sequence* followScriptedPathSequence = new Sequence();
    mainSelector = new Selector(); // Main selector for the behavior tree  

    // Construct the behavior tree
    followCharacterSequence->children.push_back(new NearCharacter());
    followCharacterSequence->children.push_back(new Move()); // Move towards character 
    followCharacterSequence->children.push_back(new Collide());
    followCharacterSequence->children.push_back(new Eat());
    // followCharacterSequence->children.push_back(followCharacterSelector);

    // followCharacterSelector->children.push_back(new Collide());
    // followCharacterSelector->children.push_back(eatSequence);

    // eatSequence->children.push_back(new Eat());

    // followScriptedPathSequence->children.push_back(new ObstacleCollide());
    followScriptedPathSequence->children.push_back(new FollowScriptedPath());

    mainSelector->children.push_back(followScriptedPathSequence);
    mainSelector->children.push_back(followCharacterSequence);

}

 
int main() {

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Indoor Environment");
    
    int cellsX = WINDOW_WIDTH / GRID_SIZE;
    int cellsY = WINDOW_HEIGHT / GRID_SIZE;

    const float movementSpeed = 100.0f; 
    const float tolerance = 5.0f; 
    bool follow;
    
    sf::Texture texture;
    if (!texture.loadFromFile("boid.png")) {
        cout << "Could not load the image" << endl;
        return 1;
    }
    sf::Vector2i currentPos={2,1};
    KinematicData character;
    KinematicData enemy;
    KinematicData target;
    KinematicData targetenemy; 
    SteeringData characterSteering;
    SteeringData enemySteering;
    // character.position = sf::Vector2f(40.f, 40.f);
    character.position = sf::Vector2f(700.f, 400.f); 
    enemy.position = sf::Vector2f(760.f, 560.f);
    character.orientation = 0.f;
    character.velocity = sf::Vector2f(0.f, 0.f);
    character.angularVelocity = 0.f;
    characterSteering.linear=sf::Vector2f(0.f, 0.f);
    target.position = sf::Vector2f(40.f, 40.f); 
    characterSteering.angular=0.f;
    sf::CircleShape enemyShape;
    enemyShape.setRadius(10.f);
    enemyShape.setFillColor(sf::Color::Black);
    sf::Sprite mainCharacter;
    mainCharacter.setTexture(texture);
    int index =0;
    int enemyindex = 0;
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
    vector<int> enemypath;
    sf::Clock timer;
    int currentPathIndex = 1;
    EnvironmentState state;
    State enemystate;
    std::unique_ptr<DecisionNode> arrive_node = std::make_unique<ArriveNode>();
    std::unique_ptr<DecisionNode> wander_node = std::make_unique<WanderNode>();
    std::unique_ptr<DecisionNode> root_node = std::make_unique<RootNode>(std::move(arrive_node), std::move(wander_node));
    Selector* mainSelector = nullptr;
    setupGame(mainSelector, character, enemy, enemySteering, targetenemy);
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
      
        if (mainSelector) {
            mainSelector->Run(character, enemy, enemySteering, targetenemy);
        }
        
       

        enemy.update(enemySteering, deltaTime, 50.f, 4.5f);
        previous_time = current_time;
        mainCharacter.setPosition(character.position);
        enemyShape.setPosition(enemy.position);
        mainCharacter.setRotation(character.orientation * 180.0 / M_PI);
        // cout<<character.position.x<<" "<< character.position.y<<endl;
        window.draw(mainCharacter);
        window.draw(enemyShape);


        drawObstaclesAndRooms(window);
        window.display();
    }
      

    return 0;
}






