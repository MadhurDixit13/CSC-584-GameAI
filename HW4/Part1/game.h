#include <cmath>
#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>

using namespace std;

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




float euclideanDistance(const sf::Vector2f& v1, const sf::Vector2f& v2) {
    float dx = v2.x - v1.x;
    float dy = v2.y - v1.y;
    return std::sqrt(dx * dx + dy * dy);
}


// DecisionTreeNode makeDecision(const sf::Vector2f& characterPos, const sf::Vector2f& targetPos, 
//                                const std::vector<std::vector<bool>>& graph) {
//     if (characterPos == targetPos) {
//         if (targetPos.x < 0 || targetPos.y < 0 || targetPos.x >= WINDOW_WIDTH || targetPos.y >= WINDOW_HEIGHT) {
//             return {Decision::OutOfBounds, -1}; // Target is out of bounds
//         }
//         else if (!graph[targetPos.y+20/GRID_SIZE][targetPos.x/GRID_SIZE] || !graph[targetPos.y/GRID_SIZE][targetPos.x+20/GRID_SIZE]||!graph[targetPos.y+20/GRID_SIZE][targetPos.x+20/GRID_SIZE]||!graph[targetPos.y-20/GRID_SIZE][targetPos.x/GRID_SIZE]
//             ||!graph[targetPos.y/GRID_SIZE][targetPos.x-20/GRID_SIZE]||!graph[targetPos.y-20/GRID_SIZE][targetPos.x-20/GRID_SIZE]||!graph[targetPos.y+20/GRID_SIZE][targetPos.x-20/GRID_SIZE]
//             ||!graph[targetPos.y-20/GRID_SIZE][targetPos.x+20/GRID_SIZE]) {
//             return {Decision::NearObstacle, -1}; // Target is near an obstacle
//         }
//         return {Decision::ReachedTargetNode, -1}; // Target reached
//     }
    
//     return {Decision::Default, -1};
// }


//cite Breadcrumbs by Derek Martin
class crumb : sf::CircleShape
{
    public:
        crumb(int id)
        {
            //set initial position and size breadcrumbs   
            this->id = id;         
            this->setRadius(5.f);
            this->setFillColor(sf::Color(0, 0, 255, 255));
            this->setPosition(-100, -100);
        }

        //tell breadcrumb to render self, using current render window
        void draw(sf::RenderWindow* window)
        {
            window->draw(*this);
        }

        //set position of breadcrumb
        void drop(float x, float y)
        {
            this->setPosition(x, y);
        }

        //set position of breadcrumb
        void drop(sf::Vector2f position)
        {
            this->setPosition(position);
        }

    private:
        int id;
};
// Forward declaration of KinematicData
struct KinematicData;

// SteeringData struct definition
struct SteeringData {
    sf::Vector2f linear;
    float angular;
};

// SteeringBehavior class declaration
class SteeringBehavior {
public:
    virtual ~SteeringBehavior() {}
    virtual sf::Vector2f calculateSteering(KinematicData& character, KinematicData& target) = 0;
    float maxVelocity;
    float maxRotation;
};

class WanderSteeringBehavior {
public:
    virtual ~WanderSteeringBehavior() {}
    virtual SteeringData calculateSteering(const KinematicData& character, const KinematicData& target) = 0;
    float maxVelocity;
    float maxRotation;
};

// KinematicData struct definition
struct KinematicData {
    sf::Vector2f position;
    float orientation; 
    sf::Vector2f velocity;
    float angularVelocity; 

    void update(SteeringData steering, float time, float maxVel, float maxRot){
  
    position += velocity * time;

    orientation += angularVelocity * time;

    velocity += steering.linear * time;
    float velocityMagnitude = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    if (velocityMagnitude > maxVel) {
        velocity = (velocity / velocityMagnitude) * maxVel;
    }

    angularVelocity += steering.angular * time;
    if (std::abs(angularVelocity) > maxRot) {
        angularVelocity = (angularVelocity < 0 ? -1 : 1) * maxRot;
    }
}

    sf::Vector2f returnfuturepos(SteeringData steering, float time, float maxVel, float maxRot){
        sf::Vector2f tempposition = position;
        sf::Vector2f tempvelocity;
        tempposition += velocity * time;

    
        return tempposition;
    }
    
};

// VelocityMatching Behaviour
class VelocityMatching : public SteeringBehavior {
public:
    float maxVelocity;
    float maxRotation;
    VelocityMatching(float maxVel, float maxRot) : maxVelocity(maxVel), maxRotation(maxRot) {}
    sf::Vector2f calculateSteering(KinematicData& character, KinematicData& target) override {
        SteeringData steering;
        
        
        sf::Vector2f mouseVelocity = target.position - character.position;
        
       
        steering.linear = mouseVelocity - character.velocity;
        
        return steering.linear;
    }
};

class Wander : public WanderSteeringBehavior {
public:
    float maxVelocity;
    float maxRotation;
    float wanderRadius;
    float wanderDistance;
    float wanderJitter;
    sf::Vector2f wanderTarget;

    Wander(float maxVel, float maxRot, float wanderRad, float wanderDist, float wanderJit)
        : maxVelocity(maxVel), maxRotation(maxRot), wanderRadius(wanderRad), wanderDistance(wanderDist), wanderJitter(wanderJit) {
        }

    SteeringData calculateSteering(const KinematicData& character, const KinematicData& target) override {
        SteeringData steer;
        sf::Vector2f orientationVector(cos(character.orientation), sin(character.orientation));
        steer.linear = maxVelocity * orientationVector;
        steer.angular = randomClamped() * maxRotation;
        // steer.angular = 0.f;
        return steer;
    }

private:
    float randomFloat(float min, float max) {
        return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    }

    float randomClamped() {
        return randomFloat(-1.f, 1.f);
    }

    sf::Vector2f normalize(const sf::Vector2f& vector) {
        float magnitude = std::sqrt(vector.x * vector.x + vector.y * vector.y);
        if (magnitude > 0) {
            return vector / magnitude;
        } else {
            return vector;
        }
    }

};

sf::Vector2f returnFuturePosition(const KinematicData& character, const SteeringData& steer, float timeStep) {
    // Calculate future position
    sf::Vector2f futurePosition = character.position + steer.linear * timeStep;
    return futurePosition;
}

class ArriveBehavior : public SteeringBehavior {
public:
    
    float maxVelocity;
    float slowingRadius;
    float satisfyRadius;
    float timeToTarget;

    ArriveBehavior(float maxSpd, float slowingRad, float satRad, float time_to_target)
        : maxVelocity(maxSpd), slowingRadius(slowingRad), satisfyRadius(satRad), timeToTarget(time_to_target) {}

    sf::Vector2f calculateSteering(KinematicData& character, KinematicData& target) override { 
        sf::Vector2f desiredVelocity = target.position - character.position;
        float distance = std::sqrt(desiredVelocity.x * desiredVelocity.x + desiredVelocity.y * desiredVelocity.y);
        float goalSpeed;
    
        if(distance < satisfyRadius){
            goalSpeed = 0.f;
        }
        else if (satisfyRadius < distance < slowingRadius) {
            goalSpeed = maxVelocity * (distance/slowingRadius);
        } else{
            goalSpeed = maxVelocity;
        }

        sf::Vector2f goalVelocity = desiredVelocity;
        float goalSpeedMagnitude = std::sqrt(goalVelocity.x * goalVelocity.x + goalVelocity.y * goalVelocity.y);

    
        if (goalSpeedMagnitude > 0) {
            goalVelocity /= goalSpeedMagnitude;
        }

        
        goalVelocity *= goalSpeed;
       
        sf::Vector2f steering = goalVelocity - character.velocity;
        steering /= timeToTarget;

        return steering;
    }
};

class AlignBehavior : public SteeringBehavior {
public:
    float maxRotation;
    float rad_dec;
    float rad_sat;
    float timeToTarget;
    AlignBehavior(float maxAngSpeed,  float radDec, float radSat, float time_to_target)
        : maxRotation(maxAngSpeed), rad_dec(radDec), rad_sat(radSat), timeToTarget(time_to_target) {}

    sf::Vector2f calculateSteering(KinematicData& character, KinematicData& target) override {
        float desiredAngularVelocity = target.orientation - character.orientation;

        
        desiredAngularVelocity = fmod(desiredAngularVelocity, static_cast<float>(2 * M_PI));


        if (abs(desiredAngularVelocity) <= M_PI){
            
        } else if (desiredAngularVelocity > M_PI) {
            desiredAngularVelocity -= 2 * M_PI;
        } else {
            desiredAngularVelocity += 2 * M_PI;
        }

        float rotationSize = abs(desiredAngularVelocity);
        float goal_rotation;

        if (rotationSize < rad_sat) {
            goal_rotation = 0;
        } else if (rotationSize > rad_dec) {
            goal_rotation = maxRotation;
        } else {
            goal_rotation = maxRotation * rotationSize / rad_dec;
        }
        if(rotationSize != 0){
            goal_rotation *= desiredAngularVelocity/rotationSize;
        }
        
        float steeringAngularVelocity = goal_rotation - character.angularVelocity;
        steeringAngularVelocity /= timeToTarget;

        return sf::Vector2f(0.f, steeringAngularVelocity);
    }

};

// class PathFollowBehavior : public SteeringBehavior {
// public:
//     float predTime;
//     vector<int> path;
//     std::map<int,sf::Vector2f> coordinates;
//     float pathOffset;

//     PathFollowBehavior(float pTime, vector<int> p, std::map<int,sf::Vector2f> c, float pOffset)
//         : predTime(pTime), path(p), coordinates(c), pathOffset(pOffset){}

//     int findNearestPointOnPath(sf::Vector2f currentPosition, vector<int> path, std::map<int,sf::Vector2f> coordinates){
//     sf::Vector2f currentPositionI = coordinates[currentPosition.y * 40/20 + currentPosition.x/20];

//     std::pair<int, int> nearestPoint;
//     float minDistance = std::numeric_limits<float>::max();
//     int closeIndex = -1;
//     for (int i = 1; i < path.size(); i++){
//         if (coordinates[path[i]] == currentPositionI)
//             continue; // Skip if the current point matches the character's position
        
//         float d = euclideanDistance(currentPositionI, coordinates[path[i]]);
//         if (d < minDistance){
//             minDistance = d;
//             closeIndex = i;
//         }
//     }
//     return closeIndex;
// }

//     sf::Vector2f calculateSteering(KinematicData& character, KinematicData& target) override {
//         sf::Vector2f futurePosition = character.position + (character.velocity * predTime);
//         int closeIndex = findNearestPointOnPath(futurePosition, path, coordinates);
//         int targetIndex = closeIndex + pathOffset;
//         cout<<targetIndex<<endl;
//         if(targetIndex >= path.size()-1)
//         {
//             targetIndex = path.size() - 1;
//         }
//         cout<<targetIndex<<endl;
//         target.position.x = coordinates[path[targetIndex]].y;
//         target.position.y = coordinates[path[targetIndex]].x;
//         // cout<<target.position.x<<" "<<target.position.y<<" "<<endl;
//         ArriveBehavior arriveBehavior(50.0f, 9.f, 4.f, 0.01f);
//         return arriveBehavior.calculateSteering(character, target);
//     }
// };

struct EnvironmentState {
    bool nearObstacle;
    bool wander;
    bool pathFind;
};


class DecisionNode {
public:
    virtual ~DecisionNode() {}
    virtual SteeringData decide(const EnvironmentState& state, KinematicData character, KinematicData target,SteeringData characterSteering) = 0;
};

class ArriveNode : public DecisionNode {
public:
    virtual SteeringData decide(const EnvironmentState& state, KinematicData character, KinematicData target,SteeringData characterSteering) override {
        cout<<"in"<<endl;
        ArriveBehavior arriveBehavior(25.0f, 7.f, 3.f, 0.1f);
        AlignBehavior alignBehavior(7.5f, 1.5f, 0.5f, 0.1f);
        // ArriveBehavior arriveBehavior(50.0f, 9.f, 4.f, 0.01f);
        // AlignBehavior alignBehavior(target.orientation, 3.f, 2.5f, 0.7f, 0.6f);
        // AlignBehavior alignBehavior(target.orientation, 4.5f, 1.9f, 0.35f, 0.03f);
        // AlignBehavior alignBehavior(4.5f, 1.8f, 0.3f, 0.03f);
        characterSteering.linear = arriveBehavior.calculateSteering(character, target);
        // sf::Vector2f orientationSteering = alignBehavior.calculateSteering(character, target);
        // characterSteering.angular = orientationSteering.y;
        characterSteering.angular = 0.f;
        return characterSteering;
    }
};

class WanderNode : public DecisionNode {
public:
    virtual SteeringData decide(const EnvironmentState& state,KinematicData character, KinematicData target,SteeringData characterSteering) override {
        // Wander wander(50.0f, 3.8f, 1.8f, 9.f, 1.f);
        Wander wander(50.0f, 3.8f, 1.8f, 9.f, 1.f);
        characterSteering = wander.calculateSteering(character, target);
        return characterSteering;
    }
};


class RootNode : public DecisionNode {
private:
    std::unique_ptr<DecisionNode> arrive_node;
    std::unique_ptr<DecisionNode> wander_node;
public:
    RootNode(std::unique_ptr<DecisionNode> arrive, std::unique_ptr<DecisionNode> wander) : arrive_node(std::move(arrive)), wander_node(std::move(wander)){}
    virtual SteeringData decide(const EnvironmentState& state, KinematicData character, KinematicData target,SteeringData characterSteering) override {
        if (state.pathFind) {
            // cout<<"in"<<endl;
            return arrive_node->decide(state, character, target, characterSteering);
        } 
        else if(state.wander){
            return wander_node->decide(state, character, target, characterSteering);
        }
    }
};

