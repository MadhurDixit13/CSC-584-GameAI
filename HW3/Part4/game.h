#include <cmath>
#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>

using namespace std;
const sf::Vector2f TOP_RIGHT = sf::Vector2f(800, 0);
const sf::Vector2f BOT_RIGHT = sf::Vector2f(800, 600);
const sf::Vector2f BOT_LEFT = sf::Vector2f(0, 600);
const sf::Vector2f TOP_LEFT = sf::Vector2f(0, 0);

float euclideanDistance(const sf::Vector2f& v1, const sf::Vector2f& v2) {
    float dx = v2.x - v1.x;
    float dy = v2.y - v1.y;
    return std::sqrt(dx * dx + dy * dy);
}

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

// SteeringBehavior class declaration
class SteeringBehavior {
public:
    virtual ~SteeringBehavior() {}
    virtual sf::Vector2f calculateSteering(KinematicData& character, KinematicData& target) = 0;
    float maxVelocity;
    float maxRotation;
};

// SteeringData struct definition
struct SteeringData {
    sf::Vector2f linear;
    float angular;
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
    AlignBehavior(float maxAngSpeed, float radDec, float radSat, float time_to_target)
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



class PathFollowBehavior : public SteeringBehavior {
public:
    float predTime;
    vector<int> path;
    std::map<int,sf::Vector2f> coordinates;
    float pathOffset;

    PathFollowBehavior(float pTime, vector<int> p, std::map<int,sf::Vector2f> c, float pOffset)
        : predTime(pTime), path(p), coordinates(c), pathOffset(pOffset){}

    int findNearestPointOnPath(sf::Vector2f currentPosition, vector<int> path, std::map<int,sf::Vector2f> coordinates){
    sf::Vector2f currentPositionI = coordinates[currentPosition.y * 40/20 + currentPosition.x/20];

    std::pair<int, int> nearestPoint;
    float minDistance = std::numeric_limits<float>::max();
    int closeIndex = -1;
    for (int i = 1; i < path.size(); i++){
        if (coordinates[path[i]] == currentPositionI)
            continue; // Skip if the current point matches the character's position
        
        float d = euclideanDistance(currentPositionI, coordinates[path[i]]);
        if (d < minDistance){
            minDistance = d;
            closeIndex = i;
        }
    }
    return closeIndex;
}

    sf::Vector2f calculateSteering(KinematicData& character, KinematicData& target) override {
        sf::Vector2f futurePosition = character.position + (character.velocity * predTime);
        int closeIndex = findNearestPointOnPath(futurePosition, path, coordinates);
        int targetIndex = closeIndex + pathOffset;
        cout<<targetIndex<<endl;
        if(targetIndex >= path.size()-1)
        {
            targetIndex = path.size() - 1;
        }
        cout<<targetIndex<<endl;
        target.position.x = coordinates[path[targetIndex]].y;
        target.position.y = coordinates[path[targetIndex]].x;
        // cout<<target.position.x<<" "<<target.position.y<<" "<<endl;
        ArriveBehavior arriveBehavior(50.0f, 9.f, 4.f, 0.01f);
        return arriveBehavior.calculateSteering(character, target);
    }
};