#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

#include <iostream>
#include <vector>

using namespace std;

int main() {
    const int WINDOW_WIDTH = 640;
    const int WINDOW_HEIGHT = 480;
    const float startX = 20.f;
    const float startY = 20.f;
    const int moveInterval = 200; // Move every 200 frames

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Game AI");
    sf::Texture texture;

    if (!texture.loadFromFile("boid.png")) {
        cout << "Could not load the image" << endl;
        return 1;
    }
    //cite - SFML textures tutorial - https://www.sfml-dev.org/tutorials/2.6/graphics-sprite.php

    texture.setSmooth(true);
    texture.setRepeated(true);

    vector<sf::Sprite> characters;
    vector<int> direction;
    // vector<sf::Vector2f> startPos;
    int frameCounter = 0;
    bool erase = false;
    bool eraseflag = false;
    // Main Game Loop
    while (window.isOpen()) {
        bool flag = false;
        eraseflag = false;
        int eraseIndex = 0;  
        sf::Event event;
        while (window.pollEvent(event)) {
            // Window Close Event (default SFML events)
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
        // Initialize if 0
        if (characters.empty()) {
            cout<<"HERE"<<endl;
            erase = false;
            flag = false;
            eraseflag = false;
            sf::Sprite character;
            character.setTexture(texture);
            character.setPosition(startX, startY);
            // startPos.push_back(sf::Vector2f(startX, startY));
            character.setScale(sf::Vector2f(0.04f, 0.04f)); 
            characters.push_back(character);
            direction.push_back(1);
        }
        // Game logic based on number of frames passed
        if (frameCounter >= moveInterval) {
            // Move all sprites
            for (int i = 0; i < characters.size(); i++) {
                if(direction[i] == 1){
                    characters[i].move(WINDOW_WIDTH/50.f , 0.f);
                    if(characters[i].getPosition().x + characters[i].getGlobalBounds().width >= WINDOW_WIDTH - 20.f){
                        direction[i] = 2;
                        characters[i].rotate(90.f);
                        if(direction[0] == direction[i]){
                            cout<<"true"<<endl;
                            flag = true;
                        }
                        
                    }
                }
                else if(direction[i] == 2){
                    characters[i].move(0.f, WINDOW_HEIGHT/50.f);
                    if(characters[i].getPosition().y + characters[i].getGlobalBounds().height >= WINDOW_HEIGHT - 20.f){
                        direction[i] = 3;
                        characters[i].rotate(90.f);
                        // characters[i].setPosition(startPos[i]);
                        if(direction[0] == direction[i]){
                            cout<<"true"<<endl;
                            flag = true;
                        }
                    }
                }
                else if(direction[i] == 3)
                {
                    characters[i].move(-WINDOW_WIDTH/50.f, 0.f);
                    if(characters[i].getPosition().x - characters[i].getGlobalBounds().width <= 20.f){
                        // characters[i].setPosition(startPos[i]);
                        direction[i] = 4;
                        characters[i].rotate(90.f);
                        if(direction[0] == direction[i]){
                            cout<<"true"<<endl;
                            flag = true;
                        }
                    }
                }
                else if(direction[i] == 4){
                    characters[i].move(0.f, -WINDOW_HEIGHT/50.f );
                    if(characters[i].getPosition().y - characters[i].getGlobalBounds().height   <= 20.f){
                        // characters[i].setPosition(startPos[i]);
                        eraseIndex = i;
                        if(direction[0] == direction[i]){
                            cout<<"true"<<endl;
                            flag = true;
                        }
                        eraseflag = true;
                    }
                }
            }
            // Erase logic
            if (erase) {
                // Erase the front element of the vector
                if(eraseflag){
                    characters.erase(characters.begin());
                direction.erase(direction.begin());

                }
                // Set erase to false so that only one element is erased at a time
                // erase = false;
            } 
            else{
            // Create new sprite when previous character hits boundary and repeat motion
            if ( characters.size() < 4 && flag) {
                cout<<"First"<<characters.size()<<endl;
                sf::Sprite newCharacter;
                newCharacter.setTexture(texture);
                newCharacter.setPosition(startX, startY);
                // newCharacter.rotate(90.f);
                // startPos.push_back(sf::Vector2f(startX, startY));
                newCharacter.setScale(sf::Vector2f(0.04f, 0.04f)); 
                characters.push_back(newCharacter);
                direction.push_back(1);
                
            } else if(characters.size() == 4 && flag){
                cout<<"2"<<characters.size()<<endl;
                // characters.erase(characters.begin());
                // direction.erase(direction.begin());
                // startPos.erase(startPos.begin());
                erase = true;
            }
            }
            frameCounter = 0;
        }

        window.clear(sf::Color::White);

        // Draw all sprites
        for (const sf::Sprite& character : characters) {
            window.draw(character);
        }

        window.display();

        frameCounter++;
    }

    return 0;
}
