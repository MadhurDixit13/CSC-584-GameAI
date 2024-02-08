#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

#include <iostream>

using namespace std;

int main() {
    const int WINDOW_WIDTH = 640;
    const int WINDOW_HEIGHT = 480;
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Game AI");
    
    // vector<sf::Sprite> characters;
    sf::Texture texture;
    sf::Sprite character1;
    // sf::Sprite character2;
    // sf::Sprite character3;
    // sf::Sprite character4;
    if (!texture.loadFromFile("boid.png")) {
        cout << "Could not load the image" << endl;
    }

    texture.setSmooth(true);
    texture.setRepeated(true);
    character1.setTexture(texture);
    // character2.setTexture(texture);
    // character3.setTexture(texture);
    // character4.setTexture(texture);
    character1.setPosition(10.f, 10.f); // Adjust the position as needed
    
    character1.setScale(sf::Vector2f(0.04f, 0.04f)); 
    int frameCounter = 0;
    const int moveInterval = 300;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
        if(frameCounter >= moveInterval){
            character1.move(10.f, 0.f);
            // character1.move(0.f,(10.f)*(480.f/640.f));
            frameCounter = 0;
        }
        if(character1.getPosition().x + character1.getGlobalBounds().width >= WINDOW_WIDTH){
            character1.setPosition(10.f, 10.f);
        }
        // for(int i = 0 ; i <= characters.size() ; i++ ){
        //     if(character1.getPosition().x + character1.getGlobalBounds().width >= WINDOW_WIDTH){

        // }

        // }
        window.clear(sf::Color::White); 
        window.draw(character1);  
        window.display();
        frameCounter++;
    }

    return 0;
}
