# AI Techniques for Games

## Project Overview
This project, developed for the Building Game AI course (CSC584), involves implementing various AI techniques used in games, such as movement, pathfinding, path following, and decision trees.

## Sections

### Section 1: Introduction to SFML
- Introduction to the SFML library in C++.
- Setting up a basic game window with a sprite.

### Section 2: Steering Behaviors
- Implementation of various steering behaviors including:
  - Arrive
  - Align
  - Wander
  - Velocity Matching
  - Collision Detection and Avoidance
- Implementation of the Boids flocking algorithm.

### Section 3: Pathfinding Algorithms
- Implementation and performance comparison of shortest pathfinding algorithms between two points:
  - Dijkstra's Algorithm
  - A* Algorithm

### Section 4: Game Development
- Using concepts from previous sections (pathfinding, wander, align, etc.) to develop a game.
- Features a character and an enemy that follows the character when in proximity.

## How to Run
1. Download and extract the project zip file.
2. Open a terminal and navigate to the extracted folder.
3. Install the required dependencies and libraries.
4. Run the following commands in the desired folder to see the implementation:
   ```bash
   make
   ./main
   
## System Specifications
- **Operating System:**
  - Ubuntu 20.04 (tested)
  - Should work on Windows, macOS, and other versions of Linux (not tested)
## Dependencies
1. x11-apps
2. build-essential
3. libsfml-dev

## Author and Acknowledgement
**Author:** Madhur Dixit

## Project Status
Completed
