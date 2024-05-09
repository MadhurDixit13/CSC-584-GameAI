#include <iostream>
#include <vector>
#include <map>
#include <SFML/Graphics.hpp> 
#include <fstream>
#include <sstream>
#include <queue>
#include <cmath>
#include <limits>
#include <chrono>
using namespace std;

const int INF = numeric_limits<int>::max();

struct Edge {
    int to;
    int weight;
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
    // Doubling the Manhattan distance between "from" node and "to" node as an overestimated heuristic
    float dx = abs(to.x - from.x);
    float dy = abs(to.y - from.y);
    return 2 * (dx + dy);
}


void astar(const vector<vector<Edge>>& graph, int start, int goal, const map<int, sf::Vector2f>& coordinates) {
    auto start_time = chrono::high_resolution_clock::now();
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    vector<int> g_scores(graph.size(), INF);
    vector<int> parents(graph.size(), -1);

    pq.push({start, 0, heuristic(coordinates.at(start), coordinates.at(goal))});
    g_scores[start] = 0;
    int max_fringe_size = 0;


    while (!pq.empty()) {
        max_fringe_size = max(max_fringe_size, static_cast<int>(pq.size()));
        Node current = pq.top();
        pq.pop();

        if (current.id == goal) {
            auto end_time = chrono::high_resolution_clock::now();
            auto runtime = chrono::duration_cast<chrono::microseconds>(end_time - start_time).count();
            cout << "Runtime: " << runtime << " microseconds" << endl;

            vector<int> path;
            int node = goal;
            while (node != -1) {
                path.push_back(node);
                node = parents[node];
            }
            cout << "Shortest path from " << start << " to " << goal << ": ";
            for (int i = path.size() - 1; i >= 0; --i) {
                cout << path[i];
                if (i != 0)
                    cout << " -> ";
            }
            cout << endl;
            // Printing the fill and fringe
            cout << "Fill: " << graph.size() << endl;
            cout << "Fringe: " << max_fringe_size << endl;
            return;
        }

        for (const Edge& edge : graph[current.id]) {
            int neighbor = edge.to;
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
}

int main() {
    map<int, sf::Vector2f> coordinates = {
        {0, sf::Vector2f(0,0)},
        {1, sf::Vector2f(-11,3)},
        {2, sf::Vector2f(-11,-17)},
        {3, sf::Vector2f(-10,15)},
        {4, sf::Vector2f(-3,-8)},
        {5, sf::Vector2f(-5,10)},
        {6, sf::Vector2f(5,-13)},
        {7, sf::Vector2f(10,-14)},
        {8, sf::Vector2f(7,12)},
        {9, sf::Vector2f(4,8)},
        {10, sf::Vector2f(8,8)},
        {11, sf::Vector2f(7,-8)},
        {12, sf::Vector2f(9,-12)},
        {13, sf::Vector2f(9,-11)},
        {14, sf::Vector2f(10,-12)},
        {15, sf::Vector2f(9,-13)},
        {16, sf::Vector2f(8,7)},
        {17, sf::Vector2f(14,8)},
        {18, sf::Vector2f(9,16)},
        {19, sf::Vector2f(18,-8)},
        {20, sf::Vector2f(12,12)},
        {21, sf::Vector2f(13,10)}
    };

    // Read graph data from file
    ifstream infile("smallgraph.txt");
    if (!infile) {
        cerr << "Error opening file." << endl;
        return 1;
    }

    vector<vector<Edge>> graph;
    string line;
    while (getline(infile, line)) {
        istringstream iss(line);
        int fromNodeId, toNodeId, weight;
        if (!(iss >> fromNodeId >> toNodeId >> weight)) {
            cerr << "Error parsing line: " << line << endl;
            continue;
        }
        if (fromNodeId >= graph.size() || toNodeId >= graph.size()) {
            graph.resize(max(fromNodeId, toNodeId) + 1);
        }
        graph[fromNodeId].push_back({toNodeId, weight});
    }

    int start = 2; // Start node
    int goal = 20; // Goal node

    astar(graph, start, goal, coordinates);

    return 0;
}

//test runs 0 - 21 Get same A* as dijkstras
//test runs  2 - 20 - get different A* than Dijkstras