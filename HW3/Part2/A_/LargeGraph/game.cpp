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
    // Calculating Euclidean distance between "from" node and "to" node as heuristic
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    return sqrt(dx * dx + dy * dy);
}

void astar(const vector<vector<Edge>>& graph, int start, int goal, const map<int, sf::Vector2f>& coordinates) {
    auto start_time = chrono::high_resolution_clock::now();

    priority_queue<Node, vector<Node>, greater<Node>> pq;
    vector<int> g_scores(graph.size(), INF);
    vector<int> parents(graph.size(), -1);

    pq.push({start, 0, heuristic(coordinates.at(start), coordinates.at(goal))});
    g_scores[start] = 0;

    // Tracking the maximum size of the priority queue (fringe)
    int max_fringe_size = 0;

    while (!pq.empty()) {
        // Updating the maximum fringe size
        max_fringe_size = max(max_fringe_size, static_cast<int>(pq.size()));

        Node current = pq.top();
        pq.pop();

        if (current.id == goal) {
            // Measuring the elapsed time
            auto end_time = chrono::high_resolution_clock::now();
            auto runtime = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
            cout << "Runtime: " << runtime << " milliseconds" << endl;

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
                double f_score = tentative_g_score + heuristic(coordinates.at(neighbor), coordinates.at(goal));
                pq.push({neighbor, tentative_g_score, f_score});
            }
        }
    }

    cout << "No path exists from " << start << " to " << goal << endl;
}

int main() {
    map<int, sf::Vector2f> coordinates;

    // Reading coordinates from txt file
    ifstream coord_file("node_coordinates.txt");
    if (!coord_file) {
        cerr << "Error opening coordinates file." << endl;
        return 1;
    }

    string line;
    getline(coord_file, line);
    while (getline(coord_file, line)) {
        istringstream iss(line);
        int nodeId;
        float x, y;
        char comma;
        if (!(iss >> nodeId >> comma >> x >> comma >> y)) {
            cerr << "Error parsing coordinates line: " << line << endl;
            continue;
        }
        coordinates[nodeId] = sf::Vector2f(x, y);
    }

    // Reading graph data from txt file
    ifstream infile("largegraph.txt");
    if (!infile) {
        cerr << "Error opening graph file." << endl;
        return 1;
    }

    getline(infile, line);
    vector<vector<Edge>> graph;
    while (getline(infile, line)) {
        istringstream iss(line);
        int fromNodeId, toNodeId, weight;
        if (!(iss >> fromNodeId >> toNodeId >> weight)) {
            cerr << "Error parsing graph line: " << line << endl;
            continue;
        }
        if (fromNodeId >= graph.size() || toNodeId >= graph.size()) {
            graph.resize(max(fromNodeId, toNodeId) + 1);
        }
        graph[fromNodeId].push_back({toNodeId, weight});
    }

    int start = 0; // Start node
    int goal = 1696406; // Goal node

    astar(graph, start, goal, coordinates);

    return 0;
}
