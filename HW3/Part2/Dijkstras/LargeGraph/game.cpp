#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <queue>
#include <limits>
#include <chrono> 

using namespace std;

const int INF = numeric_limits<int>::max();

struct Edge {
    int to;
    int weight;
};

void dijkstra(vector<vector<Edge>>& graph, int start, int goal) {
    // Defining a clock for measuring time
    auto start_time = chrono::high_resolution_clock::now();

    vector<int> distance(graph.size(), INF);
    vector<int> parent(graph.size(), -1);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    distance[start] = 0;
    pq.push(make_pair(0, start));

    // Tracking the maximum size of the priority queue (fringe)
    int max_fringe_size = 0;

    while (!pq.empty()) {
        // Updating the maximum fringe size
        max_fringe_size = max(max_fringe_size, static_cast<int>(pq.size()));

        int u = pq.top().second;
        pq.pop();

        for (const Edge& e : graph[u]) {
            int v = e.to;
            int w = e.weight;
            if (distance[v] > distance[u] + w) {
                distance[v] = distance[u] + w;
                parent[v] = u;
                pq.push(make_pair(distance[v], v));
            }
        }
    }

    // Measuring the elapsed time
    auto end_time = chrono::high_resolution_clock::now();
    auto runtime = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
    cout << "Runtime: " << runtime << " milliseconds" << endl;

    if (distance[goal] == INF) {
        cout << "No path exists from " << start << " to " << goal << endl;
        return;
    }

    cout << "Shortest path length from " << start << " to " << goal << " is: " << distance[goal] << endl;

    cout << "Path: ";
    vector<int> path;
    for (int v = goal; v != -1; v = parent[v])
        path.push_back(v);
    for (int i = path.size() - 1; i >= 0; --i) {
        cout << path[i];
        if (i != 0)
            cout << " -> ";
    }
    cout << endl;

    // Printing the fill and fringe
    cout << "Fill: " << graph.size() << endl;
    cout << "Fringe: " << max_fringe_size << endl;
}

int main() {
    ifstream infile("largegraph.txt");
    if (!infile) {
        cerr << "Error opening file." << endl;
        return 1;
    }

    int start = 0; // Start node
    int goal = 1696406; // Goal node

    vector<vector<Edge>> graph(1696415); 

    string line;
    getline(infile, line);

    while (getline(infile, line)) {
        istringstream iss(line);
        int fromNodeId, toNodeId, weight;
        if (!(iss >> fromNodeId >> toNodeId >> weight)) {
            cerr << "Error parsing line: " << line << endl;
            continue;
        }
        graph[fromNodeId].push_back({toNodeId, weight});
    }

    dijkstra(graph, start, goal);

    return 0;
}
