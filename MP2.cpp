#include <bits/stdc++.h>
using namespace std;
const int INF = numeric_limits<int>::max(); // Represents infinity for initial distances

class Node {
public:
    int index;
    int distance;

    Node(int idx, int dist) : index(idx), distance(dist) {}

    // Overloading Comparison operator for priority queue (min-heap)
    bool operator>(const Node& other) const {
        return distance > other.distance;
    }
};

// Dijkstra's algorithm function
vector<int> dijkstra(const vector<vector<pair<int, int>>>& graph, int start) {
    int n = graph.size();
    vector<int> dist(n, INF);
    priority_queue<Node, vector<Node>, greater<Node>> pq; // Min-heap priority queue

    dist[start] = 0; 
    pq.push(Node(start, 0)); // Push start node with distance 0

    while (!pq.empty()) {
        Node current = pq.top(); // Extract node with smallest distance
        pq.pop();

        int u = current.index;
        int currentDist = current.distance;

        if (currentDist > dist[u]) continue;

        // Explore neighbors of the current node
        for (const auto& neighbor : graph[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            // If a shorter path to v is found through u, update dist[v] and push v into the queue
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push(Node(v, dist[v]));
            }
        }
    }

    return dist; 
}

int main() {
    int n = 7; // Number of nodes (locations)
    vector<vector<pair<int, int>>> graph(n); // Adjacency list

    // Adding edges to the graph (undirected graph)
    graph[0].push_back({1, 7}); // Edge from node 0 to node 1 with weight 7
    graph[0].push_back({2, 9}); // Edge from node 0 to node 2 with weight 9
    graph[0].push_back({6, 14}); // Edge from node 0 to node 6 with weight 14
    graph[1].push_back({2, 10}); // Edge from node 1 to node 2 with weight 10
    graph[1].push_back({3, 15}); // Edge from node 1 to node 3 with weight 15
    graph[2].push_back({3, 12}); // Edge from node 2 to node 3 with weight 11
    graph[2].push_back({6, 5}); // Edge from node 2 to node 6 with weight 2
    graph[3].push_back({4, 7}); // Edge from node 3 to node 4 with weight 6
    graph[4].push_back({5, 9}); // Edge from node 4 to node 5 with weight 9
    graph[5].push_back({6, 9}); // Edge from node 5 to node 6 with weight 9

    int startNode = 0; // Starting node (source)
    int endNode = 5; // Ending node (destination)

    vector<int> distances = dijkstra(graph, startNode); 
    // Output 
    cout << "Shortest distance from node " << startNode << " to node " << endNode << ": " << distances[endNode] << endl;

    return 0;
}
