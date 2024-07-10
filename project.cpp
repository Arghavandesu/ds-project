#include <iostream>
#include <queue>
#include <climits>
#include <stack>
#include <vector>

using namespace std;

// Structure for nodes
struct Node {
    int id;
    pair<int, int>* vertix;
};

// Function to check if the input is valid
bool isValidInput(int s, int d, int gs) {
    return (s >= 0 && s < gs && d >= 0 && d < gs); // created as a bool function
}

// Function to print path
void printPath(int* previous, int destination) {
    stack<int> pathStack;
    int residim = destination;

    // Push nodes onto stack in reverse order
    while (residim != -1) {
        pathStack.push(residim);
        residim = previous[residim];
    }

    // Print path from stack
    cout << "Best Path: ";
    while (!pathStack.empty()) {
        cout << pathStack.top();
        pathStack.pop();
        if (!pathStack.empty()) {
            cout << " -> ";
        }
    }
    cout << endl;
}

// Dijkstra's algorithm to find the shortest path
void dijkstra(Node* graph, int gs, int source, int destination) {
    int* distance = new int[gs];   // Array for distances
    int* previous = new int[gs];   // Array for previous nodes
    bool* visited = new bool[gs];  // Array to track visited nodes

    // Initialize arrays
    for (int i = 0; i < gs; ++i) {
        distance[i] = INT_MAX;
        previous[i] = -1;
        visited[i] = false;
    }

    // Priority queue to store {distance, node}
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    distance[source] = 0; // Distance to source is 0
    pq.push({0, source});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) continue; // Skip if already visited
        visited[u] = true;

        // Traverse neighbors of node u
        for (pair<int, int>* neighbor = graph[u].vertix; neighbor->first != -1; ++neighbor) {
            int v = neighbor->first;         // Neighbor node
            int weight = neighbor->second;   // Weight of the edge u -> v

            
            if (!visited[v] && distance[u] != INT_MAX && (distance[u] + weight < distance[v])) {
                distance[v] = distance[u] + weight;
                previous[v] = u;
                pq.push({distance[v], v});
            }
        }
    }

    cout << "Best path: ";
    printPath(previous, destination);

    // Clean arrays
    delete[] distance;
    delete[] previous;
    delete[] visited;
}

// BFS algorithm
void bfs(Node* graph, int gs, int source, int destination) {
    int* previous = new int[gs];   // Array for previous nodes
    bool* visited = new bool[gs];  // Array to track visited nodes

    // Initialize arrays
    for (int i = 0; i < gs; ++i) {
        previous[i] = -1;
        visited[i] = false;
    }

    queue<int> q;
    visited[source] = true;
    q.push(source);

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        // Traverse neighbors of node u
        for (pair<int, int>* neighbor = graph[u].vertix; neighbor->first != -1; ++neighbor) {
            int v = neighbor->first;         // Neighbor node

            if (!visited[v]) {
                visited[v] = true;
                previous[v] = u;
                q.push(v);
            }

            // Stop BFS if we reach the destination
            if (v == destination) {
                break;
            }
        }
    }

    cout << "possible paths: ";
    printPath(previous, destination);

    // Clean up dynamic arrays
    delete[] previous;
    delete[] visited;
}

int main() {
    int source, destination;

    Node graph[5]; // Graph with 5 nodes

    // Setting up edges and weights
    graph[0].vertix = new pair<int, int>[3]{{1, 3}, {2, 4}, {-1, 0}};
    graph[1].vertix = new pair<int, int>[3]{{2, 3}, {3, 7}, {-1, 0}};
    graph[2].vertix = new pair<int, int>[3]{{3, 5}, {4, 10}, {-1, 0}};
    graph[3].vertix = new pair<int, int>[3]{{4, 2}, {1, 6}, {-1, 0}};
    graph[4].vertix = new pair<int, int>[2]{{0, 3}, {-1, 0}};

    // Input and validation
    cout << "Enter your start and destination (between 0 and 4): ";
    cin >> source >> destination;

    if (!isValidInput(source, destination, 5)) {
        cout << "Invalid input! Please enter valid nodes." << endl;
        return 1; // Exit with error code
    }

    // Calculate and print shortest path using Dijkstra's algorithm
    dijkstra(graph, 5, source, destination);

    // Calculate and print shortest path using BFS
    bfs(graph, 5, source, destination);

    // Clean up dynamically allocated memory
    for (int i = 0; i < 5; ++i) {
        delete[] graph[i].vertix;
    }

    return 0;
}
