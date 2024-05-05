#include <bits/stdc++.h>
#include <vector>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

using namespace std;

struct Edge {
    int to;
    int weight;
};

using Graph = vector<vector<Edge>>;

const int INF = numeric_limits<int>::max();

Graph generateGraph(int n, int m) {
    Graph graph(n);
    for (int i = 0; i < m; ++i) {
        int u = rand() % n;
        int v = rand() % n;
        int weight = rand() % 100 + 1;
        graph[u].push_back({v, weight});
        graph[v].push_back({u, weight}); 
    }
    return graph;
}

void dijkstra(const Graph& graph, int src, vector<int>& dist) {
    int n = graph.size();
    dist.assign(n, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, src});
    dist[src] = 0;

    while (!pq.empty()) {
        auto [cost, u] = pq.top(); pq.pop();
        if (cost > dist[u]) continue;
        for (auto& edge : graph[u]) {
            int v = edge.to, w = edge.weight;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }
}

int bidirectionalDijkstra(const Graph& graph, int src, int target) {
    int n = graph.size();
    vector<int> dist_s(n, INF), dist_t(n, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq_s, pq_t;
    unordered_set<int> processed_s, processed_t;

    pq_s.push({0, src});
    pq_t.push({0, target});
    dist_s[src] = 0;
    dist_t[target] = 0;

    while (!pq_s.empty() && !pq_t.empty()) {
        if (!pq_s.empty()) {
            auto [cost, u] = pq_s.top(); pq_s.pop();
            processed_s.insert(u);
            if (processed_t.find(u) != processed_t.end()) {
                return dist_s[u] + dist_t[u];
            }
            for (auto& edge : graph[u]) {
                int v = edge.to, w = edge.weight;
                if (dist_s[u] + w < dist_s[v]) {
                    dist_s[v] = dist_s[u] + w;
                    pq_s.push({dist_s[v], v});
                }
            }
        }

        if (!pq_t.empty()) {
            auto [cost, u] = pq_t.top(); pq_t.pop();
            processed_t.insert(u);
            if (processed_s.find(u) != processed_s.end()) {
                return dist_s[u] + dist_t[u];
            }
            for (auto& edge : graph[u]) {
                int v = edge.to, w = edge.weight;
                if (dist_t[u] + w < dist_t[v]) {
                    dist_t[v] = dist_t[u] + w;
                    pq_t.push({dist_t[v], v});
                }
            }
        }
    }

    return -1; 
}

void aStar(const Graph& graph, int src, int target, vector<int>& dist, const vector<int>& heuristic) {
    int n = graph.size();
    dist.assign(n, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0 + heuristic[src], src});
    dist[src] = 0;

    while (!pq.empty()) {
        auto [cost, u] = pq.top(); pq.pop();
        cost -= heuristic[u];
        if (u == target) break;
        if (cost > dist[u]) continue;
        for (auto& edge : graph[u]) {
            int v = edge.to, w = edge.weight;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v] + heuristic[v], v});
            }
        }
    }
}

template<typename Func>
double measureTime(Func func) {
    clock_t start = clock();
    func();
    return (double)(clock() - start) / CLOCKS_PER_SEC;
}

int main() {
    srand(time(nullptr)); 

    int n;
    int m;
    cout<<"Number of nodes"<<endl;
    cin>>n;
    cout<<"Number of edges"<<endl;
    cin>>m;
    Graph graph = generateGraph(n, m);
    
    int src = 0, target = n - 1;
    vector<int> dist, heuristic(n, 0); 

    double timeDijkstra = measureTime([&](){ dijkstra(graph, src, dist); });
    cout << "Dijkstra Execution Time: " << timeDijkstra << " seconds" << endl;

    double timeBiDijkstra = measureTime([&](){ return bidirectionalDijkstra(graph, src, target); });
    cout << "Bidirectional Dijkstra Execution Time: " << timeBiDijkstra << " seconds" << endl;

    double timeAStar = measureTime([&](){ aStar(graph, src, target, dist, heuristic); });
    cout << "A* Execution Time: " << timeAStar << " seconds" << endl;

    return 0;
}