#include <bits/stdc++.h>
using namespace std;

typedef pair<int, int> pii;
typedef unordered_map<int, vector<pii>> Graph;
typedef unordered_map<int, int> ParentMap;

void dijkstra(const Graph& graph, int source, ParentMap& parent, unordered_map<int, int>& dist) {
    priority_queue<pii, vector<pii>, greater<pii>> pq;
    pq.push({0, source});
    dist[source] = 0;

    while (!pq.empty()) {
        auto [current_dist, u] = pq.top(); pq.pop();

        if (current_dist > dist[u]) continue;

        for (auto& [v, weight] : graph.at(u)) {
            int new_dist = current_dist + weight;
            if (new_dist < dist[v]) {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }
}

vector<int> reconstruct_path(const ParentMap& parent, int source, int target) {
    vector<int> path;
    for (int at = target; at != source; at = parent.at(at)) {
        path.push_back(at);
    }
    path.push_back(source);
    reverse(path.begin(), path.end());
    return path;
}

pair<int, vector<int>> bidirectional_dijkstra(const Graph& graph, int source, int target) {
    ParentMap parent_forward, parent_backward;
    unordered_map<int, int> dist_forward, dist_backward;
    for (const auto& [key, _] : graph) {
        dist_forward[key] = INT_MAX;
        dist_backward[key] = INT_MAX;
    }

    dijkstra(graph, source, parent_forward, dist_forward);
    dijkstra(graph, target, parent_backward, dist_backward);

    int meeting_point = -1;
    int min_dist = INT_MAX;
    for (const auto& [node, _] : graph) {
        if (dist_forward[node] != INT_MAX && dist_backward[node] != INT_MAX) {
            int total_dist = dist_forward[node] + dist_backward[node];
            if (total_dist < min_dist) {
                min_dist = total_dist;
                meeting_point = node;
            }
        }
    }

    if (meeting_point == -1) {
        return {INT_MAX, {}};
    }

    vector<int> path_forward = reconstruct_path(parent_forward, source, meeting_point);
    vector<int> path_backward = reconstruct_path(parent_backward, meeting_point, target);
    path_backward.erase(path_backward.begin());  // Avoid repeating the meeting point

    vector<int> full_path = path_forward;
    full_path.insert(full_path.end(), path_backward.begin(), path_backward.end());

    return {min_dist, full_path};
}

int main() {
    Graph graph = {
        {0, {{1, 2}, {2, 4}}},
        {1, {{2, 3}, {3, 1}}},
        {2, {{3, 2}, {4, 2}}},
        {3, {{4, 5}}},
        {4, {}}
    };

    int source = 0, target = 4;

    auto start = chrono::high_resolution_clock::now();
    auto [distance, path] = bidirectional_dijkstra(graph, source, target);
    auto end = chrono::high_resolution_clock::now();

    cout << "Shortest path length: " << distance << endl;
    cout << "Path: ";
    for (int node : path) {
        cout << node << " ";
    }
    cout << endl;

    auto duration = chrono::duration_cast<chrono::seconds>(end - start).count();
    cout << "Execution time: " << duration << " seconds" << endl;

    return 0;
}