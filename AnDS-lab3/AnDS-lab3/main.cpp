#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <queue>
#include <limits>
#include <algorithm>

template<typename Vertex, typename Distance = double>
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance distance;

        Edge(const Vertex& f, const Vertex& t, const Distance& d)
            : from(f), to(t), distance(d) {}
    };

    // проверка-добавление-удаление вершин
    bool has_vertex(const Vertex& v) const {
        return _graph.find(v) != _graph.end();
    }

    void add_vertex(const Vertex& v) {
        _graph[v];
    }

    /////
    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v)) return false;
        _graph.erase(v);
        for (auto& [key, edges] : _graph) {
            edges.erase(std::remove_if(edges.begin(), edges.end(),
                [&v](const Edge& e) { return e.to == v; }),
                edges.end());
        }
        return true;
    }

    std::vector<Vertex> vertices() const {
        std::vector<Vertex> result;
        for (const auto& pair : _graph) {
            result.push_back(pair.first);
        }
        return result;
    }

    // проверка-добавление-удаление ребер
    void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
        _graph[from].emplace_back(from, to, d);
    }

    bool remove_edge(const Vertex& from, const Vertex& to) {
        if (!has_vertex(from)) return false;
        auto& edges = _graph[from];
        auto it = std::remove_if(edges.begin(), edges.end(),
            [&to](const Edge& e) { return e.to == to; });
        if (it != edges.end()) {
            edges.erase(it, edges.end());
            return true;
        }
        return false;
    }

    bool remove_edge(const Edge& e) {
        return remove_edge(e.from, e.to);
    }

    bool has_edge(const Vertex& from, const Vertex& to) const {
        if (!has_vertex(from)) return false;
        const auto& edges = _graph.at(from);
        return std::any_of(edges.begin(), edges.end(),
            [&to](const Edge& edge) { return edge.to == to; });
    }

    bool has_edge(const Edge& e) const {
        return has_edge(e.from, e.to);
    }

    std::vector<Edge> edges(const Vertex& vertex) {
        if (!has_vertex(vertex)) return {};
        return _graph[vertex];
    }
    /// 
    size_t order() const {
        return _graph.size();
    }

    size_t degree(const Vertex& v) const {
        if (!has_vertex(v)) return 0;
        return _graph.at(v).size();
    }

    // поиск кратчайшего пути (алгоритм Дейкстры)
    std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        std::unordered_map<Vertex, Distance> distances;
        std::unordered_map<Vertex, Vertex> previous;
        std::unordered_set<Vertex> visited;
        auto cmp = [&distances](const Vertex& left, const Vertex& right) {
            return distances[left] > distances[right];
            };
        std::priority_queue<Vertex, std::vector<Vertex>, decltype(cmp)> pq(cmp);

        for (const auto& pair : _graph) {
            distances[pair.first] = std::numeric_limits<Distance>::infinity();
        }
        distances[from] = 0;
        pq.push(from);

        while (!pq.empty()) {
            Vertex current = pq.top();
            pq.pop();

            if (visited.count(current)) continue;
            visited.insert(current);

            if (current == to) break;

            for (const auto& edge : _graph.at(current)) {
                if (visited.count(edge.to)) continue;

                Distance new_distance = distances[current] + edge.distance;
                if (new_distance < distances[edge.to]) {
                    distances[edge.to] = new_distance;
                    previous[edge.to] = current;
                    pq.push(edge.to);
                }
            }
        }

        std::vector<Edge> path;
        if (distances[to] == std::numeric_limits<Distance>::infinity()) {
            return path; // No path found
        }

        for (Vertex at = to; at != from; at = previous[at]) {
            Vertex from = previous[at];
            Distance dist = distances[at] - distances[from];
            path.emplace_back(from, at, dist);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // обход (DFS)
    std::vector<Vertex> walk(const Vertex& start_vertex) const {
        std::vector<Vertex> result;
        std::unordered_set<Vertex> visited;
        std::stack<Vertex> stack;
        stack.push(start_vertex);

        while (!stack.empty()) {
            Vertex current = stack.top();
            stack.pop();

            if (visited.count(current)) continue;
            visited.insert(current);
            result.push_back(current);

            // Добавление соседей в стек
            for (const auto& edge : _graph.at(current)) {
                if (!visited.count(edge.to)) {
                    stack.push(edge.to);
                }
            }
        }
        return result;
    }

private:
    std::unordered_map<Vertex, std::vector<Edge>> _graph;
};


int main() {
    Graph<std::string> g;

    // Добавление вершин
    g.add_vertex("A");
    g.add_vertex("B");
    g.add_vertex("C");
    g.add_vertex("D");

    // Добавление ребер
    g.add_edge("A", "B", 1.0);
    g.add_edge("A", "C", 4.0);
    g.add_edge("B", "C", 2.0);
    g.add_edge("B", "D", 5.0);
    g.add_edge("C", "D", 1.0);

    // Поиск кратчайшего пути
    auto path = g.shortest_path("A", "D");
    std::cout << "Shortest path from A to D:" << std::endl;
    for (const auto& edge : path) {
        std::cout << edge.from << " -> " << edge.to << " (distance: " << edge.distance << ")" << std::endl;
    }

    // Обход в глубину
    auto walk_result = g.walk("A");
    std::cout << "DFS walk starting from A:" << std::endl;
    for (const auto& vertex : walk_result) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;

    return 0;
}