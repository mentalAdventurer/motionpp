#ifndef GRAPH_H
#define GRAPH_H
#include "cspace.h"
#include <cwchar>
#include <vector>
#include <unordered_map>
#include <list>

// Hash function for vector<double>
namespace std {
    template<> struct hash<vector<double>> {
        size_t operator()(const vector<double>& vec) const {
            size_t seed = vec.size();
            for(auto& i : vec) {
                // Combine the hash of the current element with the running hash
                seed ^= hash<double>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
}

class Graph{
    public:
        struct Vertex;
        struct Edge;

    private:
        std::unordered_map<std::shared_ptr<const cspace::state_t>, Vertex*> vertex_map;
        bool success_flag;
        std::list<Vertex> vertices;
        std::list<Edge> edges;

    public:
        Graph(std::shared_ptr<const cspace::state_t> x0_ptr);
        void add_vertex(std::shared_ptr<const cspace::state_t> x);
        void add_edge(std::shared_ptr<const cspace::state_t> x1, std::shared_ptr<const cspace::state_t> x2, std::shared_ptr<const cspace::input_trajectory_t> u);
        cspace::trajectory_t get_input(std::shared_ptr<const cspace::state_t> x);
        auto begin()->decltype(vertices.begin());
        auto end()->decltype(vertices.end());
        auto cbegin()->decltype(vertices.cbegin());
        auto cend()->decltype(vertices.cend());

    public:
        bool get_success();
        void set_success(bool flag);
        std::size_t size_vertices();
        
};

struct Graph::Vertex {
    std::shared_ptr<const cspace::state_t> state;
    std::list<Edge*> edges_out;
    Edge* edge_in = nullptr;
    Vertex(std::shared_ptr<const cspace::state_t> x) : state(x) {}
};

struct Graph::Edge {
    Vertex* source;
    Vertex* target;
    std::shared_ptr<const cspace::input_trajectory_t> input;
    Edge(Vertex* s, Vertex* t, std::shared_ptr<const cspace::input_trajectory_t> u) : source(s), target(t), input(u) {}
};

#endif // GRAPH_H
