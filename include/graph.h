#ifndef GRAPH_H
#define GRAPH_H
#include "cspace.h"
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

// Implement both in container
class Graph{
    public:
        struct Vertex;
        struct Edge;

    private:
        std::list<Vertex> vertices;
        std::list<Edge> edges;
        std::unordered_map<cspace::state_t, Vertex*> vertex_map;


    public:
        Graph(cspace::state_t x0);
        void add_vertex(const cspace::state_t& x);
        void add_edge(const cspace::state_t& x1, const cspace::state_t& x2, const cspace::input_trajectory_t& u);
        std::list<cspace::input_t> get_input(const cspace::state_t& x);
        
};

#endif // GRAPH_H
