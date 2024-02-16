#include "graph.h"

Graph::Graph(cspace::state_t x0) {
    add_vertex(x0);
}


void Graph::add_vertex(const cspace::state_t& x) {
    vertices.emplace_back(x);
    vertex_map[x] = &vertices.back();
}

void Graph::add_edge(const cspace::state_t& x1, const cspace::state_t& x2, const cspace::input_trajectory_t& u) {
    Vertex* v1 = vertex_map.at(x1);
    Vertex* v2 = vertex_map.at(x2);
    edges.emplace_back(v1, v2, u);
    v1->edges_out.push_back(&edges.back());
    v2->edge_in = &edges.back();
}

std::list<cspace::input_t> Graph::get_input(const cspace::state_t& x) {
    std::list<cspace::input_t> path;
    Vertex* v = vertex_map.at(x);

    // While not origin vertex
    while(v->edge_in != nullptr){
        const cspace::input_trajectory_t& u = v->edge_in->input;
        path.insert(path.end(), u.begin(), u.end());
        v = v->edge_in->source;
    }
    return path;
}
