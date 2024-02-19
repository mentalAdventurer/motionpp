#include "graph.h"

Graph::Graph(std::shared_ptr<const cspace::state_t> x0_ptr) {
    add_vertex(x0_ptr);
}


void Graph::add_vertex(std::shared_ptr<const cspace::state_t> x) {
    vertices.emplace_back(x);
    vertex_map[x] = &vertices.back();
}

void Graph::add_edge(std::shared_ptr<const cspace::state_t> x1, std::shared_ptr<const cspace::state_t> x2, std::shared_ptr<const cspace::input_trajectory_t> u) {
    Vertex* v1 = vertex_map.at(x1);
    Vertex* v2 = vertex_map.at(x2);
    edges.emplace_back(v1, v2, u);
    v1->edges_out.push_back(&edges.back());
    v2->edge_in = &edges.back();
}

std::list<cspace::input_t> Graph::get_input(std::shared_ptr<const cspace::state_t> x) {
    std::list<cspace::input_t> path;
    Vertex* v = vertex_map.at(x);

    // While not origin vertex
    while(v->edge_in != nullptr){
        auto u_ptr = v->edge_in->input;
        path.insert(path.end(), u_ptr->begin(), u_ptr->end());
        v = v->edge_in->source;
    }
    return path;
}
