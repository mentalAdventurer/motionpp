#include "cell_based_search.h"
#include "cspace.h"
#include <vector>


class Graph{
    public:
        Graph(cspace::state_t x0);
        void add_vertex(cspace::state_t x);
        void add_edge(cspace::input_trajectory_t u);
    private:
        std::vector<cspace::state_t> vertices;
        std::vector<cspace::input_trajectory_t> edges;
};

class Queue{
    public:
        Queue(cspace::state_t x0);
        void push(cspace::state_t x);
        cspace::state_t pop();
        bool empty();
    private:
        std::vector<cspace::state_t> queue;
};


Graph cell_based_sarch(cspace::state_t x0,cspace::state_t xg) {
    std::size_t state_dim = x0.size();
    Graph G(x0);
    Queue Q(x0);
    cspace::Voronoi P(10000,x0,xg);

    while(!Q.empty() && !P.target_reached()){
        cspace::state_t x_cur = Q.pop();
        int time_steps = 3;
        cspace::ReachedSet R(x_cur,time_steps,0.1);
        while(!R.empty()){
            if(P.visit(R.front())){
                cspace::state_t x = R.pop_state(); 
                cspace::input_trajectory_t u = R.pop_input();
                Q.push(x);
                G.add_edge(u);
                G.add_vertex(x);
            }

        }
            

    }
    return G;
}

