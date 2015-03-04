//
//  code_incomplet.cpp
//  
//
//  Created by Jilles Dibangoye on 04/03/2015.
//
//


/*
 */

#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>

using std::unordered_map;
using std::unordered_set;
using std::array;
using std::vector;
using std::queue;
using std::priority_queue;
using std::pair;
using std::tuple;
using std::tie;
using std::string;
using std::cout;
using std::endl;

template<typename L>
struct Graph {
    typedef L Location;
    typedef typename vector<Location>::iterator iterator;
    unordered_map<Location, vector<Location> > edges;
    
    inline const vector<Location>  neighbors(Location id) {
        return edges[id];
    }

    //+-----------------------------+//
    //      Question 4, Part 1       //
    //+-----------------------------+//

    void draw() {
        cout<<"{"<<endl;
        
        pair<Location, vector<Location> > node;
        vector<Location> nodeEdges;
        for (auto node: edges) {
            cout<<"\t"<<node.first<<" -> { ";

            for (auto nodeEdges: node.second)
                cout<<nodeEdges<<" ";

            cout<<"},"<<endl;
        }

        cout<<"}"<<endl;
    }

    //+-----------------------------+//
    //      Question 5, Part 1       //
    //+-----------------------------+//

    void draw(unordered_map<Location, Location> itinary, Location start, Location goal) {
        cout<<"End <- "<<goal<<" <- ";
        Location newNode = itinary[goal];
        Location node;
        while (newNode != node) {
            node = newNode;
            cout<<node<<" <- ";
            newNode = itinary[node];
        }
        cout<<"Begin"<<endl;
    }
};

Graph<char> example_graph {{
    {'A', {'B'}},
    {'B', {'A', 'C', 'D'}},
    {'C', {'A'}},
    {'D', {'E', 'A'}},
    {'E', {'B'}}
}};

// Helpers for SquareGrid::Location

namespace std {
    // I know, this is technically not allowed
    template <>
    struct hash<tuple<int,int> > {
        inline size_t operator()(const tuple<int,int>& location) const {
            int x, y;
            tie (x, y) = location;
            return x * 1812433253 + y;
        }
    };
}

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, tuple<int,int> loc) {
    int x, y;
    tie (x, y) = loc;
    out << '(' << x << ',' << y << ')';
    return out;
}


template<class Graph>
void draw_grid(const Graph& graph, int field_width,
               unordered_map<typename Graph::Location, int>* distances=nullptr,
               unordered_map<typename Graph::Location, typename Graph::Location>* point_to=nullptr,
               vector<typename Graph::Location>* path=nullptr) {
    for (int y = 0; y != graph.height; ++y) {
        for (int x = 0; x != graph.width; ++x) {
            typename Graph::Location id {x, y};
            std::cout << std::left << std::setw(field_width);
            if (graph.walls.count(id)) {
                std::cout << string(field_width, '#');
            } else if (point_to != nullptr && point_to->count(id)) {
                int x2, y2;
                tie (x2, y2) = (*point_to)[id];
                // TODO: how do I get setw to work with utf8?
                if (x2 == x + 1) { std::cout << "\u2192 "; }
                else if (x2 == x - 1) { std::cout << "\u2190 "; }
                else if (y2 == y + 1) { std::cout << "\u2193 "; }
                else if (y2 == y - 1) { std::cout << "\u2191 "; }
                else { std::cout << "* "; }
            } else if (distances != nullptr && distances->count(id)) {
                std::cout << (*distances)[id];
            } else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) {
                std::cout << '@';
            } else {
                std::cout << '.';
            }
        }
        std::cout << std::endl;
    }
}

struct SquareGrid {
    typedef tuple<int,int> Location;
    static array<Location, 4> DIRS;
    
    int width, height;
    unordered_set<Location> walls;
    
    SquareGrid(int width_, int height_)
    : width(width_), height(height_) {}
    
    inline bool in_bounds(Location id) {
        int x, y;
        tie (x, y) = id;
        return 0 <= x && x < width && 0 <= y && y < height;
    }
    
    inline bool passable(Location id) {
        return !walls.count(id);
    }
    
    vector<Location> neighbors(Location id) {
        int x, y, dx, dy;
        tie (x, y) = id;
        vector<Location> results;
        
        for (auto dir : DIRS) {
            tie (dx, dy) = dir;
            Location next(x + dx, y + dy);
            if (in_bounds(next) && passable(next)) {
                results.push_back(next);
            }
        }
        
        if ((x + y) % 2 == 0) {
            // aesthetic improvement on square grids
            std::reverse(results.begin(), results.end());
        }
        
        return results;
    }
};


array<SquareGrid::Location, 4> SquareGrid::DIRS {Location{1, 0}, Location{0, -1}, Location{-1, 0}, Location{0, 1}};

void add_rect(SquareGrid& grid, int x1, int y1, int x2, int y2) {
    for (int x = x1; x < x2; ++x) {
        for (int y = y1; y < y2; ++y) {
            grid.walls.insert(SquareGrid::Location { x, y });
        }
    }
}

SquareGrid make_diagram1() {
    SquareGrid grid(30, 15);
    add_rect(grid, 3, 3, 5, 12);
    add_rect(grid, 13, 4, 15, 15);
    add_rect(grid, 21, 0, 23, 7);
    add_rect(grid, 23, 5, 26, 7);
    return grid;
}

struct GridWithWeights: SquareGrid {
    unordered_set<Location> forests;
    GridWithWeights(int w, int h): SquareGrid(w, h) {}
    int cost(Location a, Location b) {
        return forests.count(b) ? 5 : 1;
    }
};


GridWithWeights make_diagram4() {
    GridWithWeights grid(10, 10);
    add_rect(grid, 1, 7, 4, 9);
    typedef SquareGrid::Location L;
    grid.forests = unordered_set<SquareGrid::Location> {
        L{3, 4}, L{3, 5}, L{4, 1}, L{4, 2},
        L{4, 3}, L{4, 4}, L{4, 5}, L{4, 6},
        L{4, 7}, L{4, 8}, L{5, 1}, L{5, 2},
        L{5, 3}, L{5, 4}, L{5, 5}, L{5, 6},
        L{5, 7}, L{5, 8}, L{6, 2}, L{6, 3},
        L{6, 4}, L{6, 5}, L{6, 6}, L{6, 7},
        L{7, 3}, L{7, 4}, L{7, 5}
    };
    return grid;
}

template<typename T, typename Number=int>
struct PriorityQueue {
    typedef pair<Number, T> PQElement;
    priority_queue<PQElement, vector<PQElement>,
    std::greater<PQElement>> elements;
    
    inline bool empty() { return elements.empty(); }
    
    inline void put(T item, Number priority) {
        elements.emplace(priority, item);
    }
    
    inline T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};


template<typename Graph>
unordered_map<typename Graph::Location, typename Graph::Location>
breadth_first_search(Graph graph,
                     typename Graph::Location start,
                     typename Graph::Location goal)
{


    //+-----------------------------+//
    //          Question 3           //
    //+-----------------------------+//

    typedef typename Graph::Location Location;

    // Init vars
    queue<Location> frontier;
    unordered_map <Location, Location> came_from;
    Location current;
    Location neighbor;
    vector<Location> currentNeighbors;

    // Set vars
    frontier.push(start);
    came_from[start] = start;

    while (!frontier.empty()) {
        current = frontier.front();
        frontier.pop();

        if (current == goal)
            break;
        
        currentNeighbors = graph.neighbors(current);

        for(auto neighbor : currentNeighbors) {
            if (!came_from.count(neighbor)) {
                frontier.push(neighbor);
                came_from[neighbor] = current;
            }
        }
    }

    return came_from;
}

int test_breadth_first_search() {
    SquareGrid grid = make_diagram1();
    auto parents = breadth_first_search(grid, SquareGrid::Location{8, 7}, SquareGrid::Location{17, 2});
    draw_grid(grid, 2, nullptr, &parents);
}


template<typename Graph>
void dijkstra_search
(Graph graph,
 typename Graph::Location start,
 typename Graph::Location goal,
 unordered_map<typename Graph::Location, typename Graph::Location>& came_from,
 unordered_map<typename Graph::Location, int>& cost_so_far)
{

    //+-----------------------------+//
    //          Question 9           //
    //+-----------------------------+//

    typedef typename Graph::Location Location;

    // Init vars
    PriorityQueue<Location> frontier;
    Location current;
    Location neighbor;
    int nCost;
    vector<Location> currentNeighbors;

    // Set vars
    frontier.put(start, 0);
    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        current = frontier.get();

        if (current == goal)
            break;
        
        currentNeighbors = graph.neighbors(current);

        for(auto neighbor : currentNeighbors) {
            nCost = graph.cost(current,neighbor) + cost_so_far[current];

            if ((!came_from.count(neighbor)) || (nCost < cost_so_far[neighbor])) {
                frontier.put(neighbor, nCost);
                cost_so_far[neighbor] = nCost;
                came_from[neighbor] = current;
            }
        }
    }
}


template<typename Location>
vector<Location> reconstruct_path(
                                  Location start,
                                  Location goal,
                                  unordered_map<Location, Location>& came_from
                                  ) {
    vector<Location> path;


    //+-----------------------------+//
    //          Question 10          //
    //+-----------------------------+//
    
    cout<<"End <- "<<goal<<" <- ";

    Location newNode = came_from[goal];
    Location node;
    while (newNode != node) {
        node = newNode;
        path.push_back(node);
        cout<<node<<" <- ";
        newNode = came_from[node];
    }

    cout<<"Begin"<<endl;

    return path;
}


int test_dijkstra_search() {
    GridWithWeights grid = make_diagram4();
    SquareGrid::Location start{1, 4};
    SquareGrid::Location goal{8, 5};
    unordered_map<SquareGrid::Location, SquareGrid::Location> came_from;
    unordered_map<SquareGrid::Location, int> cost_so_far;
    dijkstra_search(grid, start, goal, came_from, cost_so_far);
    draw_grid(grid, 2, nullptr, &came_from);
    std::cout << std::endl;
    draw_grid(grid, 3, &cost_so_far, nullptr);
    std::cout << std::endl;
    vector<SquareGrid::Location> path = reconstruct_path(start, goal, came_from);
    draw_grid(grid, 3, nullptr, nullptr, &path);
}

inline double heuristic(SquareGrid::Location a, SquareGrid::Location b) {

    //+-----------------------------+//
    //          Question 11          //
    //+-----------------------------+//

    int x = abs(std::get<0>(a)-std::get<0>(b));
    int y = abs(std::get<1>(b)-std::get<1>(b));
    return sqrt(x*x+y+y);
}

template<typename Graph>
void a_star_search
(Graph graph,
 typename Graph::Location start,
 typename Graph::Location goal,
 unordered_map<typename Graph::Location, typename Graph::Location>& came_from,
 unordered_map<typename Graph::Location, int>& cost_so_far)
{

    //+-----------------------------+//
    //          Question 12          //
    //+-----------------------------+//

    typedef typename Graph::Location Location;

    // Init vars
    PriorityQueue<Location> frontier;
    Location current;
    Location neighbor;
    int nCost;
    vector<Location> currentNeighbors;

    // Set vars
    frontier.put(start, 0);
    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        current = frontier.get();

        if (current == goal)
            break;
        
        currentNeighbors = graph.neighbors(current);

        for(auto neighbor : currentNeighbors) {
            nCost = graph.cost(current,neighbor) + cost_so_far[current];

            if ((!came_from.count(neighbor)) || (nCost < cost_so_far[neighbor])) {
                frontier.put(neighbor, nCost+heuristic(neighbor,goal));
                cost_so_far[neighbor] = nCost;
                came_from[neighbor] = current;
            }
        }
    }
}

int test_a_star_search() {
    GridWithWeights grid = make_diagram4();
    SquareGrid::Location start{1, 4};
    SquareGrid::Location goal{8, 5};
    unordered_map<SquareGrid::Location, SquareGrid::Location> came_from;
    unordered_map<SquareGrid::Location, int> cost_so_far;
    a_star_search(grid, start, goal, came_from, cost_so_far);
    draw_grid(grid, 2, nullptr, &came_from);
    std::cout << std::endl;
    draw_grid(grid, 3, &cost_so_far, nullptr);
    std::cout << std::endl;
    vector<SquareGrid::Location> path = reconstruct_path(start, goal, came_from);
    draw_grid(grid, 3, nullptr, nullptr, &path);
}



int main( int argc, const char* argv[] )
{
    //+-----------------------------+//
    //      Question 5, Part 2       //
    //+-----------------------------+//
    // example_graph.draw();

    //+-----------------------------+//
    //      Question 5, Part 2       //
    //+-----------------------------+//
    // auto parents = breadth_first_search(example_graph, 'A', 'E');
    // example_graph.draw(parents, 'A', 'E');

    // TEST 1
    // test_breadth_first_search();

    // TEST 2
    // test_dijkstra_search();

    // TEST 3
    test_a_star_search();
}
