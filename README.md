# [INSA] OT Robots - Compte Rendu
## *Planification de chemin*


Compte rendu pour le TP de planification de chemin dans le cadre de l'OT Robots de l'INSA de Lyon.

### Question 2/3

```c++
template<typename Graph>
unordered_map<typename Graph::Location, typename Graph::Location>
breadth_first_search(Graph graph,
                     typename Graph::Location start,
                     typename Graph::Location goal)
{
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
```

### Question 4

Le code pour l'affichage des resultat pour le Graphe à directement été placé dans la structure Graph au travers de la fonction draw():

```c++
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
```

Il suffit ensuite de créer un petit main() et d'y exécuter:

```c++
int main( int argc, const char* argv[] )
{
    example_graph.draw();
}
```

Ce qui doit donner:

```
{
	E -> { B },
	D -> { E A },
	C -> { A },
	B -> { A C D },
	A -> { B },
}
```

### Question 5

De même que pour la question précédente, la fonction va être intégrée directement dans la structure Graph. Le nom ```draw()``` reste le même mais la signature change:

```c++
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
```

On voit que l'algorithme part du goal pour remonter noeud par noeud jusqu'à arriver au point de départ.

Enfin, ajouter ces quelques lignes dans le main afin d'exécuter le code:

```c++
auto parents = breadth_first_search(example_graph, 'A', 'E');
example_graph.draw(parents, 'A', 'E');
```

Ce qui doit donner:

```
End <- E <- D <- B <- A <- Begin
```

### Question 9

La principale différence ici intervient par l'ajout de la gestion des couts (cost_so_far...) et dans l'utilisation de la PriorityQueue.

```c++
template<typename Graph>
void dijkstra_search
(Graph graph,
 typename Graph::Location start,
 typename Graph::Location goal,
 unordered_map<typename Graph::Location, typename Graph::Location>& came_from,
 unordered_map<typename Graph::Location, int>& cost_so_far)
{
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
```

### Question 10

Assez simple car utilisant la même méthode que question 5.

```c++

template<typename Location>
vector<Location> reconstruct_path(
                                  Location start,
                                  Location goal,
                                  unordered_map<Location, Location>& came_from
                                  ) {
    vector<Location> path;
	

	Location newNode = came_from[goal];
	Location node;
	while (newNode != node) {
		node = newNode;
		path.push_back(node);
		newNode = came_from[node];
	}

    return path;
}
```

Ce qui peut simplement être testé en exécutant ```test_dijkstra_search()``` dans le main.

### Question 11

L'heuristique donne une évaluation positive quand à le coup restant avant d'arriver au but. Plusieurs solutions sont possible. Étant dans une grille 2d, j'ai choisi la **distance euclidienne**.

```c++
inline double heuristic(SquareGrid::Location a, SquareGrid::Location b) {
    int x = abs(std::get<0>(a)-std::get<0>(b));
    int y = abs(std::get<1>(b)-std::get<1>(b));
    return sqrt(x*x+y+y);
}
```

### Question 12

```c++
template<typename Graph>
void a_star_search
(Graph graph,
 typename Graph::Location start,
 typename Graph::Location goal,
 unordered_map<typename Graph::Location, typename Graph::Location>& came_from,
 unordered_map<typename Graph::Location, int>& cost_so_far)
{
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
```

Ce qui peut simplement être testé en exécutant ```test_a_star_search()``` dans le main.

### Question 13

L'heuristique sert à donner une évaluation grossière sur le coût nécessaire pour arriver à destination, et ce bien sûr, en fonction du point demandé. Cela pert tout son sens dans un cas particulier de la fonction heuristique. Mettons que l'on choisisse une **fonction heuristique constante**, elle ne permettra plus de faire la différence entre les points. Les algorithmes de Dijkstra et A* deviennent alors équivalent.

### Question 14

