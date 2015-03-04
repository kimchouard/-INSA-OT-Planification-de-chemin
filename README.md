# [INSA] OT Robots - Compte Rendu
## *Planification-de-chemin*


Compte rendu pour le TP de planification de chemin dans le cadre de l'OT Robots de l'INSA de Lyon.

### Question 2/3

```c++
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

Enfin, ajouter ces quelques lignes dans le main afin d'exécuter le code:

```c++
auto parents = breadth_first_search(example_graph, 'A', 'E');
example_graph.draw(parents, 'A', 'E');
```
