### In this article, we will take a look at graph traversal algorithm which will help you to traverse the graph in different ways

<br>

### Table of contents:


- [Introduction](#introduction)
  - [Graphs:](#graphs)
- [Depth-First Search (DFS):](#depth-first-search-dfs)
- [Breadth-First Search (BFS):](#breadth-first-search-bfs)
- [Complexity](#complexity)
- [Application](#application)
- [Conclusion](#conclusion)
  
<br>

# Introduction

**DFS** (Depth First Search) and **BFS** (Breadth First Search) are two of the most commonly used algorithms in computer science. These algorithms are used to traverse and search graph data structures. They are particularly useful in solving a wide range of problems in computer science, such as finding the shortest path between two nodes in a graph, detecting cycles in a graph, and solving the maze problems. In this article, we will discuss DFS and BFS in C++.

## Graphs:

Before diving into DFS and BFS, let's first discuss what a graph is. In computer science, a graph is a collection of nodes (also known as vertices) and edges (also known as arcs or links) that connect them. A graph can be directed or undirected, depending on whether the edges have a direction or not. Graphs can be represented using adjacency matrices, adjacency lists, or edge lists.

<br>

# Depth-First Search (DFS):

<br>

DFS is a graph traversal algorithm that explores as far as possible along each branch before backtracking. It follows the concept of exploring a vertex and all its adjacent vertices before backtracking to explore other vertices of the graph. DFS is often implemented using recursion or a stack data structure.


Here's an example of DFS implemented using recursion:

```c++
#include <iostream>
#include <vector>
#include <stack>
using namespace std;

// Graph class to represent the graph
class Graph {
private:
    int V;
    vector<int>* adj;
    void DFSUtil(int v, bool visited[]);
public:
    Graph(int V);
    void addEdge(int v, int w);
    void DFS(int v);
};

Graph::Graph(int V) {
    this->V = V;
    adj = new vector<int>[V];
}

void Graph::addEdge(int v, int w) {
    adj[v].push_back(w);
}

void Graph::DFSUtil(int v, bool visited[]) {
    visited[v] = true;
    cout << v << " ";

    for (auto i = adj[v].begin(); i != adj[v].end(); i++) {
        if (!visited[*i]) {
            DFSUtil(*i, visited);
        }
    }
}

void Graph::DFS(int v) {
    bool* visited = new bool[V];
    for (int i = 0; i < V; i++) {
        visited[i] = false;
    }

    DFSUtil(v, visited);
}

int main() {
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 0);
    g.addEdge(2, 3);
    g.addEdge(3, 3);

    cout << "DFS starting from vertex 2: ";
    g.DFS(2);
    cout << endl;

    return 0;
}

//output - DFS starting from vertex 2: 2 0 1 3

```

Here's how the DFS algorithm works:

1. Create a Graph class to represent the graph.
2. Add edges to the graph using the addEdge() function.
3. Create a DFSUtil() function that recursively visits all the vertices connected to the given vertex v.
4. Create a DFS() function that initializes a visited array and calls DFSUtil() for each unvisited vertex.
5. In the DFSUtil() function, mark the current vertex as visited and print it.
6. Traverse all the adjacent vertices of the current vertex using an iterator, and recursively call DFSUtil() for all unvisited vertices.

<br>

# Breadth-First Search (BFS):

<br>

BFS is a graph traversal algorithm that visits all the vertices of a graph in breadth-first order, i.e., it visits all the vertices at the same level before moving on to the next level. BFS uses a queue data structure to maintain the order of traversal.

Here's an example of BFS implemented using a queue:

```c++
#include <iostream>
#include <queue>
#include <vector>

using namespace std;

void bfs(vector<int> adj[], int start, int size)
{
    queue<int> q;
    vector<bool> visited(size, false);

    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        cout << curr << " ";

        for (int i = 0; i < adj[curr].size(); i++) {
            int next = adj[curr][i];
            if (!visited[next]) {
                visited[next] = true;
                q.push(next);
            }
        }
    }
}

int main()
{
    vector<int> adj[6];
    adj[1].push_back(2);
    adj[1].push_back(3);
    adj[2].push_back(4);
    adj[2].push_back(5);
    adj[3].push_back(5);
    adj[4].push_back(5);

    cout << "BFS traversal: ";
    bfs(adj, 1, 6);

    return 0;
}

//output - BFS traversal: 1 2 3 4 5 

```
Here, we have created a bfs() function that takes an adjacency list and the starting vertex as its parameters. Inside the function, we create a queue data structure to store the vertices and a vector to keep track of visited vertices. We mark the starting vertex as visited and enqueue it to the queue. Then, we loop until the queue is not empty. In each iteration, we dequeue a vertex from the queue, process it (in this case, print it), mark it as visited, and enqueue any unvisited neighbor vertices to the queue. Finally, we call the bfs() function in the main() function with the adjacency list, starting vertex and size of the graph as arguments.


<br>

# Complexity 

<br>

- Space Complexity: The space complexity of DFS and BFS is O(|V|), where |V| is the number of vertices in the graph. This is because in the worst case scenario, the entire graph needs to be stored in memory.

- Time Complexity: The time complexity of DFS and BFS is O(|V| + |E|), where |V| is the number of vertices and |E| is the number of edges in the graph. This is because both algorithms need to visit every vertex and edge in the graph.

<br>

# Application

<br>

DFS and BFS are both used in a variety of applications, including:

- Pathfinding algorithms in computer games and robotics
- Web crawlers and search engines
- Finding connected components and strongly connected components in a graph
- Detecting cycles in a graph
- Topological sorting of a directed acyclic graph
- Finding bridges and articulation points in a graph
- Network flow algorithms

> Which algorithm to choose: Whether to use DFS or BFS depends on the specific problem at hand. Generally speaking, DFS is more suitable for problems where the objective is to visit all the nodes in the graph (e.g. finding connected components), while BFS is more suitable for problems where the objective is to find the shortest path between two nodes (e.g. pathfinding).

<br>

# Conclusion

In this article, we have discussed two important graph traversal algorithms: DFS and BFS. DFS is a recursive algorithm that visits all the vertices of a graph in depth-first order while maintaining a stack. BFS is a non-recursive algorithm that visits all the vertices of a graph in breadth-first order while maintaining a queue. We have also provided C++ implementations of both algorithms using adjacency lists. By understanding these algorithms, you can efficiently traverse graphs and solve a variety of graph problems.