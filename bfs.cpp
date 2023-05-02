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
