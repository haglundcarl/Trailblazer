#include "costs.h"
#include "trailblazer.h"
#include <queue>
#include <pqueue.h>

using namespace std;

/* Searches through the deepest path first,
 * then moves on to next path, until it finds the endNode
 */
vector<Node *> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end){
    vector<Vertex*> path;
    path.push_back(start);
    start->setColor(GREEN);
    start->visited = true;

    bool deadEnd = true;

    while(path.size() != 0){

        Vertex* currentNode = path.back();

        if(currentNode == end) return path;
        deadEnd = true;

        for(auto w : graph.getNeighbors(currentNode)){
            if(w->visited == false){
                w->setColor(GREEN);
                w->visited = true;
                path.push_back(w);
                deadEnd = false;
                break;
            }
        }
        if(deadEnd){
            currentNode->setColor(GRAY);
            path.pop_back();
        }
    }
    return path;
}

/* Searches through the neighbours and start-node first,
 * then moves on to neighbours neighbours, until it finds the end-node
 */
vector<Node*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    queue<Vertex* > neighbourQueue;
    neighbourQueue.push(start);


    while(!neighbourQueue.empty()){
        Vertex* currentNode;
        currentNode = neighbourQueue.front();
        neighbourQueue.pop();
        currentNode->visited = true;
        currentNode->setColor(GREEN);
        if(currentNode == end) break;

        for(auto neighbour : graph.getNeighbors(currentNode)){
            if(neighbour->visited == false){
                neighbour->visited = true;
                neighbour->setColor(YELLOW);
                neighbour->previous = currentNode;
                neighbourQueue.push(neighbour);
            }
        }
    }

    // return path taken
    vector<Node*> path;
    Node* newEnd = end;
    while(newEnd->previous != start){
        path.push_back(newEnd);
        newEnd = newEnd->previous;
    }
    path.push_back(newEnd);
    path.push_back(start);
    return path;
}

/* Works simliar to BFS but also takes cost into account
 * which makes the algorithm more suitable for terrain
 */
vector<Node *> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {

    for(auto v : graph.getVertexSet()){ // Loops through nodes in graph and sets every node-cost to inf
        v->cost = INFINITY;
    }

    start->cost = 0;
    PriorityQueue<Vertex* > pNodeQueue;
    pNodeQueue.enqueue(start, 0);

    while(!pNodeQueue.isEmpty()){ // while queue is not empty

        Vertex* currentNode;
        currentNode = pNodeQueue.dequeue();

        currentNode->visited = true;
        currentNode->setColor(GREEN);
        if(currentNode == end) break;

        for(auto neighbour : graph.getNeighbors(currentNode)){ // loops through all neighbours to current

            double tempCost;
            Edge* tempEdge = graph.getEdge(currentNode, neighbour); //edge between currentNode and neightbour
            tempCost = currentNode->cost + tempEdge->cost;

            if(tempCost < neighbour->cost){ // if we found a shorter path to the neighbour-node
                neighbour->setColor(YELLOW);
                neighbour->cost = tempCost; // update cost
                neighbour->previous = currentNode;

                if(neighbour->visited == true){
                    pNodeQueue.changePriority(neighbour, tempCost); // update node-cost in priorityQueue
                }
                else{
                    pNodeQueue.enqueue(neighbour, tempCost); // add node to pQueue
                }
             }
        }
    }

    vector<Node*> path;
    Node* newEnd = end;
    while(newEnd->previous != start){
        path.push_back(newEnd);
        newEnd = newEnd->previous;

    }
    path.push_back(newEnd);
    path.push_back(start);

    return path;
}

/* Works simliar to Djikstras but also takes heuristics into account
 * which makes the algorithm never to over-valuate the total cost of the path
 * the heuristic also changes the priority-queue which makes it the fastest algoritm of the four
 */
vector<Node *> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    for(auto v : graph.getVertexSet()){ // Loops through nodes in graph and sets every node-cost to inf
        v->cost = INFINITY;
    }

    start->cost = 0;
    PriorityQueue<Vertex* > pNodeQueue;
    pNodeQueue.enqueue(start, start->heuristic(end)); // add heuristic to A*

    while(!pNodeQueue.isEmpty()){

        Vertex* currentNode;
        currentNode = pNodeQueue.dequeue();

        currentNode->visited = true;
        currentNode->setColor(GREEN);
        if(currentNode == end) break;

        for(auto neighbour : graph.getNeighbors(currentNode)){

            double tempCost;
            Edge* tempEdge = graph.getEdge(currentNode, neighbour);
            tempCost = currentNode->cost + tempEdge->cost;

            if(tempCost < neighbour->cost){
                neighbour->setColor(YELLOW);
                neighbour->cost = tempCost;
                neighbour->previous = currentNode;

                if(neighbour->visited == true){
                    pNodeQueue.changePriority(neighbour, tempCost + neighbour->heuristic(end)); // add heuristic to A*
                }
                else{
                    pNodeQueue.enqueue(neighbour, tempCost + neighbour->heuristic(end)); // add heuristic to A*
                }
             }
        }
    }

    vector<Node*> path;
    Node* newEnd = end;
    while(newEnd->previous != start){
        path.push_back(newEnd);
        newEnd = newEnd->previous;

    }
    path.push_back(newEnd);
    path.push_back(start);

    return path;
}
