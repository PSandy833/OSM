/*main.cpp*/

//
// Program to input Nodes (positions), Buildings and Footways
// from an Open Street Map file.
// 
// Prof. Joe Hummel
// Northwestern University
// CS 211: Winter 2023
// 

#include <iostream>
#include <cmath>
#include <stack>
#include <string>
#include <queue>
#include <utility>
#include <vector>
#include <iomanip>
#include <limits>
#include <fstream>
#include <map>
#include <functional>
#include <set>

#include "graph.h"
#include "building.h"
#include "buildings.h"
#include "footway.h"
#include "footways.h"
#include "node.h"
#include "nodes.h"
#include "osm.h"
#include "tinyxml2.h"
#include "graph.h"
#include "dist.h"

using namespace std;
using namespace tinyxml2;



constexpr  double INF = numeric_limits<double>::max();
//
// Dijkstra:
//
// Performs Dijkstra's shortest weighted path algorithm from
// the given start vertex.  Returns a vector of vertices in
// the order they were visited, along with a map of (long long,int)
// pairs where the long long is a vertex V and the int is the 
// distance from the start vertex to V; if no such path exists,
// the distance is INF (defined in util.h).
//
class prioritize {
  public:
    bool operator()(const pair<long long, double> &p1, const pair<long long, double> &p2) const {
      if (p1.second > p2.second)
        return true;
      else if (p1.second < p2.second)
        return false;
      else
        return p1.first > p2.first;
  }
};
void Dijkstra(graph& G, long long startV, long long endV, map<long long, double>& distances) {
   priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> pq;
   map<long long, bool> visited;
   map<long long, long long> prev;
   vector<long long> orderVisited; // keep track
   set<long long> S;
   for (long long &v : G.getVertices()) {
      distances.emplace(v, INF);
      prev.emplace(v, (long long)0);
      pq.push(make_pair(v, INF));
   }

   // Start at the given vertex
   distances[startV] = (double)0;
   prev[startV] = startV;
   pq.push(make_pair(startV, (double)0));
  //  cout << distances[startV] << endl;

   while (!pq.empty()) {
      // Get next unvisited vertex with the smallest distance
      long long curr = pq.top().first;
      pq.pop();
      // cout << "Start loop " << distances[curr] << endl;

      if (distances[curr] == INF)
        break;
      else if (S.find(curr) != S.end())
        continue;
      else {
        orderVisited.push_back(curr);
        S.emplace(curr);
      }

      // if (visited[curr]) {
      //    continue;
      // }
      // visited[curr] = true;

      // Add to list of visited vertices
      // orderVisited.push_back(curr);

      // Check all vertices
      // auto neighbors = G.neighbors(curr);
      for (long long v : G.neighbors(curr)) {
        //  if (!visited.count(v)) {
        double weight;
        bool in = G.getWeight(curr, v, weight);
        double dist = distances[curr] + weight;
        if (dist < distances[v]) {
          // Update distance to neighbor
          distances[v] = dist;
          // Set predecessor for shortest path
          prev[v] = curr;
          // Add neighbor to priority queue
          pq.push(make_pair(v, dist));
        }
        //  }
      }
   }
  //  for (auto p : distances) {
  //   cout << p.second << endl;
  //  }



   cout << "Shortest weighted path:" << endl;
  if(distances[endV] != INF){
    cout << "  # nodes visited: " << orderVisited.size() << endl;
    cout << "  Distance: " << distances[endV] << " miles" << endl;
    cout << "  Path: ";
    stack<long long> path;
    long long curr = endV;
    while(curr != startV){
      path.push(curr);
      curr = prev[curr];
    }
    // path.push(startV);
    cout << startV;
    while(!path.empty()){
      long long dap = path.top();
      path.pop();
      cout << "->" << dap; 
    }
    // long long dap = path.top();
    // path.pop();
    // cout << dap << endl;

  }
  else{
    cout << "**Sorry, destination unreachable" << endl;
  }

  

  //  return orderVisited;
}

// 
// addToGraph
//
void addToGraph(graph& G, Nodes& nodes, Footways& footways){
  for(Footway& f : footways.MapFootways){
      for(int i = 0; i < (int)f.NodeIDs.size() - 1; i++){
        double lat1;
        double lat2;
        double long1;
        double long2;
        bool isEntrance;
        long long start = f.NodeIDs[i];
        long long end = f.NodeIDs[i + 1];
        nodes.find(start, lat1, long1, isEntrance);
        nodes.find(end , lat2, long2, isEntrance);
        double dist1 = distBetween2Points(lat1, long1, lat2 , long2);
        double dist2 = distBetween2Points(lat2, long2, lat1 , long1);

        G.addEdge(start, end, dist1);
        G.addEdge(end, start, dist2);
      }
    }
}
//
// printPath
//
// void printPath(graph& G, long long& from , long long& to){
  
// }
//
// sanityCheck
//
void sanityCheck(graph& G, Footways& footways, long long idenNum){
  Footway footway((long long)0);
  for(Footway& f : footways.MapFootways){
    if(f.ID == idenNum){
      footway = f;
      break;
    }
  }
  cout << "Graph check of Footway ID "<< idenNum << endl;
  for(int ii = 0 ; ii < (int)footway.NodeIDs.size()-1; ii++){
    long long start = footway.NodeIDs[ii];
    long long end = footway.NodeIDs[ii + 1];
    double weight1, weight2;
    G.getWeight(start, end, weight1);
    G.getWeight(end, start, weight2);
    cout << "  Edge: ("<< start <<", "<< end << ", "<< weight1 << ")" << endl;
    cout << "  Edge: ("<< end <<", "<< start << ", "<< weight2 << ")" << endl;
  }
  
}
// Populate Graph function 
//
void populateGraph(graph& G, Nodes& nodes, Footways& footways){
    for(auto& pair : nodes){
        G.addVertex(pair.first);  
    }
    addToGraph(G, nodes, footways);
    
}
//
// insertB
//
void insertB(Buildings& buildings, string& name, Building& wantedB ){
    for(Building& B : buildings.MapBuildings){
      if(B.Name.find(name) != string::npos){
        wantedB = B;
        break;
      }
    }
}
//
//printNameLoc
//
// void printNameLoc(Building& B, Nodes& nodes){
//   cout << "  Name: "<< B.Name << endl;
//   cout << "  Approximate location: (" << B.getLocation(nodes).first << 
//   ", "<< B.getLocation(nodes).second << ")" << endl;
// }
//
// printClosestFootway
//
long long printClosestFootway(Building& B, Footways& footways, Nodes& nodes){
  long long footwayId, nodeId;
  double dist = INF;
  for(Footway& f : footways.MapFootways){
    for(long long& id : f.NodeIDs){
      double lat, lon;
      bool isEntrance;
      bool found = nodes.find(id, lat, lon, isEntrance);
      if(found){
        auto loc = B.getLocation(nodes);
        double testDist = distBetween2Points(loc.first, loc.second, lat, lon);
        if(testDist < dist){
          footwayId = f.ID;
          nodeId = id;
          dist = testDist;
        }
        
        
      }
    }
  }
  cout << "  Closest footway ID "<< footwayId << ", node ID " << nodeId << ", distance "<< dist << endl;
  return nodeId;
}
// Navigate function
//
void navigate(graph& G, Buildings& buildings, Nodes& nodes, Footways& footways){
  string start, end;
  long long FS , FD;
  Building startB((long long)0, " "," ");
  Building endB((long long)0, " "," ");
  cout << "Enter start building name (partial or complete)> "<< endl;
  getline(cin, start);
  insertB(buildings, start, startB);
  if(startB.ID == (long long)0){
    cout << "**Start building not found" << endl;
    return;
  }
  else{
    cout << "  Name: "<< startB.Name << endl;
  cout << "  Approximate location: (" << startB.getLocation(nodes).first << 
  ", "<< startB.getLocation(nodes).second << ")" << endl;
   FS = printClosestFootway(startB, footways, nodes);
    
    
  }
  cout << "Enter destination building name (partial or complete)> " << endl;
  getline(cin, end);
  insertB(buildings, end, endB);
  if(endB.ID ==(long long)0){
    cout << "**Destination building not found" << endl;
    return;
  }
  else{
    cout << "  Name: "<< endB.Name << endl;
  cout << "  Approximate location: (" << endB.getLocation(nodes).first << 
  ", "<< endB.getLocation(nodes).second << ")" << endl;
    FD = printClosestFootway(endB, footways, nodes);
  }
  map<long long, double> distances;
  Dijkstra(G, FS , FD, distances);
}
// main
//
int main()
{
  XMLDocument xmldoc;
  Nodes nodes;
  Buildings buildings;
  Footways footways;

  
  cout << setprecision(12);
  cout << "** NU open street map **" << endl;

  string filename;

  cout << endl;
  cout << "Enter map filename> " << endl;
  getline(cin, filename);

  //
  // 1. load XML-based map file 
  //
  if (!osmLoadMapFile(filename, xmldoc))
  {
    // failed, error message already output
    return 0;
  }
  
  //
  // 2. read the nodes, which are the various known positions on the map:
  //
  nodes.readMapNodes(xmldoc);


  //
  // NOTE: let's sort so we can use binary search when we need 
  // to lookup nodes.
  //
  nodes.sortByID();

  //
  // 3. read the university buildings:
  //
  buildings.readMapBuildings(xmldoc);

  //
  // 4. read the footways, which are the walking paths:
  //
  footways.readMapFootways(xmldoc);

  // 5. Graph
  graph G;
  populateGraph(G , nodes, footways);
  // 6. stats
  //
  cout << "# of nodes: " << nodes.getNumMapNodes() << endl;
  cout << "# of buildings: " << buildings.getNumMapBuildings() << endl;
  cout << "# of footways: " << footways.getNumMapFootways() << endl;
  cout << "# of graph vertices: "<< G.NumVertices() << endl;
  cout << "# of graph edges: " << G.NumEdges() << endl;
  
  //
  // now let the user for search for 1 or more buildings:
  //
  while (true)
  {
    string name;

    cout << endl;
    cout << "Enter building name, * to list, @ to navigate, or $ to end> " << endl;

    getline(cin, name);

    if (name == "$") {
      break;
    }
    else if(name == "!"){
      sanityCheck(G, footways, (long long)986532630);

    }
    else if(name == "@"){
      navigate(G, buildings, nodes, footways);
    }
    else if (name == "*") {
      buildings.print();
    }
    else {
      buildings.findAndPrint(name, nodes, footways);
    }

  }//while

  //
  // done:
  //
  cout << endl;
  cout << "** Done  **" << endl;
  cout << "# of calls to getID(): " << Node::getCallsToGetID() << endl;
  cout << "# of Nodes created: " << Node::getCreated() << endl;
  cout << "# of Nodes copied: " << Node::getCopied() << endl;
  cout << endl;

  return 0;
}
