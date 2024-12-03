#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue>  // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"

#include "json.hpp"

using namespace std;
using json = nlohmann::json;

class prioritize {
   public:
    bool operator()(const pair<long long, double>& p1,
                    const pair<long long, double>& p2) const {
        return p1.second > p2.second;
    }
};

double INF = numeric_limits<double>::max();

void buildGraph(istream& input, graph<long long, double>& g,
                vector<BuildingInfo>& buildings) {

  json data;
  input >> data;

  // extract buildings and add them to the graph
  for (const auto& building : data["buildings"]) {
    long long id = building["id"];
    double lat = building["lat"];
    double lon = building["lon"];
    string name = building["name"];
    // string abbr = building.value("abbr", "?");
    string abbr = building["abbr"];

    buildings.emplace_back(id, Coordinates{lat, lon}, name, abbr);
    g.addVertex(id);
  }

  // extract waypoints and add them to the graph
  for (const auto& waypoint : data["waypoints"]) {
    long long id = waypoint["id"];
    g.addVertex(id);
  }

  // add edges between waypoints in footways
  for (const auto& footway : data["footways"]) {
    for (size_t i = 0; i + 1 < footway.size(); i++) {
      long long from = footway[i];
      long long to = footway[i + 1];

      // find corresponding coordinates for the waypoints
      auto fromIt = find_if(
          data["waypoints"].begin(), data["waypoints"].end(),
          [from](const json& wp) { return wp["id"] == from; });

      auto toIt = find_if(
          data["waypoints"].begin(), data["waypoints"].end(),
          [to](const json& wp) { return wp["id"] == to; });

      if (fromIt != data["waypoints"].end() && toIt != data["waypoints"].end()) {
          Coordinates fromCoord{(*fromIt)["lat"], (*fromIt)["lon"]};
          Coordinates toCoord{(*toIt)["lat"], (*toIt)["lon"]};
          double distance = distBetween2Points(fromCoord, toCoord);

          g.addEdge(from, to, distance);
          g.addEdge(to, from, distance);
      }
    }
  }

  // connect buildings to nearby waypoints within 0.036 miles
  for (const auto& building : buildings) {
    for (const auto& waypoint : data["waypoints"]) {
      long long waypointId = waypoint["id"];
      double distance = distBetween2Points(
          building.location, Coordinates{waypoint["lat"], waypoint["lon"]});
      if (distance <= 0.036) {
          g.addEdge(building.id, waypointId, distance);
          g.addEdge(waypointId, building.id, distance);
      }
    }
  }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo>& buildings,
                             const string& query) {
  for (const BuildingInfo& building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo>& buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo& building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double>& G, long long start,
                           long long target,
                           const set<long long>& ignoreNodes) {

  // handle case when start and target are the same
  if (start == target) {
      return {start};
  }

  // priority queue for exploring nodes
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> pq;

  // maps to store distances and paths
  unordered_map<long long, double> distances;
  unordered_map<long long, long long> prev;
  unordered_set<long long> visited;

  // initialize all distances to infinity
  vector<long long> vertices = G.getVertices();
  for (const auto& v : vertices) {
      distances[v] = INF;
  }
  distances[start] = 0;

  // start with the initial node
  pq.push({start, 0});

  while (!pq.empty()) {
      auto curr = pq.top().first;
      auto currDistance = pq.top().second;
      pq.pop();

      // skip visited nodes
      if (visited.count(curr)) continue;
      visited.insert(curr);

      // stop if target is reached
      if (curr == target) break;

      // explore neighbors
      set<long long> neighbors = G.neighbors(curr);
      for (const auto& n : neighbors) {
          // skip ignored nodes unless they are start or target
          if (ignoreNodes.count(n) && n != start && n != target) {
              continue;
          }

          // check the edge weight
          double weight;
          if (G.getWeight(curr, n, weight)) {
              double newDistance = currDistance + weight;

              // update if a shorter path is found
              if (newDistance < distances[n]) {
                  distances[n] = newDistance;
                  prev[n] = curr;
                  pq.push({n, newDistance});
              }
          }
      }
  }

  // build the path
  vector<long long> path;
  if (distances[target] == INF) {
      return {}; // return empty if no path exists
  }

  for (long long at = target; at != start; at = prev[at]) {
      path.push_back(at);
  }
  path.push_back(start);

  // reverse the path to get it from start to target
  reverse(path.begin(), path.end());
  return path;
}

double pathLength(const graph<long long, double>& G,
                  const vector<long long>& path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

void application(const vector<BuildingInfo>& buildings,
                 const graph<long long, double>& G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto& building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
