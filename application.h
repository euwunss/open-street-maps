#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "dist.h"
#include "graph.h"

using namespace std;

struct BuildingInfo {
  long long id;
  Coordinates location;
  string name;
  string abbr;

  BuildingInfo() : id(0) {
  }

  BuildingInfo(long long id, Coordinates location, string name, string abbr)
      : id(id), location(location), name(name), abbr(abbr) {
  }

  bool operator==(const BuildingInfo& other) const {
    return id == other.id && name == other.name && abbr == other.abbr &&
           location.lat == other.location.lat &&
           location.lon == other.location.lon;
  }
};

/// @brief Build a `graph` from the given map data in the input JSON. Building
///        centers that are "close to" footway nodes are manually linked.
/// @param input stream containing JSON
/// @param G resulting graph, by reference
/// @param buildings buildings in resulting graph, by reference
void buildGraph(istream& input, graph<long long, double>& G,
                vector<BuildingInfo>& buildings);

/// @brief Queries the `buildings` info to find a building that matches the
///        query. Either the query is exactly the abbreviation, or the query
///        is a substring of the building's name.
/// @param buildings
/// @param query abbreviation or substring of building name
/// @return the building info, or a building with `id = -1` if not found
BuildingInfo getBuildingInfo(vector<BuildingInfo>& buildings,
                             const string& query);

/// @brief Searches for the closest building to the provided coordinates
/// @param buildings
/// @param c
/// @return
BuildingInfo getClosestBuilding(vector<BuildingInfo>& buildings, Coordinates c);

/// @brief Run Dijkstra's algorithm on G to find the shortest path from `start`
///        to `target` that does not include any ignored nodes.
/// @param G graph
/// @param start starting node ID
/// @param target ending node ID
/// @param ignoreNodes node IDs to skip; may contain `start` or `target` (which
///                    should not be ignored)
/// @return node IDs on shortest path from `start` to `target`
vector<long long> dijkstra(const graph<long long, double>& G, long long start,
                           long long target, const set<long long>& ignoreNodes);

/// Command loop to request input
void application(const vector<BuildingInfo>& Buildings,
                 const graph<long long, double>& G);
