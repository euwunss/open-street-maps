#include <fstream>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "application.h"
#include "graph.h"

using namespace std;

int main() {
  cout << "** Navigating UIC open street map **" << endl;
  cout << std::setprecision(8);

  string default_filename = "data/uic-fa24.osm.json";

  // Build graph from input data
  graph<long long, double> G;
  vector<BuildingInfo> buildings;
  ifstream input(default_filename);
  buildGraph(input, G, buildings);

  cout << "# of buildings: " << buildings.size() << endl;

  cout << "# of vertices: " << G.numVertices() << endl;
  cout << "# of edges: " << G.numEdges() << endl;
  application(buildings, G);

  cout << "** Done **" << endl;
  return 0;
}
