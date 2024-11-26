#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "application.h"
#include "graph.h"

using namespace std;
using namespace testing;

// Global variables to avoid multiple expensive loads/parses
graph<long long, double> UIC_GRAPH;
set<long long> BUILDING_NODES;

void fillUicGraph() {
  if (UIC_GRAPH.numVertices()) return;

  vector<BuildingInfo> buildings;
  ifstream input("data/uic-fa24.osm.json");
  buildGraph(input, UIC_GRAPH, buildings);

  for (const auto& b : buildings) {
    BUILDING_NODES.insert(b.id);
  }
}

/// Produce a graph that is a line of nodes (i.e. singly linked list)
// Note that the other tests were altered for better assertions
graph<long long, double> lineGraph(int n) {
  graph<long long, double> g;

  for (int i = 0; i < n; i++) {
    g.addVertex(i);
  }

  for (int i = 0; i < n - 1; i++) {
    g.addEdge(i, i + 1, i + 1);
  }
  return g;
}

TEST(Dijkstra, StartTargetSame) {
  graph<long long, double> g = lineGraph(3);

  ASSERT_THAT(dijkstra(g, 0, 0, {}), ElementsAreArray({0}))
      << "Path from node to itself should be the node";
}

TEST(Dijkstra, Unreachable) {
  graph<long long, double> g = lineGraph(3);

  vector<long long> expected = {};
  // 0 --> 1 --> 2; no path back from 2 to 0
  ASSERT_THAT(dijkstra(g, 2, 0, {}), IsEmpty())
      << "Unreachable nodes have an empty path";
}

TEST(Dijkstra, BasicLineGraph) {
  graph<long long, double> g = lineGraph(3);
  vector<long long> expected = {0, 1, 2};
  ASSERT_THAT(dijkstra(g, 0, 2, {}), ElementsAreArray({0, 1, 2}))
      << "Wrong shortest path in line graph";
}

TEST(Dijkstra, BasicLineGraphWithEndNodesExcluded) {
  graph<long long, double> g = lineGraph(3);

  ASSERT_THAT(dijkstra(g, 0, 2, {0, 2}), ElementsAreArray({0, 1, 2}))
      << "Wrong shortest path in line graph. "
      << "The start and target nodes may be in skipNodes.";
}

TEST(Dijkstra, LineGraphLongLongIds) {
  graph<long long, double> g;
  for (long long i = 0; i < 5; i++) {
    g.addVertex(i + 11139567102LL);
  }

  for (int i = 0; i < 4; i++) {
    g.addEdge(i + 11139567102LL, i + 11139567102LL + 1, i + 1);
  }

  ASSERT_THAT(
      dijkstra(g, 0 + 11139567102LL, 4 + 11139567102LL, {}),
      ElementsAreArray({0 + 11139567102LL, 1 + 11139567102LL, 2 + 11139567102LL,
                        3 + 11139567102LL, 4 + 11139567102LL}))
      << "Wrong shortest path in line graph that uses very large vertex IDs";
}

TEST(Dijkstra, BasicLineGraphSkipNodes) {
  graph<long long, double> g = lineGraph(3);
  g.addVertex(10);
  g.addEdge(0, 10, 0.01);
  g.addEdge(10, 2, 0.01);

  ASSERT_THAT(dijkstra(g, 0, 2, {0, 2, 10}), ElementsAreArray({0, 1, 2}))
      << "Wrong shortest path in line graph, considering skipped nodes." << endl
      << "The shortest path shouldn't go through skipped nodes (except for the "
         "start and target).";
}

TEST(Dijkstra, SmallTest) {
  graph<long long, double> g;
  for (long long i = 0; i < 5; i++) {
    g.addVertex(i);
  }
  // Short path, many edges vs. longer path, one edge
  g.addEdge(0, 1, 100.0);
  g.addEdge(1, 2, 100.0);
  g.addEdge(2, 3, 100.0);
  g.addEdge(3, 4, 100.0);
  g.addEdge(0, 4, 700.0);

  ASSERT_THAT(dijkstra(g, 0, 4, {}), ElementsAreArray({0, 1, 2, 3, 4}))
      << "Wrong shortest path in graph.";
}

TEST(Dijkstra, FloatingPoint) {
  graph<long long, double> g;
  int N = 10;
  for (long long i = 0; i < N; i++) {
    g.addVertex(i);
  }
  for (long long i = 0; i < N - 1; i++) {
    g.addEdge(i, i + 1, 0.3);
  }
  g.addEdge(0, 9, 2);

  // Are you storing anything as an int?
  ASSERT_THAT(dijkstra(g, 0, 9, {}), ElementsAreArray({0, 9}))
      << "Wrong shortest path in graph, when weights are small doubles";
}

TEST(Dijkstra, ThatOneGraphFromLecture_Maybe) {
  graph<long long, double> g;
  for (long long i = 0; i < 8; i++) {
    g.addVertex(i);
  }
  g.addEdge(0, 1, 2.0);
  g.addEdge(0, 2, 1.0);
  g.addEdge(0, 3, 4.0);

  g.addEdge(1, 5, 2.0);
  g.addEdge(1, 4, 8.0);
  g.addEdge(1, 2, 5.0);

  g.addEdge(2, 0, 9.0);
  g.addEdge(2, 4, 11.0);

  g.addEdge(3, 2, 2.0);

  g.addEdge(4, 7, 3.0);

  g.addEdge(5, 6, 3.0);

  g.addEdge(6, 7, 1.0);

  g.addEdge(7, 5, 2.0);
  g.addEdge(7, 4, 1.0);

  ASSERT_THAT(dijkstra(g, 0, 4, {}), ElementsAreArray({0, 1, 5, 6, 7, 4}))
      << "Wrong shortest path in graph.";
}

TEST(Dijkstra, YetAnotherSmallTest) {
  graph<long long, double> g;
  for (long long i = 0; i < 6; i++) {
    g.addVertex(i);
  }
  g.addEdge(0, 1, 1.0);
  g.addEdge(1, 2, 1.0);
  g.addEdge(2, 3, 1.0);
  g.addEdge(3, 4, 1.0);
  g.addEdge(0, 5, 90.0);
  g.addEdge(1, 5, 80.0);
  g.addEdge(2, 5, 70.0);
  g.addEdge(3, 5, 60.0);
  g.addEdge(4, 5, 50.0);

  ASSERT_THAT(dijkstra(g, 0, 5, {}), ElementsAreArray({0, 1, 2, 3, 4, 5}))
      << "Wrong shortest path in graph.";
}

TEST(Dijkstra, YouThoughtItWasDoneDidntYou) {
  graph<long long, double> g;
  for (long long i = 0; i < 7; i++) {
    g.addVertex(i);
  }
  g.addEdge(0, 2, 2.0);
  g.addEdge(0, 3, 1.0);
  g.addEdge(1, 0, 2.0);
  g.addEdge(2, 3, 1.0);
  g.addEdge(2, 5, 2.0);
  g.addEdge(3, 4, 1.0);
  g.addEdge(3, 6, 5.0);
  g.addEdge(3, 1, 5.0);
  g.addEdge(4, 1, 1.0);
  g.addEdge(5, 6, 10.0);
  g.addEdge(6, 4, 3.0);

  ASSERT_THAT(dijkstra(g, 4, 6, {}), ElementsAreArray({4, 1, 0, 3, 6}))
      << "Wrong shortest path in graph.";
}

TEST(Dijkstra, Full) {
  fillUicGraph();

  long long arc = 664275388;  // Person 1: ARC
  long long seo = 151960667;  // Person 2: SEO
  long long sh = 151676521;   // Midpoint: SH

  vector<long long> expectedArcToSh = {
      664275388,  9007520455,  2412572929, 1645208827, 464345369,
      463814052,  11174974876, 464748194,  462010750,  462010751,
      9862302685, 9870872111,  9862302686, 9862302687, 9862302692,
      9870872081, 9862302654,  9862302653, 151676521};
  vector<long long> expectedSeoToSh = {
      151960667,  1647971930,  462010746,  12108530536, 1645121274,
      1645121428, 12108530537, 1645121533, 1647973070,  151676521};
  EXPECT_THAT(dijkstra(UIC_GRAPH, arc, sh, BUILDING_NODES),
              ElementsAreArray(expectedArcToSh))
      << "Wrong shortest path in UIC graph from ARC to SH.";
  ASSERT_THAT(dijkstra(UIC_GRAPH, seo, sh, BUILDING_NODES),
              ElementsAreArray(expectedSeoToSh))
      << "Wrong shortest path in UIC graph from SEO to SH.";
}

TEST(Dijkstra, FullSkipNodes) {
  fillUicGraph();

  long long arc = 664275388;  // Person 1: ARC
  long long erf = 151960677;  // Person 2: ERF
  long long lcb = 151672203;  // Midpoint: LCB

  // ARC-->LCB can shortcut across buildings
  // So this path is longer than the "true" shortest:
  // { 664275388,  9007520455, 2412572929, 1645208827, 151672201, 9870872081,
  //   9862302654, 9862302653, 151676521,  9862302779, 151672203 }
  vector<long long> expectedArcToLcb = {
      664275388,   9007520455,  2412572929, 1645208827,  464345369,
      463814052,   11174974876, 464748194,  462010750,   462010751,
      9862302685,  9870872111,  7511858534, 9870872110,  462010753,
      9870872084,  462010766,   9870872109, 9870872108,  462010765,
      9870872105,  11942526176, 9862276883, 11942526177, 9862365841,
      11942526178, 9862365842,  9870872104, 9862365843,  151672203};
  vector<long long> expectedErfToLcb = {
      151960677,   5632908807,  5632908804,  4226840176, 1308175623,
      10612976164, 10612976165, 1770663458,  4226840184, 1647971942,
      1647971957,  462014176,   9870872090,  462010748,  9862302460,
      462010745,   10930768589, 11757616192, 151672203};

  EXPECT_THAT(dijkstra(UIC_GRAPH, arc, lcb, BUILDING_NODES),
              ElementsAreArray(expectedArcToLcb))
      << "Wrong shortest path in UIC graph from ARC to LCB. You may not be "
         "skipping the given nodes.";
  ASSERT_THAT(dijkstra(UIC_GRAPH, erf, lcb, BUILDING_NODES),
              ElementsAreArray(expectedErfToLcb))
      << "Wrong shortest path in UIC graph from ERF to LCB.";
}
