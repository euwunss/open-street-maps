#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "application.h"
#include "dist.h"
#include "graph.h"

using namespace std;
using namespace testing;

Coordinates intToCoords(int n) {
  return Coordinates(n, n);
}

TEST(BuildGraph, BasicNodesInGraph) {
  graph<long long, double> g;
  vector<BuildingInfo> buildings;
  ifstream input("data/empty.json");
  buildGraph(input, g, buildings);
  vector<long long> actualVertices = g.getVertices();
  sort(actualVertices.begin(), actualVertices.end());

  ASSERT_THAT(g.numVertices(), Eq(0));
  ASSERT_THAT(actualVertices, IsEmpty());
  ASSERT_THAT(buildings, IsEmpty());
  ASSERT_THAT(g.numEdges(), Eq(0));
}

TEST(BuildGraph, LineGraphNoBuildings) {
  map<long long, Coordinates> nodes;
  vector<long long> expectedVertices;

  for (int i = 0; i <= 4; i++) {
    nodes.emplace(
        i, Coordinates(41.87 + 0.001 * (i + 1), -87.65 - 0.001 * (i + 1)));
    expectedVertices.push_back(i);
  }
  nodes.emplace(5, Coordinates(0, 0));
  nodes.emplace(6, Coordinates(100, -100));
  expectedVertices.push_back(5);
  expectedVertices.push_back(6);

  graph<long long, double> g;
  vector<BuildingInfo> buildings;
  ifstream input("data/line.json");
  buildGraph(input, g, buildings);
  vector<long long> actualVertices = g.getVertices();
  sort(actualVertices.begin(), actualVertices.end());

  ASSERT_THAT(g.numVertices(), Eq(nodes.size()));
  ASSERT_THAT(actualVertices, UnorderedElementsAreArray(expectedVertices));
  ASSERT_THAT(buildings, IsEmpty());

  // 4 undirected edges
  ASSERT_THAT(g.numEdges(), Eq(4 * 2));

  double weight;
  ASSERT_THAT(g.getWeight(0, 1, weight), IsTrue());
  ASSERT_THAT(weight, DoubleNear(0.0862397, 1e-6));
}

TEST(BuildGraph, Buildings) {
  graph<long long, double> g;
  vector<BuildingInfo> actualBuildings;
  ifstream input("data/small_buildings.json");
  buildGraph(input, g, actualBuildings);

  ASSERT_THAT(g.numVertices(), Eq(4));

  vector<BuildingInfo> expectedBuildings = {
    BuildingInfo(1, Coordinates(41.8720714, -87.6492469), "North Side of Quad", "NSQ"),
    BuildingInfo(2, Coordinates(41.871696, -87.649267), "South Side of Quad", "SSQ")
  };

  ASSERT_THAT(actualBuildings, UnorderedElementsAreArray(expectedBuildings));

  double weight;
  ASSERT_THAT(g.getWeight(1, 1, weight), IsFalse()) << "Buildings should not have an edge to themselves";
  ASSERT_THAT(g.getWeight(1, 2, weight), IsFalse()) << "Buildings should not have edges to other buildings";
  ASSERT_THAT(g.getWeight(2, 1, weight), IsFalse()) << "Buildings should not have edges to other buildings";

  ASSERT_THAT(g.getWeight(2, 3, weight), IsTrue()) << "Buildings should have edges to nearby vertices";
  EXPECT_THAT(weight, DoubleNear(0.0217161, 1e-6));
  ASSERT_THAT(g.getWeight(3, 2, weight), IsTrue()) << "Building edges should go in both directions";
  EXPECT_THAT(weight, DoubleNear(0.0217161, 1e-6));

  ASSERT_THAT(g.getWeight(1, 3, weight), IsTrue()) << "Buildings should have edges to nearby vertices";
  EXPECT_THAT(weight, DoubleNear(0.0192231, 1e-6));
  ASSERT_THAT(g.getWeight(3, 1, weight), IsTrue()) << "Building edges should go in both directions";
  EXPECT_THAT(weight, DoubleNear(0.0192231, 1e-6));

  ASSERT_THAT(g.numEdges(), Eq(4)) << "Too many edges found -- is the distance threshold correct?";
}

TEST(BuildGraph, Full) {
  graph<long long, double> g;
  vector<BuildingInfo> buildings;
  ifstream input("data/uic-fa24.osm.json");
  buildGraph(input, g, buildings);

  EXPECT_THAT(g.numVertices(), 7292);
  EXPECT_THAT(g.numEdges(), 22218);

  double weight;
  ASSERT_THAT(g.getWeight(10930768586, 11757616193, weight), IsTrue());
  ASSERT_THAT(weight, DoubleNear(0.010135770596645196, 1e-6));

  ASSERT_THAT(g.getWeight(11757616193, 10930768586, weight), IsTrue());
  ASSERT_THAT(weight, DoubleNear(0.010135770596645196, 1e-6));
}
