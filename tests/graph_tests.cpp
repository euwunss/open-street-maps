#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "graph.h"

using namespace std;
using namespace testing;

// I'm not super happy with the monomorphization --> compile times here
// But templated headers kill a lot of things

void tournamentGraph(int n, graph<int, int>& g) {
  bool buildSuccess = true;
  for (int i = 0; i < n; i++) {
    buildSuccess &= g.addVertex(i);
  }
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < i; j++) {
      // Edge from i to j iff i > j
      buildSuccess &= g.addEdge(i, j, i);
    }
  }
  ASSERT_THAT(buildSuccess, IsTrue());
}

void completeGraph(int n, graph<int, int>& g) {
  bool buildSuccess = true;
  for (int i = 0; i < n; i++) {
    buildSuccess &= g.addVertex(i);
  }
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) continue;
      buildSuccess &= g.addEdge(i, j, i);
    }
  }
  ASSERT_THAT(buildSuccess, IsTrue());
}

TEST(Graph, EmptyGraph) {
  graph<string, string> g;
  EXPECT_THAT(g.numVertices(), Eq(0)) << "Empty graph should have 0 vertices";
  EXPECT_THAT(g.getVertices(), IsEmpty())
      << "Empty graph should have 0 vertices";
  EXPECT_THAT(g.numEdges(), Eq(0)) << "Empty graph should have 0 edges";
}

TEST(Graph, BadEdges) {
  graph<string, int> g;

  ASSERT_THAT(g.addVertex("0"), IsTrue());

  ASSERT_THAT(g.addEdge("missing_1", "missing_2", 1), IsFalse())
      << "addEdge when both `from` and `to` are missing should be false";
  ASSERT_THAT(g.addEdge("0", "missing", 1), IsFalse())
      << "addEdge when `to` is missing should be false";
  ASSERT_THAT(g.addEdge("missing", "0", 1), IsFalse())
      << "addEdge when `from` is missing should be false";

  ASSERT_THAT(g.numEdges(), Eq(0))
      << "Edge count changed when adding invalid edges";
}

TEST(Graph, BuildSmall) {
  graph<string, int> g;
  vector<string> expectedVertices = {"0", "1", "2", "3"};

  ASSERT_THAT(g.addVertex("0"), IsTrue())
      << "Should be able to add a new vertex";
  ASSERT_THAT(g.addVertex("1"), IsTrue())
      << "Should be able to add a new vertex";
  ASSERT_THAT(g.addVertex("2"), IsTrue())
      << "Should be able to add a new vertex";
  ASSERT_THAT(g.addVertex("3"), IsTrue())
      << "Should be able to add a new vertex";

  ASSERT_THAT(g.addEdge("0", "1", 2), IsTrue())
      << "Should be able to add a new edge";
  ASSERT_THAT(g.addEdge("2", "1", 4), IsTrue())
      << "Should be able to add a new edge";
  ASSERT_THAT(g.addEdge("1", "2", 3), IsTrue())
      << "Should be able to add a new edge";
  ASSERT_THAT(g.addEdge("1", "3", 1), IsTrue())
      << "Should be able to add a new edge";
  ASSERT_THAT(g.addEdge("3", "1", -1), IsTrue())
      << "Should be able to add a new edge";
  ASSERT_THAT(g.addEdge("0", "0", -2), IsTrue())
      << "Should be able to add a new self-edge";

  vector<string> actualVertices = g.getVertices();
  ASSERT_THAT(actualVertices, UnorderedElementsAreArray(expectedVertices));
  ASSERT_THAT(g.numVertices(), Eq(4));
  ASSERT_THAT(g.numEdges(), Eq(6));

  int actualWeight;
  ASSERT_THAT(g.getWeight("0", "1", actualWeight), IsTrue())
      << "Edge should exist in graph";
  ASSERT_THAT(actualWeight, Eq(2)) << "Edge has wrong weight";
  ASSERT_THAT(g.getWeight("2", "1", actualWeight), IsTrue())
      << "Edge should exist in graph";
  ASSERT_THAT(actualWeight, Eq(4)) << "Edge has wrong weight";
  ASSERT_THAT(g.getWeight("1", "2", actualWeight), IsTrue())
      << "Edge should exist in graph";
  ASSERT_THAT(actualWeight, Eq(3)) << "Edge has wrong weight";
  ASSERT_THAT(g.getWeight("1", "3", actualWeight), IsTrue())
      << "Edge should exist in graph";
  ASSERT_THAT(actualWeight, Eq(1)) << "Edge has wrong weight";
  ASSERT_THAT(g.getWeight("3", "1", actualWeight), IsTrue())
      << "Edge should exist in graph";
  ASSERT_THAT(actualWeight, Eq(-1)) << "Edge has wrong weight";
  ASSERT_THAT(g.getWeight("0", "0", actualWeight), IsTrue())
      << "Edge should exist in graph";
  ASSERT_THAT(actualWeight, Eq(-2)) << "Edge has wrong weight";

  ASSERT_THAT(g.getWeight("1", "0", actualWeight), IsFalse())
      << "Edge should not exist in graph";
  ASSERT_THAT(g.getWeight("2", "3", actualWeight), IsFalse())
      << "Edge should not exist in graph";
}

TEST(Graph, EdgeOverwrite) {
  graph<string, int> g;

  ASSERT_THAT(g.addVertex("0"), IsTrue());
  ASSERT_THAT(g.addEdge("0", "0", 1), IsTrue()) << "addEdge should succeed";
  ASSERT_THAT(g.addEdge("0", "0", 2), IsTrue())
      << "addEdge replacing an edge should succeed";

  int weight;
  ASSERT_THAT(g.getWeight("0", "0", weight), IsTrue());
  ASSERT_THAT(weight, Eq(2));
  ASSERT_THAT(g.numEdges(), Eq(1))
      << "Edge count changed when overwriting edges";
}

TEST(Graph, CompleteEdgesStress) {
  int n = 400;
  graph<int, int> g;
  completeGraph(n, g);
  int weight;

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i != j) {
        ASSERT_THAT(g.getWeight(i, j, weight), IsTrue())
            << "Edge should be present in complete graph";
        ASSERT_THAT(weight, Eq(i)) << "Edge has wrong weight";
      } else {
        ASSERT_THAT(g.getWeight(i, j, weight), IsFalse())
            << "Self-edge should not be present in complete graph";
      }
    }
  }
}

TEST(Graph, NeighborsStress) {
  int n = 100;
  graph<int, int> g;
  tournamentGraph(n, g);
  if (HasFatalFailure()) return;

  set<int> expectedNeighbors;
  vector<int> expectedVertices;

  for (int i = 0; i < n; i++) {
    ASSERT_THAT(g.neighbors(i), ContainerEq(expectedNeighbors));
    // In a tournament graph, vertices have edges to smaller ones
    expectedNeighbors.emplace(i);
    expectedVertices.push_back(i);
  }

  vector<int> actualVertices = g.getVertices();
  ASSERT_THAT(actualVertices, UnorderedElementsAreArray(expectedVertices));
}

TEST(Graph, DefaultCopies) {
  graph<int, int> g;
  completeGraph(4, g);
  if (HasFatalFailure()) return;

  graph<int, int> copy = g;
  graph<int, int> opEq;
  opEq = g;
}

TEST(Graph, ManyVertices) {
  graph<int, int> g;
  int N = 500000;
  // Assertions take time, so just use a global boolean
  // Before reaching this point, should be sure that things work
  bool buildSuccess = true;

  // Populating vertices, expect true
  for (int i = 0; i < N; i++) {
    if (!g.addVertex(i)) {
      buildSuccess = false;
    }
  }

  // Adding a duplicate vertex, expect false
  for (int i = 0; i < N; i++) {
    if (g.addVertex(N - 1)) {
      buildSuccess = false;
    }
  }

  // Populating edges, expect true
  for (int i = 0; i < N - 1; i++) {
    if (!g.addEdge(i, i + 1, 2 * i)) {
      buildSuccess = false;
    }
  }

  // More edges, expect true
  for (int i = 2; i < N; i++) {
    if (!g.addEdge(0, i, i)) {
      buildSuccess = false;
    }
  }
  ASSERT_THAT(buildSuccess, IsTrue())
      << "Incorrect return while building the graph";

  int weight;
  for (int i = 0; i < N - 1; i++) {
    ASSERT_THAT(g.getWeight(i, i + 1, weight), IsTrue());
    ASSERT_THAT(weight, Eq(i * 2));
  }
  for (int i = 2; i < N; i++) {
    ASSERT_THAT(g.getWeight(0, i, weight), IsTrue());
    ASSERT_THAT(weight, Eq(i));
  }
}

TEST(Graph, BigCompleteGraph) {
  int N = 1000;
  graph<int, int> g;
  completeGraph(N, g);
  if (HasFatalFailure()) return;

  int weight;
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      if (i == j) {
        ASSERT_THAT(g.getWeight(i, j, weight), IsFalse());
      } else {
        ASSERT_THAT(g.getWeight(i, j, weight), IsTrue());
      }
    }
  }
}
