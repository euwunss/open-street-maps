#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
 private:
  size_t edgeCount;
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> adjacencyList;

 public:
  /// Default constructor
  graph() {
    this->edgeCount = 0;
  }

  /// @brief Add the vertex `v` to the graph, must typically be O(1).
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    // check if the vertex doesn't exist in the adjacency list
    if (adjacencyList.find(v) != adjacencyList.end())
      return false;

    adjacencyList[v] = {};
    return true;
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    // check if both vertices exist in the graph
    auto fromIt = adjacencyList.find(from);
    if (fromIt == adjacencyList.end() ||
        adjacencyList.find(to) == adjacencyList.end()) {
      return false;
    }

    // check if edge is new and increment edge count for new edge
    if (adjacencyList[from].find(to) == adjacencyList[from].end())
      edgeCount++;

    adjacencyList[from][to] = weight;
    return true;
  }

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    // check if the 'from' vertex exists in the adjacency list
    auto fromIt = adjacencyList.find(from);
    if (fromIt == adjacencyList.end())
      return false;

    auto toIt = fromIt->second.find(to);
    if (toIt == fromIt->second.end())
      return false;

    weight = toIt->second;
    return true;
  }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;

    // check if the vertex exists in the adjacency list
    auto it = adjacencyList.find(v);
    if (it != adjacencyList.end()) {
      // add all neighbors to the set
      for (const auto& edge : it->second) {
        S.insert(edge.first);
      }
    }

    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
    vector<VertexT> vertices;

    // iterate through the adjacency list and push all vertices to the vector
    for (const auto& pair : adjacencyList) {
      vertices.push_back(pair.first);
    }

    return vertices;
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    return adjacencyList.size();
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    return this->edgeCount;
  }
};
