// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ASTAR__ASTAR_HPP_
#define ASTAR__ASTAR_HPP_

#include <cassert>
#include <iostream>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <utility>
#include <vector>

namespace astar
{

// Nodes
using node_type = int;

// Edges
struct Edge
{
  node_type a;
  node_type b;

  // Distance between a and b, in either direction
  double distance;

  Edge(node_type _a, node_type _b, double _distance)
  : a{_a}, b{_b}, distance{_distance} {}
};

// Used for get_neighbors
struct Neighbor
{
  node_type node;

  // Distance to this neighbor
  double distance;

  Neighbor(node_type _node, double _distance)
  : node{_node}, distance{_distance} {}
};

// Graph
struct Graph
{
  std::vector<Edge> edges_;

  explicit Graph(std::vector<Edge> graph)
  : edges_{std::move(graph)} {}

  std::vector<Neighbor> get_neighbors(node_type node);
};

// Nodes in the open_set_ contain additional state
struct CandidateNode
{
  node_type node;

  // Cost of cheapest path from start to this node
  double g_score;

  // Heuristic for cost of cheapest path from start through this node to the destination
  // f_score = g_score + h_(this)
  double f_score;

  // Must be default constructible to be used in a priority queue
  CandidateNode() = default;

  // Useful constructor
  CandidateNode(node_type _node, double _g_score, double h)
  : node{_node}, g_score{_g_score}, f_score{_g_score + h} {}

  // Priority queue uses value operator as comparator
  bool operator()(const CandidateNode & a, const CandidateNode & b)
  {
    return a.f_score > b.f_score;
  }
};

std::ostream & operator<<(std::ostream & os, CandidateNode const & c);

// Find the shortest path in a graph
class Solver
{
  Graph graph_;

  // Heuristic function provides a rough guess of distance between 2 nodes,
  // must be < than actual distance
  using HeuristicFn = std::function<double (node_type a, node_type b)>;

  HeuristicFn h_;

  // Best g_score from start to a node
  std::map<node_type, double> best_g_score_;

  // Parent nodes, used to reconstruct the path at the end
  std::map<node_type, node_type> best_parents_;

  // Set of candidate nodes
  std::priority_queue<CandidateNode, std::vector<CandidateNode>, CandidateNode> open_set_;

  void print_open_set();

  void reconstruct_path(node_type node, std::vector<node_type> & path);

  void reset();

public:
  explicit Solver(std::vector<Edge> edges, HeuristicFn h)
  : graph_{Graph(std::move(edges))}, h_{std::move(h)} {}

  // Find the best path from start to destination, return true if successful
  bool find_shortest_path(node_type start, node_type destination, std::vector<node_type> & result);
};

}  // namespace astar

#endif  // ASTAR__ASTAR_HPP_
