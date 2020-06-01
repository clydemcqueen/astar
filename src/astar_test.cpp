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

#include "astar/astar.hpp"

#include <vector>

void astar_test()
{
  std::cout << "=== ASTAR TEST ===" << std::endl;

  using astar::Edge;

  std::vector<Edge> edges = std::vector<Edge>{
    Edge(0, 1, 10),
    Edge(0, 2, 15),
    Edge(1, 3, 10),
    Edge(1, 2, 5),
    Edge(2, 3, 15)
  };

  // A* heuristic: how far from node to the destination?
  auto heuristic = [](astar::node_type a, astar::node_type b) -> double
    {
      if (a == 0) {
        return 18;
      } else if (a == 1) {
        return 9;
      } else if (a == 2) {
        return 12;
      } else if (a == 3) {
        return 0;
      } else {
        assert(false);
      }
    };

  auto solver = astar::Solver(edges, heuristic);

  std::vector<int> path;
  std::cout << (solver.find_shortest_path(0, 3, path) ? "success" : "failure") << std::endl;

  for (auto item : path) {
    std::cout << item << ", ";
  }
  std::cout << std::endl;
}

int main(int argc, char ** argv)
{
  astar_test();
}
