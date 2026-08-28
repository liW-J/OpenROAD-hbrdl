// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <string>
#include <unordered_set>
#include <vector>

#include "grt/GRoute.h"

namespace odb {
class dbDatabase;
class Point;
}  // namespace odb

namespace utl {
class Logger;
}

namespace grt {

class Grid;
class Net;

// A deterministic, congestion-unaware true-3D global-routing backend.
//
// The configured cut layer partitions the routing stack into two dies. A net
// contained in one die never crosses the cut. A net spanning both dies uses
// exactly one cut location and is routed as two die-local rectilinear trees
// joined at that location.
class True3DRouter
{
 public:
  True3DRouter(odb::dbDatabase* db, Grid* grid, utl::Logger* logger);

  NetRouteMap route(const std::vector<Net*>& nets,
                    int min_routing_layer,
                    int max_routing_layer,
                    const std::string& cut_layer_name);

 private:
  struct RouteNode
  {
    int x;
    int y;
    int layer;
  };

  struct DieStack
  {
    int min_layer;
    int max_layer;
    int horizontal_layer;
    int vertical_layer;
  };

  struct Interface
  {
    int bottom_layer;
    int upper_layer;
    DieStack bottom;
    DieStack upper;
  };

  struct Statistics
  {
    int routed_nets = 0;
    int die_local_nets = 0;
    int cross_die_nets = 0;
    int hbt_count = 0;
    std::size_t segment_count = 0;
  };

  using SegmentSet = std::unordered_set<GSegment, GSegmentHash>;

  Interface makeInterface(const std::string& cut_layer_name,
                          int min_routing_layer,
                          int max_routing_layer) const;
  DieStack makeDieStack(int min_layer,
                        int max_layer,
                        bool search_from_top) const;
  GRoute routeNet(Net* net, const Interface& interface);
  void routeLocalTree(const std::vector<RouteNode>& nodes,
                      const DieStack& stack,
                      GRoute& route,
                      SegmentSet& segments) const;
  void routePair(const RouteNode& source,
                 const RouteNode& target,
                 const DieStack& stack,
                 GRoute& route,
                 SegmentSet& segments) const;
  void addWire(int x0,
               int y0,
               int x1,
               int y1,
               int layer,
               GRoute& route,
               SegmentSet& segments) const;
  void addViaStack(int x,
                   int y,
                   int layer0,
                   int layer1,
                   GRoute& route,
                   SegmentSet& segments) const;
  void addSegment(const GSegment& segment,
                  GRoute& route,
                  SegmentSet& segments) const;
  odb::Point selectHbt(const std::vector<RouteNode>& terminals) const;

  odb::dbDatabase* db_;
  Grid* grid_;
  utl::Logger* logger_;
  Statistics statistics_;
};

}  // namespace grt
