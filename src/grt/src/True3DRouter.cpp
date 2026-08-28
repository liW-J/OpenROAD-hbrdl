// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include "True3DRouter.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "Grid.h"
#include "Net.h"
#include "Pin.h"
#include "odb/db.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "utl/Logger.h"

namespace grt {

using utl::GRT;

True3DRouter::True3DRouter(odb::dbDatabase* db,
                           Grid* grid,
                           utl::Logger* logger)
    : db_(db), grid_(grid), logger_(logger)
{
}

NetRouteMap True3DRouter::route(const std::vector<Net*>& nets,
                                const int min_routing_layer,
                                const int max_routing_layer,
                                const std::string& cut_layer_name)
{
  statistics_ = {};
  const Interface interface = makeInterface(
      cut_layer_name, min_routing_layer, max_routing_layer);

  logger_->info(GRT,
                321,
                "Running basic true-3D global routing across cut layer {} "
                "(routing layers {} and {}).",
                cut_layer_name,
                interface.bottom_layer,
                interface.upper_layer);

  NetRouteMap routes;
  odb::dbTechLayer* max_layer
      = db_->getTech()->findRoutingLayer(max_routing_layer);
  for (Net* net : nets) {
    if (net->getNumPins() <= 1
        || (net->hasWires() && !net->hasStackedVias(max_layer))) {
      continue;
    }

    GRoute route = routeNet(net, interface);
    if (!route.empty()) {
      statistics_.routed_nets++;
      statistics_.segment_count += route.size();
      routes.emplace(net->getDbNet(), std::move(route));
    }
  }

  logger_->info(GRT,
                322,
                "Basic true-3D routing finished: {} routed nets, {} die-local "
                "nets, {} cross-die nets, {} HBTs, {} segments.",
                statistics_.routed_nets,
                statistics_.die_local_nets,
                statistics_.cross_die_nets,
                statistics_.hbt_count,
                statistics_.segment_count);
  return routes;
}

True3DRouter::Interface True3DRouter::makeInterface(
    const std::string& cut_layer_name,
    const int min_routing_layer,
    const int max_routing_layer) const
{
  odb::dbTechLayer* cut_layer
      = db_->getTech()->findLayer(cut_layer_name.c_str());
  if (cut_layer == nullptr) {
    logger_->error(
        GRT, 323, "True-3D cut layer {} was not found.", cut_layer_name);
  }
  if (cut_layer->getType() != odb::dbTechLayerType::CUT) {
    logger_->error(GRT,
                   324,
                   "True-3D interface layer {} is not a cut layer.",
                   cut_layer_name);
  }

  odb::dbTechLayer* bottom_layer = cut_layer->getLowerLayer();
  odb::dbTechLayer* upper_layer = cut_layer->getUpperLayer();
  while (bottom_layer != nullptr && bottom_layer->getRoutingLevel() == 0) {
    bottom_layer = bottom_layer->getLowerLayer();
  }
  while (upper_layer != nullptr && upper_layer->getRoutingLevel() == 0) {
    upper_layer = upper_layer->getUpperLayer();
  }
  if (bottom_layer == nullptr || upper_layer == nullptr) {
    logger_->error(GRT,
                   325,
                   "True-3D cut layer {} does not have routing layers on both "
                   "sides.",
                   cut_layer_name);
  }

  const int bottom_level = bottom_layer->getRoutingLevel();
  const int upper_level = upper_layer->getRoutingLevel();
  if (upper_level != bottom_level + 1) {
    logger_->error(GRT,
                   326,
                   "True-3D cut layer {} must separate adjacent routing "
                   "levels, but found levels {} and {}.",
                   cut_layer_name,
                   bottom_level,
                   upper_level);
  }
  if (min_routing_layer > bottom_level
      || max_routing_layer < upper_level) {
    logger_->error(GRT,
                   327,
                   "Routing range {}-{} does not span true-3D cut layer {} "
                   "between routing levels {} and {}.",
                   min_routing_layer,
                   max_routing_layer,
                   cut_layer_name,
                   bottom_level,
                   upper_level);
  }

  Interface interface;
  interface.bottom_layer = bottom_level;
  interface.upper_layer = upper_level;
  interface.bottom
      = makeDieStack(min_routing_layer, bottom_level, true);
  interface.upper
      = makeDieStack(upper_level, max_routing_layer, false);
  return interface;
}

True3DRouter::DieStack True3DRouter::makeDieStack(
    const int min_layer,
    const int max_layer,
    const bool search_from_top) const
{
  DieStack stack{min_layer, max_layer, -1, -1};
  const int begin = search_from_top ? max_layer : min_layer;
  const int end = search_from_top ? min_layer : max_layer;
  const int step = search_from_top ? -1 : 1;
  for (int layer = begin;; layer += step) {
    odb::dbTechLayer* tech_layer
        = db_->getTech()->findRoutingLayer(layer);
    if (tech_layer->getDirection() == odb::dbTechLayerDir::HORIZONTAL
        && stack.horizontal_layer == -1) {
      stack.horizontal_layer = layer;
    }
    if (tech_layer->getDirection() == odb::dbTechLayerDir::VERTICAL
        && stack.vertical_layer == -1) {
      stack.vertical_layer = layer;
    }
    if (layer == end) {
      break;
    }
  }

  // A one-layer stack is still useful for a basic route. Detailed routing is
  // allowed to use non-preferred direction inside its guide in that case.
  if (stack.horizontal_layer == -1) {
    stack.horizontal_layer = begin;
  }
  if (stack.vertical_layer == -1) {
    stack.vertical_layer = begin;
  }
  return stack;
}

GRoute True3DRouter::routeNet(Net* net, const Interface& interface)
{
  GRoute route;
  SegmentSet segments;
  std::vector<RouteNode> bottom_nodes;
  std::vector<RouteNode> upper_nodes;
  std::vector<RouteNode> all_terminals;

  for (Pin& pin : net->getPins()) {
    const odb::Point& position = pin.getOnGridPosition();
    const int pin_layer = pin.getConnectionLayer();
    const bool on_bottom = pin_layer <= interface.bottom_layer;
    const DieStack& stack = on_bottom ? interface.bottom : interface.upper;
    const int access_layer
        = std::clamp(pin_layer, stack.min_layer, stack.max_layer);
    RouteNode terminal{position.x(), position.y(), access_layer};

    std::vector<RouteNode>& die_nodes
        = on_bottom ? bottom_nodes : upper_nodes;
    if (pin.isDriver()) {
      die_nodes.insert(die_nodes.begin(), terminal);
    } else {
      die_nodes.push_back(terminal);
    }
    all_terminals.push_back(terminal);
    addViaStack(position.x(),
                position.y(),
                pin_layer,
                access_layer,
                route,
                segments);
  }

  const bool is_cross_die
      = !bottom_nodes.empty() && !upper_nodes.empty();
  if (is_cross_die) {
    const odb::Point hbt = selectHbt(all_terminals);
    bottom_nodes.push_back(
        {hbt.x(), hbt.y(), interface.bottom_layer});
    upper_nodes.push_back({hbt.x(), hbt.y(), interface.upper_layer});
    addViaStack(hbt.x(),
                hbt.y(),
                interface.bottom_layer,
                interface.upper_layer,
                route,
                segments);
    statistics_.cross_die_nets++;
    statistics_.hbt_count++;
  } else {
    statistics_.die_local_nets++;
  }

  routeLocalTree(bottom_nodes, interface.bottom, route, segments);
  routeLocalTree(upper_nodes, interface.upper, route, segments);
  return route;
}

void True3DRouter::routeLocalTree(const std::vector<RouteNode>& nodes,
                                  const DieStack& stack,
                                  GRoute& route,
                                  SegmentSet& segments) const
{
  if (nodes.size() < 2) {
    return;
  }

  std::vector<bool> connected(nodes.size(), false);
  connected[0] = true;
  for (std::size_t connected_count = 1; connected_count < nodes.size();
       connected_count++) {
    int64_t best_cost = std::numeric_limits<int64_t>::max();
    std::size_t best_source = 0;
    std::size_t best_target = 0;
    for (std::size_t source = 0; source < nodes.size(); source++) {
      if (!connected[source]) {
        continue;
      }
      for (std::size_t target = 0; target < nodes.size(); target++) {
        if (connected[target]) {
          continue;
        }
        const int64_t cost
            = static_cast<int64_t>(std::abs(nodes[source].x - nodes[target].x))
              + std::abs(nodes[source].y - nodes[target].y)
              + static_cast<int64_t>(grid_->getTileSize())
                    * std::abs(nodes[source].layer - nodes[target].layer);
        if (cost < best_cost) {
          best_cost = cost;
          best_source = source;
          best_target = target;
        }
      }
    }
    routePair(
        nodes[best_source], nodes[best_target], stack, route, segments);
    connected[best_target] = true;
  }
}

void True3DRouter::routePair(const RouteNode& source,
                             const RouteNode& target,
                             const DieStack& stack,
                             GRoute& route,
                             SegmentSet& segments) const
{
  const bool has_horizontal = source.x != target.x;
  const bool has_vertical = source.y != target.y;
  if (!has_horizontal && !has_vertical) {
    addViaStack(source.x,
                source.y,
                source.layer,
                target.layer,
                route,
                segments);
    return;
  }

  if (!has_vertical) {
    addViaStack(source.x,
                source.y,
                source.layer,
                stack.horizontal_layer,
                route,
                segments);
    addWire(source.x,
            source.y,
            target.x,
            target.y,
            stack.horizontal_layer,
            route,
            segments);
    addViaStack(target.x,
                target.y,
                stack.horizontal_layer,
                target.layer,
                route,
                segments);
    return;
  }
  if (!has_horizontal) {
    addViaStack(source.x,
                source.y,
                source.layer,
                stack.vertical_layer,
                route,
                segments);
    addWire(source.x,
            source.y,
            target.x,
            target.y,
            stack.vertical_layer,
            route,
            segments);
    addViaStack(target.x,
                target.y,
                stack.vertical_layer,
                target.layer,
                route,
                segments);
    return;
  }

  const int hv_vias = std::abs(source.layer - stack.horizontal_layer)
                      + std::abs(stack.horizontal_layer - stack.vertical_layer)
                      + std::abs(stack.vertical_layer - target.layer);
  const int vh_vias = std::abs(source.layer - stack.vertical_layer)
                      + std::abs(stack.vertical_layer - stack.horizontal_layer)
                      + std::abs(stack.horizontal_layer - target.layer);
  if (hv_vias <= vh_vias) {
    const int bend_x = target.x;
    const int bend_y = source.y;
    addViaStack(source.x,
                source.y,
                source.layer,
                stack.horizontal_layer,
                route,
                segments);
    addWire(source.x,
            source.y,
            bend_x,
            bend_y,
            stack.horizontal_layer,
            route,
            segments);
    addViaStack(bend_x,
                bend_y,
                stack.horizontal_layer,
                stack.vertical_layer,
                route,
                segments);
    addWire(bend_x,
            bend_y,
            target.x,
            target.y,
            stack.vertical_layer,
            route,
            segments);
    addViaStack(target.x,
                target.y,
                stack.vertical_layer,
                target.layer,
                route,
                segments);
  } else {
    const int bend_x = source.x;
    const int bend_y = target.y;
    addViaStack(source.x,
                source.y,
                source.layer,
                stack.vertical_layer,
                route,
                segments);
    addWire(source.x,
            source.y,
            bend_x,
            bend_y,
            stack.vertical_layer,
            route,
            segments);
    addViaStack(bend_x,
                bend_y,
                stack.vertical_layer,
                stack.horizontal_layer,
                route,
                segments);
    addWire(bend_x,
            bend_y,
            target.x,
            target.y,
            stack.horizontal_layer,
            route,
            segments);
    addViaStack(target.x,
                target.y,
                stack.horizontal_layer,
                target.layer,
                route,
                segments);
  }
}

void True3DRouter::addWire(const int x0,
                           const int y0,
                           const int x1,
                           const int y1,
                           const int layer,
                           GRoute& route,
                           SegmentSet& segments) const
{
  if (x0 == x1 && y0 == y1) {
    return;
  }

  const int tile_size = grid_->getTileSize();
  if (y0 == y1) {
    const auto [begin, end] = std::minmax(x0, x1);
    for (int x = begin; x < end;) {
      const int next_x = std::min(x + tile_size, end);
      addSegment(
          GSegment(x, y0, layer, next_x, y0, layer), route, segments);
      x = next_x;
    }
    return;
  }
  if (x0 == x1) {
    const auto [begin, end] = std::minmax(y0, y1);
    for (int y = begin; y < end;) {
      const int next_y = std::min(y + tile_size, end);
      addSegment(
          GSegment(x0, y, layer, x0, next_y, layer), route, segments);
      y = next_y;
    }
  }
}

void True3DRouter::addViaStack(const int x,
                               const int y,
                               const int layer0,
                               const int layer1,
                               GRoute& route,
                               SegmentSet& segments) const
{
  const auto [bottom, top] = std::minmax(layer0, layer1);
  for (int layer = bottom; layer < top; layer++) {
    addSegment(
        GSegment(x, y, layer, x, y, layer + 1), route, segments);
  }
}

void True3DRouter::addSegment(const GSegment& segment,
                              GRoute& route,
                              SegmentSet& segments) const
{
  if (segments.insert(segment).second) {
    GSegment true_3d_segment = segment;
    true_3d_segment.setIs3DRoute(true);
    route.push_back(true_3d_segment);
  }
}

odb::Point True3DRouter::selectHbt(
    const std::vector<RouteNode>& terminals) const
{
  std::vector<int> x_coordinates;
  std::vector<int> y_coordinates;
  x_coordinates.reserve(terminals.size());
  y_coordinates.reserve(terminals.size());
  for (const RouteNode& terminal : terminals) {
    x_coordinates.push_back(terminal.x);
    y_coordinates.push_back(terminal.y);
  }
  const std::size_t median = terminals.size() / 2;
  std::nth_element(x_coordinates.begin(),
                   x_coordinates.begin() + median,
                   x_coordinates.end());
  std::nth_element(y_coordinates.begin(),
                   y_coordinates.begin() + median,
                   y_coordinates.end());
  return {x_coordinates[median], y_coordinates[median]};
}

}  // namespace grt
