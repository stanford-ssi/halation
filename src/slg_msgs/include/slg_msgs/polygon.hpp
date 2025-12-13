// Copyright (c) 2017 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
################################################################
# Ray-casting algorithm
#
# adapted from http://rosettacode.org/wiki/Ray-casting_algorithm
################################################################
*/

#ifndef SLG_MSGS__POLYGON_HPP_
#define SLG_MSGS__POLYGON_HPP_

#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <numeric>

#include "geometry_msgs/msg/polygon.hpp"

#include "point2D.hpp"

const double epsilon = std::numeric_limits<float>().epsilon();
const std::numeric_limits<double> DOUBLE;
const double MIN = DOUBLE.min();
const double MAX = DOUBLE.max();

namespace slg
{
struct Edge
{
  explicit Edge(
    const Point2D & new_a = Point2D(0.0, 0.0),
    const Point2D & new_b = Point2D(0.0, 0.0))
  : a(new_a), b(new_b) {}

  Edge(const Edge & e)
  : a(e.a), b(e.b) {}

  bool operator()(const Point2D & p) const
  {
    if (a.y > b.y) {return Edge(b, a)(p);}
    if (p.y == a.y || p.y == b.y) {return operator()(Point2D(p.x, p.y + epsilon));}
    if (p.y > b.y || p.y < a.y || p.x > std::max(a.x, b.x)) {return false;}
    if (p.x < std::min(a.x, b.x)) {return true;}
    auto blue = std::abs(a.x - p.x) > MIN ? (p.y - a.y) / (p.x - a.x) : MAX;
    auto red = std::abs(a.x - b.x) > MIN ? (b.y - a.y) / (b.x - a.x) : MAX;
    return blue >= red;
  }

  double distance(const Point2D & p)
  {
    Point2D difAB = b - a;
    Point2D difPA = a - p;
    return fabs(difAB.x * difPA.y - difAB.y * difPA.x) / difAB.length();
  }

  bool self() const
  {
    return a == b;
  }

  Edge & operator=(const Edge & e)
  {
    if (this != &e) {
      this->a = e.a;
      this->b = e.b;
    }
    return *this;
  }

  bool operator==(const Edge & e) const
  {
    return (a == e.a && b == e.b) || (a == e.b && b == e.a);
  }

  bool operator!=(const Edge & e) const
  {
    return !(*this == e);
  }

  friend std::ostream & operator<<(std::ostream & out, const Edge & e)
  {
    out << "[" << e.a << " -> " << e.b << "] ";
    return out;
  }

  Point2D a, b;
};

class Polygon
{
public:
  Polygon()
  : name("") {}

  Polygon(const Polygon & poly)
  : name(poly.get_name()),
    edges(poly.get_edges()) {}

  explicit Polygon(const geometry_msgs::msg::Polygon & polygonMsg)
  {
    // Read n-1 points
    for (unsigned int i = 0; i < polygonMsg.points.size() - 1; i++) {
      Point2D a(polygonMsg.points[i]);
      Point2D b(polygonMsg.points[i + 1]);
      edges.push_back(Edge(a, b));
    }
    // Add the last edge
    Point2D a(polygonMsg.points.back());
    Point2D b(polygonMsg.points.front());
    edges.push_back(Edge(a, b));
  }

  ~Polygon() {}

  int size() const {return edges.size();}
  bool empty() const {return edges.empty();}
  void clear() {edges.clear(); name.clear();}
  std::string get_name() const {return name;}
  void set_name(std::string new_name) {name = new_name;}
  std::vector<Edge> get_edges() const {return edges;}
  Edge get_edge(int e) const {return edges[e];}
  void add_edge(Edge edge) {edges.push_back(edge);}

  bool contains(const Point2D & p) const
  {
    auto c = 0;
    for (auto e : edges) {if (e(p)) {c++;}}
    return c % 2 != 0;
  }

  Point2D centroid() const
  {
    std::vector<Point2D> points;
    for (const auto & edge : edges) {
      points.push_back(edge.a);
    }
    Point2D sum = std::accumulate(points.begin(), points.end(), Point2D(0.0, 0.0));
    return sum / points.size();
  }

  bool is_closed() const
  {
    return edges.front().a == edges.back().b;
  }

  void close()
  {
    if (!is_closed()) {
      edges.push_back(Edge(edges.back().b, edges.front().a));
    }
  }

  void add_point(const Point2D & p)
  {
    // If the polygon is empty, add the first edge as a self edge
    if (edges.empty()) {
      edges.push_back(Edge(p, p));
    } else {
      // If the last edge is a self edge, add a new point to it
      if (edges.back().self()) {
        edges.back().b = p;
      } else {
        // If the polygon is closed, remove the last edge
        if (is_closed()) {
          edges.pop_back();
        }
        // Add the new point
        edges.push_back(Edge(edges.back().b, p));
        // Close the polygon
        close();
      }
    }
  }

  std::vector<Point2D> get_points() const
  {
    std::vector<Point2D> points;
    for (const auto & edge : edges) {
      points.push_back(edge.a);
    }
    return points;
  }

  operator geometry_msgs::msg::Polygon()
  {
    geometry_msgs::msg::Polygon polygonMsg;
    for (const auto & edge : edges) {
      polygonMsg.points.push_back(edge.a);
    }
    return polygonMsg;
  }

  Polygon & operator=(const Polygon & poly)
  {
    if (this != &poly) {
      this->name = poly.get_name();
      this->edges = poly.get_edges();
    }
    return *this;
  }

  Polygon & operator=(const geometry_msgs::msg::Polygon & polygonMsg)
  {
    *this = Polygon(polygonMsg);
    return *this;
  }

  bool operator==(const Polygon & poly) const
  {
    return name == poly.get_name() && edges == poly.get_edges();
  }

  bool operator!=(const Polygon & poly) const
  {
    return !(*this == poly);
  }

  friend std::ostream & operator<<(std::ostream & out, const Polygon & poly)
  {
    out << "Polygon: " << poly.get_name() << std::endl;
    for (const auto & edge : poly.get_edges()) {
      out << edge.a << " -> " << edge.b << std::endl;
    }
    return out;
  }

private:
  std::string name;
  std::vector<Edge> edges;
};
}  // namespace slg

#endif  // SLG_MSGS__POLYGON_HPP_
