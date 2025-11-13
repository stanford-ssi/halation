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

#ifndef SLG_MSGS__POINT2D_HPP_
#define SLG_MSGS__POINT2D_HPP_

#include <limits>
#include <cmath>
#include <iostream>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace slg
{

// Label
typedef enum {BACKGROUND, PERSON, PERSON_CANE, PERSON_WHEEL_CHAIR} Label;

// Point 2D
struct Point2D
{
  explicit Point2D(double new_x = 0.0, double new_y = 0.0, Label new_label = BACKGROUND)
  : x(new_x), y(new_y), label(new_label) {}
  Point2D(const Point2D & p)
  : x(p.x), y(p.y), label(p.label) {}
  explicit Point2D(const geometry_msgs::msg::Point & p)
  : x(p.x), y(p.y), label(BACKGROUND) {}
  explicit Point2D(const geometry_msgs::msg::Point32 & p)
  : x(p.x), y(p.y), label(BACKGROUND) {}
  ~Point2D() {}

  static Point2D from_polar_coords(const double r, const double phi)
  {
    return Point2D(r * cos(phi), r * sin(phi));
  }
  static Point2D quiet_NaN()
  {
    return Point2D(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN());
  }

  bool isnan()           const {return std::isnan(x) || std::isnan(y);}

  double length()         const {return sqrt(pow(x, 2.0) + pow(y, 2.0));}
  double length_squared() const {return pow(x, 2.0) + pow(y, 2.0);}
  double angle()          const {return atan2(y, x);}
  double angle_deg()      const {return 180.0 * atan2(y, x) / M_PI;}
  double dot(const Point2D & p)   const {return x * p.x + y * p.y;}
  double cross(const Point2D & p) const {return x * p.y - y * p.x;}
  double angle3(const Point2D & p, const Point2D & q) const
  {
    return atan2(q.y - y, q.x - x) - atan2(p.y - y, p.x - x);
  }

  Point2D normalized() {return (length() > 0.0) ? *this / length() : *this;}
  Point2D reflected(const Point2D & normal) const
  {
    return *this - 2.0 * normal * (normal.dot(*this));
  }
  Point2D perpendicular() const {return Point2D(-y, x);}

  friend Point2D operator+(const Point2D & p1, const Point2D & p2)
  {
    return Point2D(p1.x + p2.x, p1.y + p2.y);
  }
  friend Point2D operator-(const Point2D & p1, const Point2D & p2)
  {
    return Point2D(p1.x - p2.x, p1.y - p2.y);
  }
  friend Point2D operator*(const double f, const Point2D & p) {return Point2D(f * p.x, f * p.y);}
  friend Point2D operator*(const Point2D & p, const double f) {return Point2D(f * p.x, f * p.y);}
  friend Point2D operator/(const Point2D & p, const double f)
  {
    return (f != 0.0) ? Point2D(p.x / f, p.y / f) : Point2D::quiet_NaN();
  }

  Point2D operator-() {return Point2D(-x, -y);}
  Point2D operator+() {return Point2D(x, y);}

  operator geometry_msgs::msg::Point() const {
    geometry_msgs::msg::Point pointMsg;
    pointMsg.x = x;
    pointMsg.y = y;
    return pointMsg;
  }

  operator geometry_msgs::msg::Point32() const {
    geometry_msgs::msg::Point32 pointMsg;
    pointMsg.x = x;
    pointMsg.y = y;
    pointMsg.z = 0.0;
    return pointMsg;
  }

  Point2D & operator=(const Point2D & p) {if (this != &p) {x = p.x; y = p.y;} return *this;}
  Point2D & operator+=(const Point2D & p) {x += p.x; y += p.y; return *this;}
  Point2D & operator-=(const Point2D & p) {x -= p.x; y -= p.y; return *this;}

  Point2D & operator=(const geometry_msgs::msg::Point & pointMsg)
  {
    *this = Point2D(pointMsg);
    return *this;
  }

  Point2D & operator=(const geometry_msgs::msg::Point32 & pointMsg)
  {
    *this = Point2D(pointMsg);
    return *this;
  }

  friend bool operator==(const Point2D & p1, const Point2D & p2)
  {
    return p1.x == p2.x && p1.y == p2.y;
  }
  friend bool operator!=(const Point2D & p1, const Point2D & p2) {return !(p1 == p2);}
  friend bool operator<(const Point2D & p1, const Point2D & p2)
  {
    return p1.length_squared() < p2.length_squared();
  }
  friend bool operator<=(const Point2D & p1, const Point2D & p2)
  {
    return p1.length_squared() <= p2.length_squared();
  }
  friend bool operator>(const Point2D & p1, const Point2D & p2)
  {
    return p1.length_squared() > p2.length_squared();
  }
  friend bool operator>=(const Point2D & p1, const Point2D & p2)
  {
    return p1.length_squared() >= p2.length_squared();
  }
  friend bool operator!(const Point2D & p1) {return p1.x == 0.0 && p1.y == 0.0;}

  friend std::ostream & operator<<(std::ostream & out, const Point2D & p)
  {
    out << "(" << p.x << ", " << p.y << ")"; return out;
  }

  double x;
  double y;
  Label label;
};
}  // namespace slg

#endif  // SLG_MSGS__POINT2D_HPP_
