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

#include "gtest/gtest.h"
#include "geometry_msgs/msg/point.hpp"
#include "slg_msgs/point2D.hpp"

// Default constructor, constructor with values and copy constructor
TEST(Point2DTest, constructors) {
  // Default constructor
  slg::Point2D point1;
  EXPECT_DOUBLE_EQ(point1.x, 0.0);
  EXPECT_DOUBLE_EQ(point1.y, 0.0);
  EXPECT_EQ(point1.label, slg::BACKGROUND);
  // Constructor with values
  slg::Point2D point2(1.0, 2.0, slg::PERSON);
  EXPECT_DOUBLE_EQ(point2.x, 1.0);
  EXPECT_DOUBLE_EQ(point2.y, 2.0);
  EXPECT_EQ(point2.label, slg::PERSON);
  // Copy constructor
  slg::Point2D point3(point2);
  EXPECT_DOUBLE_EQ(point3.x, 1.0);
  EXPECT_DOUBLE_EQ(point3.y, 2.0);
  EXPECT_EQ(point3.label, slg::PERSON);
  // Constructor from geometry_msgs::msg::Point
  geometry_msgs::msg::Point pointMsg;
  pointMsg.x = 3.0;
  pointMsg.y = 4.0;
  slg::Point2D point4(pointMsg);
  EXPECT_DOUBLE_EQ(point4.x, 3.0);
  EXPECT_DOUBLE_EQ(point4.y, 4.0);
  EXPECT_EQ(point4.label, slg::BACKGROUND);
  // Constructor from geometry_msgs::msg::Point32
  geometry_msgs::msg::Point32 pointMsg32;
  pointMsg32.x = 5.0;
  pointMsg32.y = 6.0;
  slg::Point2D point5(pointMsg32);
  EXPECT_DOUBLE_EQ(point5.x, 5.0);
  EXPECT_DOUBLE_EQ(point5.y, 6.0);
  EXPECT_EQ(point5.label, slg::BACKGROUND);
  // From polar coordinates
  slg::Point2D point6 = slg::Point2D::from_polar_coords(1.0, M_PI / 4.0);
  EXPECT_DOUBLE_EQ(point6.x, 1.0 / sqrt(2.0));
  EXPECT_DOUBLE_EQ(point6.y, 1.0 / sqrt(2.0));
}

// NaN and is_NaN
TEST(Point2DTest, NaN) {
  // Not a number
  slg::Point2D point = slg::Point2D(std::numeric_limits<double>::quiet_NaN(), 1.0);
  EXPECT_TRUE(std::isnan(point.x));
  EXPECT_FALSE(std::isnan(point.y));
  // Check if is NaN
  EXPECT_TRUE(point.isnan());

  // Also not a number
  slg::Point2D point2 = slg::Point2D(1.0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(std::isnan(point2.x));
  EXPECT_TRUE(std::isnan(point2.y));
  // Check if is NaN
  EXPECT_TRUE(point2.isnan());

  // Not a number
  slg::Point2D point3 = slg::Point2D::quiet_NaN();
  EXPECT_TRUE(std::isnan(point3.x));
  EXPECT_TRUE(std::isnan(point3.y));
  // Check if is NaN
  EXPECT_TRUE(point3.isnan());

  // This is a number
  slg::Point2D point4 = slg::Point2D(1.0, 1.0);
  EXPECT_FALSE(std::isnan(point4.x));
  EXPECT_FALSE(std::isnan(point4.y));
  // Check if is NaN
  EXPECT_FALSE(point4.isnan());
}

// Check dimensions
TEST(Point2DTest, dimensions) {
  // Check the length
  slg::Point2D point(3.0, 4.0);
  EXPECT_DOUBLE_EQ(point.length(), 5.0);
  // Check the length squared of the point
  EXPECT_DOUBLE_EQ(point.length_squared(), 25.0);
  // Check the angle of the point
  slg::Point2D point2(1.0, 1.0);
  EXPECT_DOUBLE_EQ(point2.angle(), M_PI / 4.0);
  // Check the angle in degrees of the point
  EXPECT_DOUBLE_EQ(point2.angle_deg(), 45.0);
  // Check the dot product of the points
  EXPECT_DOUBLE_EQ(point.dot(point2), 7.0);
  // Check the cross product of the points
  EXPECT_DOUBLE_EQ(point.cross(point2), -1.0);
  // Check the angle3
  slg::Point2D point3(5.0, 6.0);
  EXPECT_DOUBLE_EQ(point.angle3(point2, point3), 2.9441970937399122);
}

// Check the geometry
TEST(Point2DTest, geometry) {
  // Check the point normalized
  slg::Point2D point(3.0, 4.0);
  slg::Point2D point2 = point.normalized();
  EXPECT_DOUBLE_EQ(point2.x, 3.0 / 5.0);
  EXPECT_DOUBLE_EQ(point2.y, 4.0 / 5.0);
  // Check the point is normalized when the length is zero
  slg::Point2D point3 = slg::Point2D(0.0, 0.0).normalized();
  EXPECT_DOUBLE_EQ(point3.x, 0.0);
  EXPECT_DOUBLE_EQ(point3.y, 0.0);
  // Check the point reflected
  slg::Point2D normal(1.0, 0.0);
  slg::Point2D point4 = point.reflected(normal);
  EXPECT_DOUBLE_EQ(point4.x, -3.0);
  EXPECT_DOUBLE_EQ(point4.y, 4.0);
  // Check the point perpendicular
  slg::Point2D point5 = point.perpendicular();
  EXPECT_DOUBLE_EQ(point5.x, -4.0);
  EXPECT_DOUBLE_EQ(point5.y, 3.0);
}

// Check the arithmetic operators
TEST(Point2DTest, arithmeticOperators) {
  // Check the operator addition
  slg::Point2D point(1.0, 1.0);
  slg::Point2D point2(2.0, 2.0);
  slg::Point2D point3 = point + point2;
  EXPECT_DOUBLE_EQ(point3.x, 3.0);
  EXPECT_DOUBLE_EQ(point3.y, 3.0);
  // Check the operator subtraction
  point3 = point - point2;
  EXPECT_DOUBLE_EQ(point3.x, -1.0);
  EXPECT_DOUBLE_EQ(point3.y, -1.0);
  // Check the operator multiplication by a scalar
  point3 = 2.0 * point;
  EXPECT_DOUBLE_EQ(point3.x, 2.0);
  EXPECT_DOUBLE_EQ(point3.y, 2.0);
  // Check the operator multiplication by a scalar
  point3 = point * 2.0;
  EXPECT_DOUBLE_EQ(point3.x, 2.0);
  EXPECT_DOUBLE_EQ(point3.y, 2.0);
  // Check the operator division
  point3 = point / 2.0;
  EXPECT_DOUBLE_EQ(point3.x, 0.5);
  EXPECT_DOUBLE_EQ(point3.y, 0.5);
  // Check the operator division by zero
  point3 = point / 0.0;
  EXPECT_TRUE(std::isnan(point3.x));
  EXPECT_TRUE(std::isnan(point3.y));
  // Check the operator unary minus
  point3 = -point;
  EXPECT_DOUBLE_EQ(point3.x, -1.0);
  EXPECT_DOUBLE_EQ(point3.y, -1.0);
  // Check the operator unary plus
  point3 = +point;
  EXPECT_DOUBLE_EQ(point3.x, 1.0);
}

// Check the implicit conversion to geometry_msgs
TEST(Point2DTest, implicitConversions) {
  // Check the implicit conversion to geometry_msgs::msg::Point
  slg::Point2D point(1.0, 1.0);
  geometry_msgs::msg::Point pointMsg = point;
  EXPECT_DOUBLE_EQ(pointMsg.x, 1.0);
  EXPECT_DOUBLE_EQ(pointMsg.y, 1.0);
  EXPECT_DOUBLE_EQ(pointMsg.z, 0.0);
  // Check the implicit conversion to geometry_msgs::msg::Point32
  geometry_msgs::msg::Point32 pointMsg32 = point;
  EXPECT_DOUBLE_EQ(pointMsg32.x, 1.0);
  EXPECT_DOUBLE_EQ(pointMsg32.y, 1.0);
  EXPECT_DOUBLE_EQ(pointMsg32.z, 0.0);
}

// Check the assignment operators
TEST(Point2DTest, assignmentOperators) {
  // Check the operator assignment
  slg::Point2D point(1.0, 1.0);
  slg::Point2D point2;
  point2 = point;
  EXPECT_DOUBLE_EQ(point2.x, 1.0);
  EXPECT_DOUBLE_EQ(point2.y, 1.0);
  // Check the operator assignment when the points are the same
  point2 = point2;
  EXPECT_DOUBLE_EQ(point2.x, 1.0);
  EXPECT_DOUBLE_EQ(point2.y, 1.0);
  // Check the operator addition assignment
  point2 = slg::Point2D(2.0, 2.0);
  point2 += point;
  EXPECT_DOUBLE_EQ(point2.x, 3.0);
  EXPECT_DOUBLE_EQ(point2.y, 3.0);
  // Check the operator subtraction assignment
  point2 = slg::Point2D(2.0, 2.0);
  point2 -= point;
  EXPECT_DOUBLE_EQ(point2.x, 1.0);
  EXPECT_DOUBLE_EQ(point2.y, 1.0);
  // Check the operator assignment from geometry_msgs::msg::Point
  point2 = geometry_msgs::msg::Point();
  geometry_msgs::msg::Point pointMsg;
  pointMsg.x = 1.0;
  pointMsg.y = 2.0;
  point2 = pointMsg;
  EXPECT_DOUBLE_EQ(point2.x, 1.0);
  EXPECT_DOUBLE_EQ(point2.y, 2.0);
  // Check the operator assignment from geometry_msgs::msg::Point32
  point2 = geometry_msgs::msg::Point32();
  geometry_msgs::msg::Point32 pointMsg32;
  pointMsg32.x = 1.0;
  pointMsg32.y = 2.0;
  point2 = pointMsg32;
  EXPECT_DOUBLE_EQ(point2.x, 1.0);
  EXPECT_DOUBLE_EQ(point2.y, 2.0);
}

// Check the comparison operators
TEST(Point2DTest, comparisonOperators) {
  // Check the operator equal
  slg::Point2D point(1.0, 1.0);
  slg::Point2D point2(1.0, 1.0);
  EXPECT_TRUE(point == point2);
  // Check the operator not equal
  slg::Point2D point3(2.0, 2.0);
  EXPECT_TRUE(point != point3);
  // Check the operator less than
  EXPECT_TRUE(point < point3);
  // Check the operator less than or equal
  EXPECT_TRUE(point <= point2);
  EXPECT_TRUE(point <= point3);
  // Check the operator greater than
  EXPECT_TRUE(point3 > point);
  // Check the operator greater than or equal
  EXPECT_TRUE(point3 >= point2);
  EXPECT_TRUE(point3 >= point);
  // Check the operator not
  slg::Point2D point4(1.0, 0.0);
  EXPECT_FALSE(!point4);
  slg::Point2D point5(0.0, 1.0);
  EXPECT_FALSE(!point5);
  slg::Point2D point6(0.0, 0.0);
  EXPECT_TRUE(!point6);
  // Check the operator output stream
  std::stringstream ss;
  ss << point;
  EXPECT_EQ(ss.str(), "(1, 1)");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
