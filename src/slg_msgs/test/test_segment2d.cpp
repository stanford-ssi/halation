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
#include "slg_msgs/segment2D.hpp"

// Default constructor, constructor with values and copy constructor
TEST(Segment2DTest, constructors) {
  // Default constructor
  slg::Segment2D segment1;
  EXPECT_EQ(segment1.get_id(), 0);
  EXPECT_EQ(segment1.get_label(), slg::BACKGROUND);
  EXPECT_DOUBLE_EQ(segment1.get_angular_distance_to_closest_boundary(), 0.0);
  EXPECT_DOUBLE_EQ(segment1.get_prior_segment().x, 0.0);
  EXPECT_DOUBLE_EQ(segment1.get_prior_segment().y, 0.0);
  EXPECT_DOUBLE_EQ(segment1.get_next_segment().x, 0.0);
  EXPECT_DOUBLE_EQ(segment1.get_next_segment().y, 0.0);
  EXPECT_DOUBLE_EQ(segment1.get_last_centroid().x, 0.0);
  EXPECT_DOUBLE_EQ(segment1.get_last_centroid().y, 0.0);
  // Constructor with values
  slg::Segment2D segment2(1,
    slg::Point2D(1.0, 2.0, slg::PERSON),
    slg::Point2D(3.0, 4.0, slg::PERSON),
    slg::Point2D(5.0, 6.0, slg::PERSON));
  EXPECT_EQ(segment2.get_id(), 1);
  EXPECT_EQ(segment2.get_label(), slg::BACKGROUND);
  EXPECT_DOUBLE_EQ(segment2.get_angular_distance_to_closest_boundary(), 0.0);
  EXPECT_DOUBLE_EQ(segment2.get_points()[0].x, 3.0);
  EXPECT_DOUBLE_EQ(segment2.get_points()[0].y, 4.0);
  EXPECT_DOUBLE_EQ(segment2.get_prior_segment().x, 1.0);
  EXPECT_DOUBLE_EQ(segment2.get_prior_segment().y, 2.0);
  EXPECT_DOUBLE_EQ(segment2.get_next_segment().x, 5.0);
  EXPECT_DOUBLE_EQ(segment2.get_next_segment().y, 6.0);
  EXPECT_DOUBLE_EQ(segment2.get_last_centroid().x, 3.0);
  EXPECT_DOUBLE_EQ(segment2.get_last_centroid().y, 4.0);
  // Copy constructor
  slg::Segment2D segment3(segment2);
  EXPECT_EQ(segment3.get_id(), 1);
  EXPECT_EQ(segment3.get_label(), slg::BACKGROUND);
  EXPECT_DOUBLE_EQ(segment3.get_angular_distance_to_closest_boundary(), 0.0);
  EXPECT_DOUBLE_EQ(segment3.get_points()[0].x, 3.0);
  EXPECT_DOUBLE_EQ(segment3.get_points()[0].y, 4.0);
  EXPECT_DOUBLE_EQ(segment3.get_prior_segment().x, 1.0);
  EXPECT_DOUBLE_EQ(segment3.get_prior_segment().y, 2.0);
  EXPECT_DOUBLE_EQ(segment3.get_next_segment().x, 5.0);
  EXPECT_DOUBLE_EQ(segment3.get_next_segment().y, 6.0);
  EXPECT_DOUBLE_EQ(segment3.get_last_centroid().x, 3.0);
  EXPECT_DOUBLE_EQ(segment3.get_last_centroid().y, 4.0);
  // Constructor with sets functions
  slg::Segment2D segment4;
  segment4.set_id(1);
  segment4.set_label(slg::PERSON);
  segment4.set_prior_segment(slg::Point2D(7.0, 8.0, slg::PERSON));
  segment4.set_next_segment(slg::Point2D(9.0, 10.0, slg::PERSON));
  segment4.set_last_centroid(slg::Point2D(11.0, 12.0, slg::PERSON));
  segment4.add_point(slg::Point2D(11.0, 12.0, slg::PERSON));
  segment4.set_angular_distance_to_closest_boundary(0.5);
  EXPECT_EQ(segment4.get_id(), 1);
  EXPECT_EQ(segment4.get_label(), slg::PERSON);
  EXPECT_DOUBLE_EQ(segment4.get_angular_distance_to_closest_boundary(), 0.5);
  EXPECT_DOUBLE_EQ(segment4.get_points()[0].x, 11.0);
  EXPECT_DOUBLE_EQ(segment4.get_points()[0].y, 12.0);
  EXPECT_DOUBLE_EQ(segment4.get_prior_segment().x, 7.0);
  EXPECT_DOUBLE_EQ(segment4.get_prior_segment().y, 8.0);
  EXPECT_DOUBLE_EQ(segment4.get_next_segment().x, 9.0);
  EXPECT_DOUBLE_EQ(segment4.get_next_segment().y, 10.0);
  EXPECT_DOUBLE_EQ(segment4.get_last_centroid().x, 11.0);
  EXPECT_DOUBLE_EQ(segment4.get_last_centroid().y, 12.0);
  // Constructor from slg_msgs::msg::Segment
  slg_msgs::msg::Segment segmentMsg;
  segmentMsg.id = 2;
  segmentMsg.label = slg::PERSON;
  segmentMsg.angular_distance = 0.5;
  segmentMsg.last_point_prior_segment.x = 7.0;
  segmentMsg.last_point_prior_segment.y = 8.0;
  segmentMsg.first_point_next_segment.x = 9.0;
  segmentMsg.first_point_next_segment.y = 10.0;
  segmentMsg.points.push_back(slg::Point2D(11.0, 12.0, slg::PERSON));
  slg::Segment2D segment5(segmentMsg);
  EXPECT_EQ(segment5.get_id(), 2);
  EXPECT_EQ(segment5.get_label(), slg::PERSON);
  EXPECT_DOUBLE_EQ(segment5.get_angular_distance_to_closest_boundary(), 0.5);
  EXPECT_DOUBLE_EQ(segment5.get_points()[0].x, 11.0);
  EXPECT_DOUBLE_EQ(segment5.get_points()[0].y, 12.0);
  EXPECT_DOUBLE_EQ(segment5.get_prior_segment().x, 7.0);
  EXPECT_DOUBLE_EQ(segment5.get_prior_segment().y, 8.0);
  EXPECT_DOUBLE_EQ(segment5.get_next_segment().x, 9.0);
  EXPECT_DOUBLE_EQ(segment5.get_next_segment().y, 10.0);
  EXPECT_DOUBLE_EQ(segment5.get_last_centroid().x, 11.0);
  EXPECT_DOUBLE_EQ(segment5.get_last_centroid().y, 12.0);
}

// Check the dimensions of the segment
TEST(Segment2DTest, dimensions) {
  // Check the length of the segment
  slg::Segment2D segment;
  segment.add_point(slg::Point2D(0.0, 0.0));
  segment.add_point(slg::Point2D(3.0, 4.0));
  EXPECT_DOUBLE_EQ(segment.size(), 2);
  // Check the width of the segment
  EXPECT_DOUBLE_EQ(segment.width(), 5.0);
  // Check the width squared of the segment
  EXPECT_DOUBLE_EQ(segment.width_squared(), 25.0);
  // Check the first point of the segment
  EXPECT_DOUBLE_EQ(segment.first_point().x, 0.0);
  EXPECT_DOUBLE_EQ(segment.first_point().y, 0.0);
  // Check the last point of the segment
  EXPECT_DOUBLE_EQ(segment.last_point().x, 3.0);
  EXPECT_DOUBLE_EQ(segment.last_point().y, 4.0);
  // Check the vector of the segment
  EXPECT_DOUBLE_EQ(segment.vector().x, 3.0);
  EXPECT_DOUBLE_EQ(segment.vector().y, 4.0);
  // Check the minimum angle of the segment
  EXPECT_DOUBLE_EQ(segment.min_angle(), 0.0);
  // Check the maximum angle of the segment
  EXPECT_DOUBLE_EQ(segment.max_angle(), 0.92729521800161219);
  // Check the mean angle of the segment
  EXPECT_DOUBLE_EQ(segment.mean_angle(), 0.46364760900080609);
  // Check the orientation of the segment
  EXPECT_DOUBLE_EQ(segment.orientation(), -2.4980915447965089);
  // Check the orientation of the segment when the vector is horizontal
  slg::Segment2D segment2;
  segment2.add_point(slg::Point2D(0.0, 0.0));
  segment2.add_point(slg::Point2D(2.0, 0.0));
  EXPECT_DOUBLE_EQ(segment2.orientation(), 0.0);
  // Projection of the segment
  slg::Point2D point(1.0, 1.0);
  slg::Point2D projection = segment.projection(point);
  EXPECT_DOUBLE_EQ(projection.x, 0.84);
  EXPECT_DOUBLE_EQ(projection.y, 1.12);
  // Check the distance to point
  EXPECT_DOUBLE_EQ(segment.distance_to(point), 0.2);
  // Check the centroid of the segment
  slg::Point2D centroid = segment.centroid();
  EXPECT_DOUBLE_EQ(centroid.x, 1.5);
  EXPECT_DOUBLE_EQ(centroid.y, 2.0);
  // Check the height
  EXPECT_DOUBLE_EQ(segment.height(), 0.0);
  // Check if the segment is empty
  EXPECT_FALSE(segment.empty());
  // Clear the segment
  segment.clear();
  EXPECT_EQ(segment.size(), 0);
}

// Check splitting the segment
TEST(Segment2DTest, mergeAndSplitting) {
  // Merge two segments
  slg::Segment2D segment1;
  segment1.add_point(slg::Point2D(0.0, 0.0));
  segment1.add_point(slg::Point2D(3.0, 4.0));
  segment1.set_label(slg::PERSON);
  slg::Segment2D segment2;
  segment2.add_point(slg::Point2D(5.0, 6.0));
  segment2.add_point(slg::Point2D(7.0, 8.0));
  segment2.set_label(slg::PERSON);
  segment1.merge(segment2);
  EXPECT_DOUBLE_EQ(segment1.size(), 4);
  EXPECT_DOUBLE_EQ(segment1.get_points()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(segment1.get_points()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(segment1.get_points()[1].x, 3.0);
  EXPECT_DOUBLE_EQ(segment1.get_points()[1].y, 4.0);
  EXPECT_DOUBLE_EQ(segment1.get_points()[2].x, 5.0);
  EXPECT_DOUBLE_EQ(segment1.get_points()[2].y, 6.0);
  EXPECT_DOUBLE_EQ(segment1.get_points()[3].x, 7.0);
  EXPECT_DOUBLE_EQ(segment1.get_points()[3].y, 8.0);
  // Merge two segments with different labels
  slg::Segment2D segment3;
  segment3.add_point(slg::Point2D(0.0, 0.0));
  segment3.add_point(slg::Point2D(3.0, 4.0));
  segment3.set_label(slg::PERSON);
  slg::Segment2D segment4;
  segment4.add_point(slg::Point2D(5.0, 6.0));
  segment4.add_point(slg::Point2D(7.0, 8.0));
  segment4.set_label(slg::BACKGROUND);
  segment3.merge(segment4);
  EXPECT_DOUBLE_EQ(segment3.size(), 4);
  EXPECT_EQ(segment3.get_label(), slg::BACKGROUND);
  // Split the segment
  slg::Segment2D left_segment = segment1.left_split(2);
  EXPECT_DOUBLE_EQ(left_segment.size(), 2);
  EXPECT_DOUBLE_EQ(left_segment.get_points()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(left_segment.get_points()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(left_segment.get_points()[1].x, 3.0);
  EXPECT_DOUBLE_EQ(left_segment.get_points()[1].y, 4.0);
  slg::Segment2D right_segment = segment1.right_split(2);
  EXPECT_DOUBLE_EQ(right_segment.size(), 2);
  EXPECT_DOUBLE_EQ(right_segment.get_points()[0].x, 5.0);
  EXPECT_DOUBLE_EQ(right_segment.get_points()[0].y, 6.0);
  EXPECT_DOUBLE_EQ(right_segment.get_points()[1].x, 7.0);
  EXPECT_DOUBLE_EQ(right_segment.get_points()[1].y, 8.0);
}

// Implicit conversion to slg_msgs::msg::Segment
TEST(Segment2DTest, implicitConversionToSlgMsgsSegment) {
  slg::Segment2D segment;
  segment.add_point(slg::Point2D(1.0, 2.0, slg::PERSON));
  segment.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  segment.add_point(slg::Point2D(5.0, 6.0, slg::PERSON));
  segment.set_prior_segment(slg::Point2D(7.0, 8.0, slg::PERSON));
  segment.set_next_segment(slg::Point2D(9.0, 10.0, slg::PERSON));
  slg_msgs::msg::Segment segment_msg = segment;
  EXPECT_EQ(segment_msg.id, 0);
  EXPECT_EQ(segment_msg.label, slg::BACKGROUND);
  EXPECT_DOUBLE_EQ(segment_msg.angular_distance, 0.0);
  EXPECT_DOUBLE_EQ(segment_msg.last_point_prior_segment.x, 7.0);
  EXPECT_DOUBLE_EQ(segment_msg.last_point_prior_segment.y, 8.0);
  EXPECT_DOUBLE_EQ(segment_msg.first_point_next_segment.x, 9.0);
  EXPECT_DOUBLE_EQ(segment_msg.first_point_next_segment.y, 10.0);
  EXPECT_EQ(segment_msg.points.size(), 3);
  EXPECT_DOUBLE_EQ(segment_msg.points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(segment_msg.points[0].y, 2.0);
  EXPECT_DOUBLE_EQ(segment_msg.points[1].x, 3.0);
  EXPECT_DOUBLE_EQ(segment_msg.points[1].y, 4.0);
  EXPECT_DOUBLE_EQ(segment_msg.points[2].x, 5.0);
  EXPECT_DOUBLE_EQ(segment_msg.points[2].y, 6.0);
}

// Check the assignment operators
TEST(Segment2DTest, assignmentOperators) {
  // Check the operator assignment
  slg::Segment2D segment;
  segment.add_point(slg::Point2D(1.0, 2.0, slg::PERSON));
  segment.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  segment.add_point(slg::Point2D(5.0, 6.0, slg::PERSON));
  slg::Segment2D segment2 = segment;
  EXPECT_TRUE(segment == segment2);
  segment2.add_point(slg::Point2D(7.0, 8.0, slg::PERSON));
  EXPECT_FALSE(segment == segment2);
  // Check the operator assignment when the segments are the same
  segment2 = segment2;
  EXPECT_FALSE(segment == segment2);
  EXPECT_TRUE(segment2 == segment2);
  // Assignment operator for the slg_msgs::msg::Segment
  slg::Segment2D segment3;
  slg_msgs::msg::Segment segment_msg;
  segment_msg.id = 1;
  segment_msg.label = slg::PERSON;
  segment_msg.angular_distance = 0.5;
  segment_msg.last_point_prior_segment.x = 7.0;
  segment_msg.last_point_prior_segment.y = 8.0;
  segment_msg.first_point_next_segment.x = 9.0;
  segment_msg.first_point_next_segment.y = 10.0;
  segment_msg.points.push_back(slg::Point2D(11.0, 12.0, slg::PERSON));
  segment3 = segment_msg;
  EXPECT_EQ(segment3.get_id(), 1);
  EXPECT_EQ(segment3.get_label(), slg::PERSON);
  EXPECT_DOUBLE_EQ(segment3.get_angular_distance_to_closest_boundary(), 0);
  EXPECT_DOUBLE_EQ(segment3.get_points()[0].x, 11.0);
  EXPECT_DOUBLE_EQ(segment3.get_points()[0].y, 12.0);
  EXPECT_DOUBLE_EQ(segment3.get_prior_segment().x, 7.0);
  EXPECT_DOUBLE_EQ(segment3.get_prior_segment().y, 8.0);
  EXPECT_DOUBLE_EQ(segment3.get_next_segment().x, 9.0);
  EXPECT_DOUBLE_EQ(segment3.get_next_segment().y, 10.0);
  EXPECT_DOUBLE_EQ(segment3.get_last_centroid().x, 11.0);
  EXPECT_DOUBLE_EQ(segment3.get_last_centroid().y, 12.0);
}

// Check comparison operators
TEST(Segment2DTest, comparisonOperators) {
  // Check the operator equal
  slg::Segment2D segment;
  segment.add_point(slg::Point2D(1.0, 2.0, slg::PERSON));
  segment.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  segment.add_point(slg::Point2D(5.0, 6.0, slg::PERSON));
  slg::Segment2D segment2;
  segment2.add_point(slg::Point2D(1.0, 2.0, slg::PERSON));
  segment2.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  segment2.add_point(slg::Point2D(5.0, 6.0, slg::PERSON));
  EXPECT_TRUE(segment == segment2);
  // Check the operator not equal
  slg::Segment2D segment3;
  segment3.add_point(slg::Point2D(1.0, 2.0, slg::PERSON));
  segment3.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  segment3.add_point(slg::Point2D(5.0, 6.0, slg::PERSON));
  slg::Segment2D segment4;
  segment4.add_point(slg::Point2D(1.0, 2.0, slg::PERSON));
  segment4.add_point(slg::Point2D(3.0, 4.0, slg::BACKGROUND));
  segment4.add_point(slg::Point2D(5.0, 6.0, slg::PERSON));
  segment4.add_point(slg::Point2D(7.0, 8.0, slg::PERSON));
  EXPECT_TRUE(segment3 != segment4);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
