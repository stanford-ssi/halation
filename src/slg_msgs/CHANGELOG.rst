^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package slg_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.9.2 (06-02-2025)
------------------
* First jazzy release.

3.9.1 (31-07-2024)
------------------
* Update to use modern CMake idioms.

3.9.0 (29-02-2024)
------------------
* Improve formatting.
* Added testing.

3.8.0 (25-04-2023)
------------------
* Prepare for humble release.

3.7.0 (13-01-2023)
------------------
* Added get_points function.
* Fix add point.

3.6.0 (11-01-2023)
------------------
* A polygon can be created from a vector of points.
* Added checks for closed polygon.

3.5.1 (15-11-2022)
------------------
* Fix install in CMakeLists.txt
* Fix slg_msgs in segment2D.hpp

3.5.0 (17-06-2022)
------------------
* Delete unused SegmentStamped message.
* Added implicit conversions.
* Update syntax.
* Remove old conversions.

3.2.1 (10-01-2022)
------------------
* Added copy constructor in segment2D.
* Fix some linter errors.

3.2.0 (10-01-2022)
------------------
* Added ability to create NaN point.

3.1.0 (16-12-2021)
------------------
* Added string enums.

3.0.0 (15-12-2021)
------------------
* Using slg namespace for Point2D, Segment2D and polygon.

2.1.0 (15-12-2021)
----------------
* Adding conversion for polygon to geometry_msgs.

2.0.0 (13-12-2021)
------------------
* Rename internal message field to comply with ROS standard.

1.1.0 (13-12-2021)
------------------
* Adding conversion for Point2D to geometry_msgs.

1.0.3 (21-07-2021)
------------------
* Now using slg namespace.

1.0.2 (19-07-2021)
------------------
* Added segment2D to PCL conversion.

1.0.0 (16-07-2021)
------------------
* Initial release.
* Create README.md.
* Create CHANGELOG.rst.
* Create LICENSE.
* Added Point2D, Segment2D and Polygon classes.
* Added Segment.msg, SegmentStamped.msg and SegmentArray.msg messages.
* Added conversions (.h and .cpp files).
* Contributors: Alberto Tudela
