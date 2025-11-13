ament_add_gtest(test_point2d
  test_point2d.cpp
)
target_link_libraries(test_point2d
  ${geometry_msgs_TARGETS}
  ${library_name}
)

ament_add_gtest(test_polygon
  test_polygon.cpp
)
target_link_libraries(test_polygon
  ${geometry_msgs_TARGETS}
  ${library_name}
)

ament_add_gtest(test_segment2d
  test_segment2d.cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

target_link_libraries(test_segment2d
  ${geometry_msgs_TARGETS}
  ${library_name}
  "${cpp_typesupport_target}"
)