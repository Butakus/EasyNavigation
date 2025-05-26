ament_add_gtest(sensors_node_tests sensors_node_tests.cpp)
target_link_libraries(sensors_node_tests
  ${PROJECT_NAME}
  tf2_ros::tf2_ros
  pcl_conversions::pcl_conversions
  ${PCL_LIBRARIES}
)
