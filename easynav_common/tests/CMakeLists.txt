ament_add_gtest(perceptions_tests perceptions_tests.cpp)
target_link_libraries(perceptions_tests
  ${PROJECT_NAME}
)

ament_add_gtest(pointperceptionsops_tests pointperceptionsops_tests.cpp)
target_link_libraries(pointperceptionsops_tests
  ${PROJECT_NAME}
)

ament_add_gtest(navstate_tests navstate_tests.cpp)
target_link_libraries(navstate_tests
  ${PROJECT_NAME}
)
