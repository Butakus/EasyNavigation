ament_add_gtest(perceptionsops_tests perceptionsops_tests.cpp)
target_link_libraries(perceptionsops_tests
  ${PROJECT_NAME}
)

ament_add_gtest(navstate_tests navstate_tests.cpp)
target_link_libraries(navstate_tests
  ${PROJECT_NAME}
)
