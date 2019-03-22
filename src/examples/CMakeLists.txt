add_executable(linear_test linear_test.cpp)

# Require C++11
set_property(TARGET linear_test PROPERTY CXX_STANDARD 11)
set_property(TARGET linear_test PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(linear_test
  RobotControllers
)