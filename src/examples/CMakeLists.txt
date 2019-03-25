add_executable(linear_test linear_test.cpp)

# Require C++11
set_property(TARGET linear_test PROPERTY CXX_STANDARD 11)
set_property(TARGET linear_test PROPERTY CXX_STANDARD_REQUIRED ON)

# add_library(PassiveDS SHARED IMPORTED)
# set_property(TARGET PassiveDS PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/../robot_controllers/low/PassiveDSController.so)

# Add SIMD
add_simd(linear_test)

target_link_libraries(linear_test
  RobotControllers
  # PassiveDSController
)