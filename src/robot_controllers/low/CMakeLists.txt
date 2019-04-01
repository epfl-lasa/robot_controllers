# All headers for installation
set(low_level_controllers_HEADERS
    PassiveDS.hpp
    Pid.hpp
)

# PID plugin
corrade_add_plugin(PidController ${CMAKE_CURRENT_BINARY_DIR} "" Dummy.conf Pid.cpp ${robot_controllers_utils_SOURCES})

# Includes
target_include_directories(PidController PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${Eigen3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

# Require C++11
set_property(TARGET PidController PROPERTY CXX_STANDARD 11)
set_property(TARGET PidController PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(PidController PUBLIC RobotControllers)

# Add SIMD
add_simd(PidController)

install(TARGETS PidController
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR})

# PassiveDS plugin
corrade_add_plugin(PassiveDSController ${CMAKE_CURRENT_BINARY_DIR} "" Dummy.conf PassiveDS.cpp ${robot_controllers_utils_SOURCES})

# Includes
target_include_directories(PassiveDSController PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${Eigen3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

# Require C++11
set_property(TARGET PassiveDSController PROPERTY CXX_STANDARD 11)
set_property(TARGET PassiveDSController PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(PassiveDSController PUBLIC RobotControllers)

# Add SIMD
add_simd(PassiveDSController)

install(TARGETS PassiveDSController
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR})

install(FILES ${low_level_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR}/low)