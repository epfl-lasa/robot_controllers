# All headers for installation
set(low_level_controllers_HEADERS
    PassiveDS.hpp
    Pid.hpp
)

# PID plugin
corrade_add_plugin(PidController ${CMAKE_CURRENT_BINARY_DIR} "" Dummy.conf Pid.cpp)

# Includes
target_include_directories(PidController PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

# Require C++11
set_property(TARGET PidController PROPERTY CXX_STANDARD 11)
set_property(TARGET PidController PROPERTY CXX_STANDARD_REQUIRED ON)

install(TARGETS PidController
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR})

# PassiveDS plugin
corrade_add_plugin(PassiveDSController ${CMAKE_CURRENT_BINARY_DIR} "" Dummy.conf PassiveDS.cpp)

# Includes
target_include_directories(PassiveDSController PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

# Require C++11
set_property(TARGET PassiveDSController PROPERTY CXX_STANDARD 11)
set_property(TARGET PassiveDSController PROPERTY CXX_STANDARD_REQUIRED ON)

install(TARGETS PassiveDSController
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR})

install(FILES ${low_level_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR}/low)