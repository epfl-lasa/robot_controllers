# All headers for installation
set(high_level_controllers_HEADERS
    LinearDS.hpp
)

# Linear DS plugin
corrade_add_plugin(LinearDSController ${CMAKE_CURRENT_BINARY_DIR} "" Dummy.conf LinearDS.cpp)

# Includes
target_include_directories(LinearDSController PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

# Require C++11
set_property(TARGET LinearDSController PROPERTY CXX_STANDARD 11)
set_property(TARGET LinearDSController PROPERTY CXX_STANDARD_REQUIRED ON)

install(TARGETS LinearDSController
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR})

install(FILES ${high_level_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR}/high)