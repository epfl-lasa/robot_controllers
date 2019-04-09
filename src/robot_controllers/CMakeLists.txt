set(robot_controllers_HEADERS
    AbstractController.hpp
    CascadeController.hpp
    SumController.hpp
    low/PassiveDS.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/Config.hpp
)

set(robot_controllers_SOURCES
    AbstractController.cpp
    CascadeController.cpp
    SumController.cpp
    low/PassiveDS.cpp

)

set(robot_controllers_utils_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/math.hpp
)

set(robot_controllers_utils_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/math.cpp
)

set(ROBOT_CONTROLLERS_PLUGINS_DIR ${CMAKE_INSTALL_PREFIX}/lib/RobotControllersPlugins)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/Config.hpp)

add_library(RobotControllers SHARED ${robot_controllers_SOURCES} ${robot_controllers_utils_SOURCES})

target_include_directories(RobotControllers PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${Eigen3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>
    PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/..)

target_link_libraries(RobotControllers PUBLIC Corrade::PluginManager)

# Require C++11
set_property(TARGET RobotControllers PROPERTY CXX_STANDARD 11)
set_property(TARGET RobotControllers PROPERTY CXX_STANDARD_REQUIRED ON)

# Add SIMD
add_simd(RobotControllers)

install(TARGETS RobotControllers
    RUNTIME DESTINATION ${ROBOT_CONTROLLERS_BINARY_INSTALL_DIR}
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR}
    ARCHIVE DESTINATION ${ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR})

install(FILES ${robot_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR})
install(FILES ${robot_controllers_utils_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR}/utils)

add_subdirectory(low)
add_subdirectory(high)
