set(robot_controllers_HEADERS
    AbstractController.hpp
)

set(low_level_controllers_SOURCES
    low/PassiveDS.cpp
)

set(high_level_controllers_SOURCES
    high/LinearDS.cpp
)

add_library(RobotControllers SHARED ${low_level_controllers_SOURCES} ${high_level_controllers_SOURCES})

target_include_directories(RobotControllers PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS})

install(TARGETS RobotControllers
    RUNTIME DESTINATION ${ROBOT_CONTROLLERS_BINARY_INSTALL_DIR}
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR}
    ARCHIVE DESTINATION ${ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR})

install(FILES ${robot_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR})

add_subdirectory(low)
add_subdirectory(high)
