set(control_stack_HEADERS
    control_stack.hpp
    macros.hpp
)

set(controllers_SOURCES
    controllers/passive_ds.cpp
)

set(planners_SOURCES
    planners/linear_ds.cpp
)

add_library(ControlStack SHARED ${controllers_SOURCES} ${planners_SOURCES})

target_include_directories(ControlStack PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

install(TARGETS ControlStack
    RUNTIME DESTINATION ${CONTROL_STACK_BINARY_INSTALL_DIR}
    LIBRARY DESTINATION ${CONTROL_STACK_LIBRARY_INSTALL_DIR}
    ARCHIVE DESTINATION ${CONTROL_STACK_LIBRARY_INSTALL_DIR})

install(FILES ${control_stack_HEADERS} DESTINATION ${CONTROL_STACK_INCLUDE_INSTALL_DIR})

add_subdirectory(controllers)
add_subdirectory(planners)
