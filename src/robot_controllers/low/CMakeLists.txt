set(low_level_controllers_HEADERS
    PassiveDS.hpp
    Pid.hpp
)

install(FILES ${low_level_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR}/low)