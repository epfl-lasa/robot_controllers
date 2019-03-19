set(low_level_controllers_HEADERS
    PassiveDS.hpp
)

install(FILES ${low_level_controllers_HEADERS} DESTINATION ${CONTROL_STACK_INCLUDE_INSTALL_DIR}/controllers)