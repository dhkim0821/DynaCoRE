add_subdirectory(NAO)
add_subdirectory(Valkyrie)
add_subdirectory(Mercury)
add_subdirectory(Atlas)

install(FILES RobotSystem.hpp RobotSystem2D.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")
