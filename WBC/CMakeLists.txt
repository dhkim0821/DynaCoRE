add_subdirectory(WBDC)
add_subdirectory(WBDC_Relax)
add_subdirectory(WBDC_Rotor)

install(FILES WBC.hpp Task.hpp ContactSpec.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")
