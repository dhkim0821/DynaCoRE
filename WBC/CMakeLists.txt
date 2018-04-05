add_subdirectory(WBLC)
add_subdirectory(WBDC)

install(FILES WBC.hpp Task.hpp ContactSpec.hpp DESTINATION
    "${INSTALL_INCLUDE_DIR}")
