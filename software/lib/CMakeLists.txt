if(NOT "${PUYA_CHIP}" STREQUAL "")
    add_subdirectory(puya_libs)
    add_subdirectory(flash)
    add_subdirectory(debug_io)
    add_subdirectory(config)
endif()

add_subdirectory(pid)
add_subdirectory(rbuff)