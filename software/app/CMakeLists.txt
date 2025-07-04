# Set executable sources
add_executable(${App} 
    src/py32f0xx_it.c 
    src/py32f0xx_hal_msp.c 
    src/main.c 
    src/bdcsc.c
    src/checksum.c
    src/default_config.c
)

# Add custom command to touch main.c to force recompilation to update date and time
add_custom_target(compile_time 
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Updating Compile Time"
    COMMAND ${CMAKE_COMMAND} -E touch_nocreate src/main.c
)

# Get the version using git describe
execute_process(
    COMMAND git describe --tag --dirty
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Add the version as a compile definition
target_compile_definitions(${App} PUBLIC GIT_VERSION="${GIT_VERSION}")

add_dependencies(${App} compile_time)

target_compile_definitions(${App} PUBLIC
    SET_DEFAULT_CONFIG          # provide a default configuration in the binary.
)

target_link_libraries(${App} PUBLIC debug_io config flash config pid) 
target_link_libraries(${App} PUBLIC $<TARGET_OBJECTS:PuyaBSP>)
target_include_directories(${App} PRIVATE $<TARGET_PROPERTY:PuyaBSP,INTERFACE_INCLUDE_DIRECTORIES>)

set_target_properties(${App} PROPERTIES LINK_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/linker_script_app.ld")

# Generate hex and bin files
add_custom_command(TARGET ${App}
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating binaries"
    # Create temporary hex file
    COMMAND ${CMAKE_OBJCOPY} ARGS 
        -O ihex ${App}.elf temp.hex
    # Insert the checksum in the hex file
    COMMAND srec_cat ARGS 
        temp.hex -Intel 
        -crop 0x08001000 0x08007FFC 
        -fill 0xFF 0x08001000 0x08007FFC
        -STM32 0x08007FFC
        # -line-length=44
        -o ${CMAKE_BINARY_DIR}/bin/${App}.hex -Intel
    # Create binary file
    COMMAND ${CMAKE_OBJCOPY} ARGS
        --input-target=ihex 
        --output-target=binary 
        ${CMAKE_BINARY_DIR}/bin/${App}.hex
        ${CMAKE_BINARY_DIR}/bin/${App}.bin
    # Extracting config from hex file
    COMMAND srec_cat ARGS 
        temp.hex -Intel 
        -crop 0x0800F000 0x0800FFFF 
        -o ${CMAKE_BINARY_DIR}/bin/${PROJECT_NAME}_Cfg.hex -Intel
)