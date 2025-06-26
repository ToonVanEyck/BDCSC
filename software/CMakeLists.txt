cmake_minimum_required (VERSION 3.16.3 FATAL_ERROR)

project(BDCSC  LANGUAGES  C CXX)
set (CMAKE_C_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

if(NOT "${PUYA_CHIP}" STREQUAL "")
    add_compile_definitions(${PUYA_CHIP})
    set(CMAKE_EXECUTABLE_SUFFIX ".elf")
else()
    include(CTest)
endif()

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(DEBUG_IO_RTT_BACKEND 1 CACHE BOOL "Use RTT as debug input output backend")

# Include the libraries
add_subdirectory(lib)

if(NOT "${PUYA_CHIP}" STREQUAL "")
    set(App ${PROJECT_NAME}_App)
    set(Btl ${PROJECT_NAME}_Btl)
    add_subdirectory(app)
    add_subdirectory(btl)

    # Merge Btl + App + Cfg into one binary
    add_custom_command(OUTPUT ${PROJECT_NAME}_BtlAppCfg.hex
        COMMENT "Merging Bootloader, Application and Configuration"
        COMMAND srec_cat 
                bin/${PROJECT_NAME}_App.hex -Intel 
                bin/${PROJECT_NAME}_Cfg.hex -Intel 
                btl/${Btl}.hex -Intel 
                -o bin/${PROJECT_NAME}_BtlAppCfg.hex -Intel -line-length=44
        DEPENDS ${App} ${Btl}
        )
    add_custom_target(BtlAppCfg_hex ALL DEPENDS ${PROJECT_NAME}_BtlAppCfg.hex)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L ${CMAKE_SOURCE_DIR}" CACHE INTERNAL "Linker options")

    # Configure custom flash Btl + App + Cfg target
    add_custom_target(flash_target
        COMMENT "Flashing Btl + App + Cfg on Puya board"
        COMMAND pyocd load 
                --probe `pyocd list | grep -Po '[0-9a-z]{48}|LU_2022_8888'`
                --pack ${CMAKE_SOURCE_DIR}/Puya.PY32F0xx_DFP.1.1.7.pack 
                --target ${PUYA_CHIP} 
                ${CMAKE_BINARY_DIR}/bin/${PROJECT_NAME}_BtlAppCfg.hex
        )
    add_dependencies(flash_target BtlAppCfg_hex)

    # Configure custom flash Btl + App target
    add_custom_target(flash_app
        COMMENT "Flashing Btl + App on Puya board"
        COMMAND pyocd load 
                --probe `pyocd list | grep -Po '[0-9a-z]{48}|LU_2022_8888'`
                --pack ${CMAKE_SOURCE_DIR}/Puya.PY32F0xx_DFP.1.1.7.pack 
                --target ${PUYA_CHIP} 
                ${CMAKE_BINARY_DIR}/bin/${PROJECT_NAME}_App.hex
        )
    add_dependencies(flash_app ${App})

    # Configure custom "rtt" target
    add_custom_target(rtt
        COMMENT "Starting rtt listener"
        COMMAND pyocd rtt
                #--probe `pyocd list | grep -Po '[0-9a-z]{48}|LU_2022_8888'`
                --pack ${CMAKE_SOURCE_DIR}/Puya.PY32F0xx_DFP.1.1.7.pack 
                --target ${PUYA_CHIP} 
                -a `grep -Po '.*(0x[0-9a-f]{8}).*_SEGGER_RTT' ${CMAKE_BINARY_DIR}/app/${PROJECT_NAME}.map | grep -Po '0x[0-9a-f]{8}'`
        )
endif()