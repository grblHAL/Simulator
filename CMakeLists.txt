cmake_minimum_required(VERSION 3.10)
project(grblHAL-sim C)

add_compile_definitions(F_CPU=16000000)
#add_compile_definitions(SQUARING_ENABLED=1)

# Capture the effective build-time flags for the -v/--version output.
string(TOUPPER "${CMAKE_BUILD_TYPE}" _sim_build_type)
if(_sim_build_type STREQUAL "")
    set(SIM_BUILD_TYPE "None")
else()
    set(SIM_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
endif()
string(STRIP "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${_sim_build_type}}" SIM_C_FLAGS)
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/src/build_info.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/build_info.h
    @ONLY
)

include(src/grbl/CMakeLists.txt)

if (WIN32)
    add_compile_definitions(PLATFORM_WINDOWS)

    set(platform_SRC
        src/platform_windows.h
        src/platform_windows.c
    )

    set(platform_LIB
        ws2_32
    )
endif(WIN32)

if(UNIX)
    add_compile_definitions(PLATFORM_LINUX)

    set(platform_SRC
        src/platform_linux.h
        src/platform_linux.c
    )

    if(APPLE)
        set(platform_LIB
            pthread
        )
    else(APPLE)
        set(platform_LIB
            rt
            pthread
        )
    endif(APPLE)
endif(UNIX)

add_executable(grblHAL_sim 
    src/main.c 
    src/simulator.c
    src/driver.c
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/mcu.c
    src/serial.c
    src/grbl_interface.c
    ${platform_SRC}
)

target_link_libraries(grblHAL_sim PRIVATE
    m
    grbl
    ${platform_LIB}
)

# Generated build_info.h lands in the build dir.
target_include_directories(grblHAL_sim PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_executable(grblHAL_validator
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/validator.c 
    src/validator_driver.c 
    ${platform_SRC}
)

target_link_libraries(grblHAL_validator PRIVATE 
    m
    grbl
    ${platform_LIB}
)
