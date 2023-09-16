cmake_minimum_required(VERSION 3.10)
project(grblHAL-sim C)

add_compile_definitions(F_CPU=16000000)

set(grbl_SRC 
    src/grbl/grbllib.h
    src/grbl/crossbar.c
    src/grbl/grbllib.c
    src/grbl/protocol.h
    src/grbl/protocol.c
    src/grbl/planner.h
    src/grbl/planner.c
    src/grbl/settings.h
    src/grbl/settings.c
    src/grbl/nuts_bolts.h
    src/grbl/nuts_bolts.c
    src/grbl/stepper.h
    src/grbl/stepper.c
    src/grbl/gcode.h
    src/grbl/gcode.c
    src/grbl/messages.c
    src/grbl/stream.h
    src/grbl/stream.c
    src/grbl/spindle_control.h
    src/grbl/spindle_control.c
    src/grbl/motion_control.h
    src/grbl/motion_control.c
    src/grbl/machine_limits.h
    src/grbl/machine_limits.c
    src/grbl/coolant_control.h
    src/grbl/coolant_control.c
    src/grbl/system.h
    src/grbl/system.c
    src/grbl/report.h
    src/grbl/report.c
    src/grbl/state_machine.h
    src/grbl/state_machine.c
    src/grbl/override.h
    src/grbl/override.c
    src/grbl/nvs_buffer.h
    src/grbl/nvs_buffer.c
    src/grbl/sleep.h
    src/grbl/sleep.c
    src/grbl/tool_change.h
    src/grbl/tool_change.c
    src/grbl/my_plugin.c
    src/grbl/regex.h
    src/grbl/regex.c
    src/grbl/ngc_expr.h
    src/grbl/ngc_expr.c
    src/grbl/ngc_params.h
    src/grbl/ngc_params.c
    src/grbl/ngc_expr.c
    src/grbl/ioports.h
    src/grbl/ioports.c
    src/grbl/errors.h
    src/grbl/errors.c
    src/grbl/alarms.h
    src/grbl/alarms.c
)

if (WIN32)
    add_compile_definitions(PLATFORM_WINDOWS)

    set(platform_SRC
        src/platform_windows.h
        src/platform_windows.c
    )

    set(platform_LIB)
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
    src/simulator.h
    src/simulator.c
    src/driver.h
    src/driver.c
    src/eeprom.h
    src/eeprom.c
    src/grbl_eeprom_extensions.h
    src/grbl_eeprom_extensions.c
    src/mcu.h
    src/mcu.c
    src/serial.h
    src/serial.c
    src/grbl_interface.h
    src/grbl_interface.c
    src/planner_inject_accessors.c
    ${platform_SRC}
    ${grbl_SRC}
)

target_link_libraries(grblHAL_sim PRIVATE 
    m
    ${platform_LIB}
)

add_executable(grblHAL_validator 
    src/validator.c 
    src/validator_driver.c 
    ${grbl_SRC}
)

target_link_libraries(grblHAL_validator PRIVATE 
    m 
    ${platform_LIB}
)
