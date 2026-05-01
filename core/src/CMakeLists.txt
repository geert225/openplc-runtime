cmake_minimum_required(VERSION 3.10)
project(plc_application CXX C)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Python development libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(PYTHON REQUIRED python3-embed)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/core/src/plc_app
                    ${CMAKE_SOURCE_DIR}/core/src/plc_app/utils
                    ${CMAKE_SOURCE_DIR}/core/strucpp_runtime/include
                    ${CMAKE_SOURCE_DIR}/core/generated/plc_lib
                    ${CMAKE_SOURCE_DIR}/core/generated/plc_lib/lib
                    ${PYTHON_INCLUDE_DIRS})

# Compiler options
add_compile_options(-Wall -Werror -Wextra -fstack-protector-strong
    -D_FORTIFY_SOURCE=2 -O2 -Wformat
    -Werror=format-security -fPIC -fPIE)

# On Cygwin/MSYS2, disable treating warnings as errors due to header conflicts
# The _POSIX_C_SOURCE redefinition warning is caused by Python headers vs system headers
if(CMAKE_SYSTEM_NAME MATCHES "CYGWIN|MSYS")
    add_compile_options(-Wno-error)
endif()

# Step 3: Build the executable and link against the shared library
add_executable(plc_main
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/plc_main.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/utils/log.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/utils/utils.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/utils/watchdog.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/image_tables.cpp
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/journal_buffer.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/plc_state_manager.cpp
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/plcapp_manager.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/scan_cycle_manager.c
    ${CMAKE_SOURCE_DIR}/core/src/drivers/plugin_driver.c
    ${CMAKE_SOURCE_DIR}/core/src/drivers/plugin_config.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/unix_socket.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/debug_handler.c
    ${CMAKE_SOURCE_DIR}/core/src/plc_app/client_tcp_udp.c
)

# Link against shared library
target_link_libraries(plc_main
    dl
    pthread
    ${PYTHON_LIBRARIES}
)

# Export symbols from the executable so that dlopen'd PLC shared libraries
# can resolve functions provided by the runtime (e.g. TCP communication blocks)
target_link_options(plc_main PRIVATE -rdynamic)

# Ensure executable can find shared library at runtime
set_target_properties(plc_main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
