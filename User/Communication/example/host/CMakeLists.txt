cmake_minimum_required(VERSION 3.10)
project(uproto_host C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

## C-only demo removed; focus on C++ target below

# C++ host target (reuses core + host platform + serial)
add_executable(uproto_host_cpp
    cpp/main.cpp
    # header-only modules included via headers
    ../../core/uproto.c
    ../../core/comm.c
    ../../core/CRC8_CRC16.c
)

target_compile_features(uproto_host_cpp PRIVATE cxx_std_17)
target_compile_definitions(uproto_host_cpp PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE _GNU_SOURCE)
target_include_directories(uproto_host_cpp PRIVATE ../../core)
target_include_directories(uproto_host_cpp PRIVATE ../shared)
target_include_directories(uproto_host_cpp PRIVATE .)
target_include_directories(uproto_host_cpp PRIVATE cpp)
if (WIN32)
    target_link_libraries(uproto_host_cpp PRIVATE ws2_32)
endif()
