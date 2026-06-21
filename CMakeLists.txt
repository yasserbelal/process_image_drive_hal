cmake_minimum_required(VERSION 3.16)
project(neura_process_image VERSION 1.0.0 LANGUAGES CXX)

# C++17, strict warnings ────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Header-only library target ────────────────────────────────────────────────
# All implementation is in headers: This target just carries the include path.
add_library(neura_pi INTERFACE)
target_include_directories(neura_pi INTERFACE ${CMAKE_SOURCE_DIR}/include)

# Demo executable ───────────────────────────────────────────────────────────
add_executable(demo src/main.cpp)
target_link_libraries(demo PRIVATE neura_pi)

# Test executable ───────────────────────────────────────────────────────────
enable_testing()
add_executable(test_all tests/test_all.cpp)
target_link_libraries(test_all PRIVATE neura_pi)
add_test(NAME unit_tests COMMAND test_all)
