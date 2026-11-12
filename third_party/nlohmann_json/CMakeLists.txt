# The upstream single-header distribution needs no generated files or libraries.
# Both board and host consumers may add this directory from their own CMake.
cmake_minimum_required(VERSION 3.16)

if(TARGET nlohmann_json::nlohmann_json)
    return()
endif()

add_library(nlohmann_json INTERFACE)
add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
target_include_directories(nlohmann_json SYSTEM INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/include")
target_compile_features(nlohmann_json INTERFACE cxx_std_11)
