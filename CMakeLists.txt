cmake_minimum_required(VERSION 3.21)
project(SoftionicsHub LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INSTALL_SimpleBLE OFF)
add_subdirectory(third_party/simpleble/simpleble ${CMAKE_BINARY_DIR}/simpleble)

add_library(hub_core
  core/src/Framer.cpp
  core/src/Parser.cpp
  core/src/Pipeline.cpp
  core/src/filters/EMA.cpp
  core/src/filters/MA.cpp
  core/src/filters/Notch60.cpp
  core/src/filters/Bias.cpp
  core/src/model/Linear.cpp
)

target_include_directories(hub_core PUBLIC core/include)
target_link_libraries(hub_core PUBLIC simpleble::simpleble)

add_executable(softionics_hub_cli apps/cli/main.cpp)
target_link_libraries(softionics_hub_cli PRIVATE hub_core)
