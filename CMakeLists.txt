cmake_minimum_required(VERSION 3.21)
project(SoftionicsHub LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INSTALL_SimpleBLE OFF)
add_subdirectory(third_party/simpleble/simpleble ${CMAKE_BINARY_DIR}/simpleble)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Charts SerialPort)

add_library(hub_core
  core/src/Framer.cpp
  core/src/Parser.cpp
  core/src/Pipeline.cpp
  core/src/filters/EMA.cpp
  core/src/filters/MA.cpp
  core/src/filters/Notch60.cpp
  core/src/filters/Bias.cpp
)

target_include_directories(hub_core PUBLIC core/include)
target_link_libraries(hub_core PUBLIC simpleble::simpleble)
set_target_properties(hub_core PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)

file(GLOB HUB_MODEL_SOURCES CONFIGURE_DEPENDS core/src/model/*.cpp)
add_library(hub_models OBJECT ${HUB_MODEL_SOURCES})
target_include_directories(hub_models PUBLIC core/include)
set_target_properties(hub_models PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)

if (WIN32)
  add_executable(softionics_hub_gui WIN32
    apps/gui/main.cpp
    apps/gui/MainWindow.h
    apps/gui/MainWindow.cpp
    apps/gui/BleWorker.h
    apps/gui/BleWorker.cpp
    apps/gui/PositionTrackingEngine.h
    apps/gui/PositionTrackingEngine.cpp
    apps/gui/PositionTrackingWindow.h
    apps/gui/PositionTrackingWindow.cpp
  )
else()
  add_executable(softionics_hub_gui
    apps/gui/main.cpp
    apps/gui/MainWindow.h
    apps/gui/MainWindow.cpp
    apps/gui/BleWorker.h
    apps/gui/BleWorker.cpp
    apps/gui/PositionTrackingEngine.h
    apps/gui/PositionTrackingEngine.cpp
    apps/gui/PositionTrackingWindow.h
    apps/gui/PositionTrackingWindow.cpp
  )
endif()

target_sources(softionics_hub_gui PRIVATE $<TARGET_OBJECTS:hub_models>)

set_source_files_properties(apps/gui/BleWorker.cpp PROPERTIES SKIP_AUTOMOC ON)

target_link_libraries(softionics_hub_gui PRIVATE Qt6::Widgets Qt6::Charts Qt6::SerialPort hub_core)
