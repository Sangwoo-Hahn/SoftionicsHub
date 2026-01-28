cmake_minimum_required(VERSION 3.21)
project(SoftionicsHub LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INSTALL_SimpleBLE OFF)
add_subdirectory(third_party/simpleble/simpleble ${CMAKE_BINARY_DIR}/simpleble)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)

add_library(hub_core
  core/src/Framer.cpp
  core/src/Parser.cpp
  core/src/Pipeline.cpp
  core/src/filters/EMA.cpp
  core/src/filters/MA.cpp
  core/src/filters/Notch60.cpp
  core/src/filters/Bias.cpp
  core/src/model/Linear.cpp
  core/src/model/BF16.cpp
)

target_include_directories(hub_core PUBLIC core/include)
target_link_libraries(hub_core PUBLIC simpleble::simpleble)

# hub_core는 Qt moc/uic/rcc 불필요 (스캔/오탐 방지 + 빌드속도 개선)
set_target_properties(hub_core PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)

if (WIN32)
  add_executable(softionics_hub_gui WIN32
    apps/gui/main.cpp
    apps/gui/MainWindow.h
    apps/gui/MainWindow.cpp
    apps/gui/BleWorker.h
    apps/gui/BleWorker.cpp
    apps/gui/BF16Window.h
    apps/gui/BF16Window.cpp
  )
else()
  add_executable(softionics_hub_gui
    apps/gui/main.cpp
    apps/gui/MainWindow.h
    apps/gui/MainWindow.cpp
    apps/gui/BleWorker.h
    apps/gui/BleWorker.cpp
    apps/gui/BF16Window.h
    apps/gui/BF16Window.cpp
  )
endif()

# ✅ 핵심: BleWorker.cpp는 Q_OBJECT를 직접 담는 소스가 아니므로 AutoMoc 스캔을 끔
# (BleWorker.h는 정상적으로 moc 처리됨)
set_source_files_properties(apps/gui/BleWorker.cpp PROPERTIES SKIP_AUTOMOC ON)

target_link_libraries(softionics_hub_gui PRIVATE Qt6::Widgets Qt6::Charts hub_core)
