cmake_minimum_required(VERSION 3.8)
project(robot_debugger_ui)

# C++ 17 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt 자동 생성 코드에서 발생할 수 있는 경고 무시
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Qt5 의존성 찾기
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui UiTools Network Sql Concurrent PrintSupport)

# 포함 디렉토리 명시적으로 추가
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5UiTools_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
  ${Qt5Sql_INCLUDE_DIRS}
  ${Qt5Concurrent_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
)

# QCustomPlot 라이브러리 확인 및 추가
set(QCUSTOMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/qcustomplot)
set(QCUSTOMPLOT_SRC ${QCUSTOMPLOT_DIR}/qcustomplot.cpp)
if(EXISTS ${QCUSTOMPLOT_SRC})
  list(APPEND SOURCES ${QCUSTOMPLOT_SRC})
  include_directories(${QCUSTOMPLOT_DIR})
  # QCustomPlot에 필요한 정의 추가
  add_compile_definitions(QCUSTOMPLOT_USE_LIBRARY)
else()
  message(STATUS "QCustomPlot 소스 파일을 찾을 수 없습니다. QCustomPlot 기능이 비활성화됩니다.")
  add_compile_definitions(NO_QCUSTOMPLOT)
endif()

# 소스 및 헤더 파일 목록
set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/topic_manager.cpp
  src/message_store.cpp
  src/configuration_manager.cpp
  src/plugin_manager.cpp
  src/panels/battery_gauge_panel.cpp
)

set(HEADERS
  include/robot_debugger_ui/main_window.hpp
  include/robot_debugger_ui/topic_manager.hpp
  include/robot_debugger_ui/message_store.hpp
  include/robot_debugger_ui/configuration_manager.hpp
  include/robot_debugger_ui/plugin_manager.hpp
  include/robot_debugger_ui/panel_interface.hpp
  include/robot_debugger_ui/panels/battery_gauge_panel.hpp
)

# Qt 자동 생성 코드 설정 (MOC, UIC, RCC)
qt5_wrap_cpp(MOC_FILES ${HEADERS})

# Qt .ui 파일 자동 빌드
set(UIS_HDRS
  ui/main_window.ui
  ui/dashboard.ui
  ui/jetcobot_tab.ui
  ui/pinky_tab.ui
)
qt5_wrap_ui(UI_FILES_GENERATED ${UIS_HDRS})

# 리소스 파일 설정
set(QRC_FILES
  resources/icons.qrc
)
qt5_add_resources(QRC_RESOURCES ${QRC_FILES})

# 실행 파일 설정
add_executable(robot_debugger_ui_node
  ${SOURCES}
  ${MOC_FILES}
  ${UI_FILES_GENERATED}
  ${QRC_RESOURCES}
)

# 헤더 디렉토리 설정
target_include_directories(robot_debugger_ui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# 라이브러리 링크
ament_target_dependencies(robot_debugger_ui_node
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

target_link_libraries(robot_debugger_ui_node
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::UiTools
  Qt5::Network
  Qt5::Sql
  Qt5::Concurrent
  Qt5::PrintSupport
)

# Qt 자동 생성 코드 디버깅 옵션
set(CMAKE_AUTOMOC_RELAXED_MODE ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 실행 파일 설치
install(TARGETS
  robot_debugger_ui_node
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# UI 파일, 리소스 설치
install(DIRECTORY
  ui
  resources
  DESTINATION share/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY
  include/
  DESTINATION include
)

# QCustomPlot에 필요한 정의 추가는 파일이 존재할 때만 적용
if(NOT DEFINED NO_QCUSTOMPLOT)
  target_compile_definitions(robot_debugger_ui_node PRIVATE QCUSTOMPLOT_USE_LIBRARY)
endif()

ament_package() 