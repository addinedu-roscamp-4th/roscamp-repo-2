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
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui UiTools)

# 소스 및 헤더 파일 목록
set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/topic_manager.cpp
  src/message_store.cpp
  src/configuration_manager.cpp
  src/plugin_manager.cpp
)

set(HEADERS
  include/robot_debugger_ui/main_window.hpp
  include/robot_debugger_ui/topic_manager.hpp
  include/robot_debugger_ui/message_store.hpp
  include/robot_debugger_ui/configuration_manager.hpp
  include/robot_debugger_ui/plugin_manager.hpp
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

ament_package() 