#include <memory>
#include <QApplication>
#include <QCommandLineParser>
#include <rclcpp/rclcpp.hpp>

#include "robot_debugger_ui/main_window.hpp"

int main(int argc, char** argv) {
  // ROS 2 초기화
  rclcpp::init(argc, argv);
  
  // ROS 노드 생성
  auto node = std::make_shared<rclcpp::Node>("robot_debugger_ui_node");
  
  // QApplication 초기화
  QApplication app(argc, argv);
  app.setApplicationName("로봇 디버깅 도구");
  app.setApplicationVersion("0.1.0");
  
  // 명령행 인수 파서 설정
  QCommandLineParser parser;
  parser.setApplicationDescription("로봇 디버깅 UI 도구");
  parser.addHelpOption();
  parser.addVersionOption();
  
  // 도메인 옵션
  QCommandLineOption domainsOption(
    QStringList() << "d" << "domains",
    "도메인 ID 패턴 (쉼표로 구분)",
    "domains",
    "10,20"
  );
  parser.addOption(domainsOption);
  
  // 네임스페이스 옵션
  QCommandLineOption namespacesOption(
    QStringList() << "n" << "namespaces",
    "네임스페이스 패턴 (쉼표로 구분)",
    "namespaces",
    "jetcobot_1,jetcobot_2,pinky_1,pinky_2,pinky_3"
  );
  parser.addOption(namespacesOption);
  
  // 로그 레벨 옵션
  QCommandLineOption logLevelOption(
    QStringList() << "l" << "log-level",
    "로그 레벨 (debug, info, warn, error, critical)",
    "level",
    "info"
  );
  parser.addOption(logLevelOption);
  
  // 명령행 인수 파싱
  parser.process(app);
  
  // 메인 윈도우 생성
  robot_debugger_ui::MainWindow mainWindow;
  mainWindow.show();
  
  // ROS 스피너 생성
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  
  // 스피너 스레드 시작
  std::thread spinner([&executor]() {
    executor->spin();
  });
  
  // Qt 이벤트 루프 시작
  int result = app.exec();
  
  // 종료 처리
  rclcpp::shutdown();
  
  if (spinner.joinable()) {
    spinner.join();
  }
  
  return result;
} 