#include <memory>
#include <QApplication>
#include <QCommandLineParser>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <cstring>
#include <QDir>
#include <QFile>
#include <QLibraryInfo>
#include <QCoreApplication>
#include <QUiLoader>
#include <QWidget>
#include <QPluginLoader>  // 플러그인 로더 디버깅을 위해 추가
#include <thread>

#include "robot_debugger_ui/main_window.hpp"

int main(int argc, char** argv) {
  // ROS 2 초기화 전에 인수 필터링
  std::vector<char*> ros_args;
  std::vector<char*> qt_args;
  
  // 프로그램 이름은 항상 Qt에 전달
  qt_args.push_back(argv[0]);
  
  // 인수 분리
  bool ros_args_section = false;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--ros-args") == 0) {
      ros_args_section = true;
      ros_args.push_back(argv[i]);
    } else if (ros_args_section) {
      // ROS 2 인수 섹션에 있는 모든 인수
      ros_args.push_back(argv[i]);
    } else {
      // 일반 인수는 Qt로 전달
      qt_args.push_back(argv[i]);
    }
  }
  
  // ROS 2는 원래 인수로 초기화
  rclcpp::init(argc, argv);
  
  // Qt 애플리케이션 인스턴스 생성 (필터링된 인수 사용)
  std::cout << "Qt 애플리케이션 시작" << std::endl;
  int qt_argc = qt_args.size();
  QApplication app(qt_argc, qt_args.data());
  app.setApplicationName("로봇 디버깅 도구");
  app.setApplicationVersion("0.1.0");
  
  // AMENT_PREFIX_PATH 환경 변수 출력
  const char* ament_prefix = std::getenv("AMENT_PREFIX_PATH");
  if (ament_prefix) {
    std::cout << "AMENT_PREFIX_PATH: " << ament_prefix << std::endl;
  } else {
    std::cout << "AMENT_PREFIX_PATH가 설정되지 않았습니다." << std::endl;
  }
  
  // Qt 기본 동작 확인
  std::cout << "Qt 버전: " << qVersion() << std::endl;
  std::cout << "Qt 공유 디렉토리: " << QLibraryInfo::location(QLibraryInfo::PrefixPath).toStdString() << std::endl;
  
  // QUiLoader 디버깅 (plugins, 공유 디렉토리 표시)
  QUiLoader loader;
  std::cout << "QUiLoader 플러그인 경로: " << loader.pluginPaths().join(", ").toStdString() << std::endl;
  
  // 현재 경로 출력
  QDir currentDir = QDir::current();
  std::cout << "현재 디렉토리: " << currentDir.absolutePath().toStdString() << std::endl;
  
  // UI 파일 직접 테스트
  std::cout << "UI 파일 직접 테스트 시작" << std::endl;
  
  // UI 파일 경로 탐색 (여러 가능한 경로 확인)
  QStringList uiPaths;
  
  // 1. 애플리케이션 디렉토리 기준 상대 경로
  uiPaths << QCoreApplication::applicationDirPath() + "/../share/robot_debugger_ui/ui";
  
  // 2. 소스 디렉토리 기준 상대 경로 (개발 환경)
  uiPaths << QCoreApplication::applicationDirPath() + "/../../src/robot_debugger_ui/ui";
  
  // 3. 현재 디렉토리 기준 상대 경로
  uiPaths << QDir::current().absolutePath() + "/src/robot_debugger_ui/ui";
  uiPaths << QDir::current().absolutePath() + "/install/robot_debugger_ui/share/robot_debugger_ui/ui";
  
  // 유효한 UI 디렉토리 찾기
  QString validUiPath;
  for (const QString& path : uiPaths) {
    QDir dir(path);
    if (dir.exists() && dir.exists("dashboard.ui")) {
      validUiPath = dir.absolutePath();
      std::cout << "유효한 UI 디렉토리 찾음: " << validUiPath.toStdString() << std::endl;
      
      // QUiLoader에 경로 추가
      loader.addPluginPath(validUiPath);
      
      // 테스트 로드
      QFile testUiFile(validUiPath + "/dashboard.ui");
      if (testUiFile.open(QFile::ReadOnly)) {
        QWidget* testWidget = loader.load(&testUiFile);
        if (testWidget) {
          std::cout << "UI 파일 로드 성공: " << testUiFile.fileName().toStdString() << std::endl;
          testWidget->deleteLater();
        } else {
          std::cout << "UI 파일 로드 실패: " << loader.errorString().toStdString() << std::endl;
        }
        testUiFile.close();
      }
      
      // 환경 변수에 경로 추가 (QUiLoader가 내부적으로 사용)
      qputenv("QT_UI_PATH", validUiPath.toLocal8Bit());
      break;
    }
  }
  
  if (validUiPath.isEmpty()) {
    std::cout << "UI 디렉토리를 찾을 수 없습니다." << std::endl;
    std::cout << "시도한 경로들:" << std::endl;
    for (const QString& path : uiPaths) {
      std::cout << " - " << path.toStdString() << std::endl;
    }
  }
  
  // 명령행 인수 파서 설정
  QCommandLineParser parser;
  parser.setApplicationDescription("로봇 디버깅 UI 도구");
  parser.addHelpOption();
  parser.addVersionOption();
  
  // 도메인 옵션
  QCommandLineOption domainsOption(
    QStringList() << "d" << "domains",
    "도메인 ID 패턴 (쉼표로 구분)",
    "domain_list",
    "10,20"
  );
  parser.addOption(domainsOption);
  
  // 네임스페이스 옵션
  QCommandLineOption namespacesOption(
    QStringList() << "n" << "namespaces",
    "네임스페이스 패턴 (쉼표로 구분)",
    "namespace_list",
    "jetcobot_1,jetcobot_2,pinky_1,pinky_2,pinky_3"
  );
  parser.addOption(namespacesOption);
  
  // 로그 레벨 옵션
  QCommandLineOption logLevelOption(
    QStringList() << "l" << "log-level",
    "로그 레벨 (debug, info, warn, error, fatal)",
    "level",
    "info"
  );
  parser.addOption(logLevelOption);
  
  // 명령행 인수 파싱 (try-catch로 오류 처리)
  try {
    parser.process(app);
  } catch (const std::exception& e) {
    qWarning("명령행 인수 파싱 오류: %s", e.what());
    // 오류가 발생해도 계속 진행
  }
  
  // ROS 스피너 생성
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<rclcpp::Node>("robot_debugger_ui_node");
  executor->add_node(node);
  
  // 스피너 스레드 시작
  std::thread spinner([&executor]() {
    executor->spin();
  });
  
  // 메인 윈도우 생성
  std::cout << "메인 윈도우 생성 시작" << std::endl;
  
  int result = 0;
  
  try {
    robot_debugger_ui::MainWindow mainWindow(node);
    std::cout << "MainWindow 객체 생성 완료" << std::endl;
    mainWindow.show();
    std::cout << "메인 윈도우 표시 완료" << std::endl;
    
    // Qt 이벤트 루프 시작
    std::cout << "Qt 이벤트 루프 시작" << std::endl;
    result = app.exec();
    std::cout << "Qt 애플리케이션 종료: " << result << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "메인 윈도우 실행 중 예외 발생: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "메인 윈도우 실행 중 알 수 없는 예외 발생" << std::endl;
    return 1;
  }
  
  // 종료 처리
  rclcpp::shutdown();
  
  if (spinner.joinable()) {
    spinner.join();
  }
  
  return result;
} 