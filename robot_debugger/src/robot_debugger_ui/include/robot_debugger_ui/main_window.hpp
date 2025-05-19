#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QDockWidget>
#include <QToolBar>
#include <QStatusBar>
#include <QStackedWidget>
#include <QTabWidget>
#include <memory>

#include "robot_debugger_ui/topic_manager.hpp"
#include "robot_debugger_ui/message_store.hpp"
#include "robot_debugger_ui/configuration_manager.hpp"
#include "robot_debugger_ui/plugin_manager.hpp"

// 전방 선언
namespace Ui {
class MainWindow;
}

namespace robot_debugger_ui {

/**
 * @brief 메인 윈도우 클래스
 * 
 * 로봇 디버깅 UI의 메인 윈도우를 담당합니다.
 * 툴바, 상태바, 사이드바, 탭별 페이지를 포함합니다.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  /**
   * @brief 생성자
   * @param parent 부모 위젯
   */
  explicit MainWindow(QWidget* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~MainWindow() override;

  /**
   * @brief UI 초기화
   */
  void initializeUi();
  
  /**
   * @brief ROS 노드 초기화
   */
  void initializeRos();

private slots:
  /**
   * @brief 재연결 버튼 클릭 시 호출
   */
  void onReconnectClicked();
  
  /**
   * @brief 긴급 정지 버튼 클릭 시 호출
   */
  void onEmergencyClicked();
  
  /**
   * @brief 로그 레벨 변경 시 호출
   */
  void onLogLevelChanged(int level);
  
  /**
   * @brief 토픽 메시지 수신 시 호출
   */
  void onMessageReceived(const std::string& topic, const std::vector<uint8_t>& msg);

private:
  /**
   * @brief 대시보드 탭 설정
   */
  void setupDashboardTab();
  
  /**
   * @brief Jetcobot 탭 설정
   */
  void setupJetcobotTab();
  
  /**
   * @brief Pinky 탭 설정
   */
  void setupPinkyTab();
  
  /**
   * @brief 상태바 설정
   */
  void setupStatusBar();
  
  /**
   * @brief 사이드바 설정
   */
  void setupSidebar();

  Ui::MainWindow* ui_; ///< Qt Designer로 생성된 UI 폼 (현재는 사용하지 않음)
  QStackedWidget* central_widget_; ///< 중앙 위젯
  QDockWidget* sidebar_; ///< 사이드바 독 위젯
  
  std::shared_ptr<TopicManager> topic_manager_; ///< 토픽 관리자
  std::shared_ptr<MessageStore> message_store_; ///< 메시지 저장소
  std::shared_ptr<ConfigurationManager> config_manager_; ///< 설정 관리자
  std::shared_ptr<PluginManager> plugin_manager_; ///< 플러그인 관리자
};

} // namespace robot_debugger_ui

#endif // MAIN_WINDOW_HPP 