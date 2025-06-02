#ifndef BATTERY_GAUGE_PANEL_HPP
#define BATTERY_GAUGE_PANEL_HPP

#include <QWidget>
#include <QLabel>
#include <QComboBox>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <memory>
#include "robot_debugger_ui/panel_interface.hpp"
#include "robot_debugger_ui/topic_manager.hpp"
#include "robot_debugger_ui/message_store.hpp"

namespace Ui {
class BatteryGaugePanel;
}

namespace robot_debugger_ui {

/**
 * @brief 배터리 게이지 패널 클래스
 * 
 * 로봇의 배터리 상태를 표시하는 패널입니다.
 */
class BatteryGaugePanel : public PanelInterface {
  Q_OBJECT

public:
  /**
   * @brief 생성자
   * @param topic_manager 토픽 관리자
   * @param message_store 메시지 저장소
   * @param parent 부모 위젯
   */
  explicit BatteryGaugePanel(
    std::shared_ptr<TopicManager> topic_manager,
    std::shared_ptr<MessageStore> message_store,
    QWidget* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~BatteryGaugePanel() override;

  /**
   * @brief 패널 ID 가져오기
   * @return 패널 ID
   */
  QString id() const override;

  /**
   * @brief 패널 타이틀 가져오기
   * @return 패널 타이틀
   */
  QString title() const override;

  /**
   * @brief 패널 설명 가져오기
   * @return 패널 설명
   */
  QString description() const override;

  /**
   * @brief 패널 아이콘 가져오기
   * @return 패널 아이콘
   */
  QIcon icon() const override;

  /**
   * @brief 패널 설정 가져오기
   * @return 패널 설정
   */
  QVariantMap settings() const override;

  /**
   * @brief 패널 설정 저장
   * @param settings 패널 설정
   */
  void saveSettings(const QVariantMap& settings) override;

  /**
   * @brief 패널 초기화
   * @param settings 패널 설정
   * @return 초기화 성공 여부
   */
  bool initialize(const QVariantMap& settings) override;

  /**
   * @brief 패널 상태 갱신
   */
  void update() override;

  /**
   * @brief 패널 정지
   */
  void stop() override;

  /**
   * @brief 패널 종료
   */
  void shutdown() override;

  /**
   * @brief 설정 대화상자 존재 여부
   * @return 설정 대화상자 존재 여부
   */
  bool hasConfigDialog() const override;

  /**
   * @brief 설정 대화상자 표시
   * @return 대화상자 수락 여부
   */
  bool showConfigDialog() override;

private slots:
  /**
   * @brief 토픽 선택 변경 시 호출
   * @param topic_name 선택된 토픽 이름
   */
  void onTopicSelected(const QString& topic_name);

  /**
   * @brief 업데이트 타이머에 의해 주기적으로 호출
   */
  void onUpdateTimer();

  /**
   * @brief 토픽 목록 갱신 버튼 클릭 시 호출
   */
  void onRefreshTopics();

  /**
   * @brief 토픽 목록 변경 시 호출
   * @param topics 토픽 정보 목록
   */
  void onTopicsChanged(const std::vector<TopicManager::TopicInfo>& topics);

  /**
   * @brief 메시지 수신 시 호출
   * @param message 수신된 메시지
   */
  void onMessageReceived(const MessageStore::StoredMessage& message);

private:
  /**
   * @brief UI 초기화
   */
  void setupUi();

  /**
   * @brief 배터리 토픽 목록 갱신
   */
  void updateBatteryTopics();

  /**
   * @brief 배터리 상태 업데이트
   * @param topic_name 토픽 이름
   * @param percentage 배터리 잔량 (%)
   * @param voltage 전압 (V)
   */
  void updateBatteryStatus(const std::string& topic_name, double percentage, double voltage);

  std::shared_ptr<TopicManager> topic_manager_; ///< 토픽 관리자
  std::shared_ptr<MessageStore> message_store_; ///< 메시지 저장소
  
  QLabel* title_label_; ///< 타이틀 레이블
  QComboBox* topic_combo_; ///< 토픽 선택 콤보박스
  QPushButton* refresh_button_; ///< 갱신 버튼
  QProgressBar* battery_progress_; ///< 배터리 프로그레스바
  QLabel* voltage_label_; ///< 전압 레이블
  QLabel* percentage_label_; ///< 잔량 레이블
  QLabel* status_label_; ///< 상태 레이블
  QTimer* update_timer_; ///< 업데이트 타이머
  
  QString current_topic_; ///< 현재 선택된 토픽
  QVariantMap panel_settings_; ///< 패널 설정
  
  double last_percentage_; ///< 마지막 배터리 잔량
  double last_voltage_; ///< 마지막 전압
  
  static const QString PANEL_ID; ///< 패널 ID
  static const QString PANEL_TITLE; ///< 패널 타이틀
  static const QString PANEL_DESCRIPTION; ///< 패널 설명
};

} // namespace robot_debugger_ui

#endif // BATTERY_GAUGE_PANEL_HPP 