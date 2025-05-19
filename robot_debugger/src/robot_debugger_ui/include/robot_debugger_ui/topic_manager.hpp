#ifndef TOPIC_MANAGER_HPP
#define TOPIC_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <QObject>
#include <QTimer>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>

namespace robot_debugger_ui {

/**
 * @brief 토픽 관리자 클래스
 * 
 * ROS 2 토픽을 관리하는 클래스입니다.
 * 지정된 패턴에 맞는 토픽을 스캔하고 구독합니다.
 */
class TopicManager : public QObject {
  Q_OBJECT

public:
  /**
   * @brief 생성자
   * @param node ROS 2 노드
   * @param parent 부모 QObject
   */
  TopicManager(std::shared_ptr<rclcpp::Node> node, QObject* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~TopicManager() override;

  /**
   * @brief 토픽 스캔 시작
   */
  void startScan();
  
  /**
   * @brief 토픽 스캔 중지
   */
  void stopScan();

  /**
   * @brief 도메인 패턴 설정
   * @param domains 도메인 ID 패턴 목록
   */
  void setDomainPatterns(const std::vector<int>& domains);
  
  /**
   * @brief 네임스페이스 패턴 설정
   * @param namespaces 네임스페이스 패턴 목록
   */
  void setNamespacePatterns(const std::vector<std::string>& namespaces);

signals:
  /**
   * @brief 메시지 수신 시 발생하는 시그널
   * @param topic 토픽 이름
   * @param msg 직렬화된 메시지 데이터
   */
  void messageReceived(const std::string& topic, const std::vector<uint8_t>& msg);
  
  /**
   * @brief 토픽 목록이 변경되었을 때 발생하는 시그널
   * @param topics 감지된 토픽 목록
   */
  void topicsChanged(const std::vector<std::string>& topics);

private slots:
  /**
   * @brief 토픽 스캔 타이머에 의해 호출되는 슬롯
   */
  void onScanTimer();

private:
  /**
   * @brief 토픽이 패턴에 맞는지 확인
   * @param topic_name 토픽 이름
   * @return 패턴에 맞으면 true, 아니면 false
   */
  bool matchesPattern(const std::string& topic_name);
  
  /**
   * @brief 토픽 구독
   * @param topic_name 토픽 이름
   * @param type_name 토픽 타입 이름
   */
  void subscribeToTopic(const std::string& topic_name, const std::string& type_name);

  std::shared_ptr<rclcpp::Node> node_; ///< ROS 2 노드
  QTimer scan_timer_; ///< 스캔 타이머
  
  std::vector<int> domain_patterns_; ///< 도메인 패턴 목록
  std::vector<std::string> namespace_patterns_; ///< 네임스페이스 패턴 목록
  
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_; ///< 구독 맵
  std::mutex subscriptions_mutex_; ///< 구독 맵 뮤텍스
};

} // namespace robot_debugger_ui

#endif // TOPIC_MANAGER_HPP 