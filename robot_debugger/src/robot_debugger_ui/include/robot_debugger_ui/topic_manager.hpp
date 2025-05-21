#ifndef TOPIC_MANAGER_HPP
#define TOPIC_MANAGER_HPP

#include <QObject>
#include <QTimer>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "robot_debugger_ui/configuration_manager.hpp"

namespace robot_debugger_ui {

/**
 * @brief 토픽 관리자 클래스
 * 
 * ROS 2 토픽을 관리하고 모니터링하는 클래스입니다.
 * 지정된 도메인 ID와 네임스페이스 패턴에 매칭되는 토픽을 자동으로 감지하고,
 * 해당 토픽을 구독하여 메시지를 수신합니다.
 */
class TopicManager : public QObject {
  Q_OBJECT

public:
  /**
   * @brief 토픽 정보 구조체
   */
  struct TopicInfo {
    std::string name;        ///< 토픽 이름
    std::string type;        ///< 토픽 타입 이름
    std::string namespace_;  ///< 네임스페이스
    std::string node;        ///< 발행 노드 이름
    bool is_subscribed;      ///< 구독 여부
    int domain_id;           ///< 도메인 ID
    int qos_profile;         ///< QoS 프로파일 ID
  };

  /**
   * @brief 생성자
   * @param node ROS 노드 포인터
   * @param config_manager 설정 관리자 포인터 (기본값: nullptr)
   * @param parent 부모 QObject
   */
  explicit TopicManager(rclcpp::Node::SharedPtr node, 
                       std::shared_ptr<ConfigurationManager> config_manager = nullptr,
                       QObject* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~TopicManager() override;

  /**
   * @brief 토픽 스캔 시작
   * @param interval_ms 스캔 간격 (밀리초)
   */
  void startTopicScan(int interval_ms = 1000);

  /**
   * @brief 토픽 스캔 중지
   */
  void stopTopicScan();

  /**
   * @brief 토픽 구독
   * @param topic_name 토픽 이름
   * @param topic_type 토픽 타입 이름
   * @return 구독 성공 여부
   */
  bool subscribeTopic(const std::string& topic_name, const std::string& topic_type);

  /**
   * @brief 토픽 구독 중지
   * @param topic_name 토픽 이름
   * @return 중지 성공 여부
   */
  bool unsubscribeTopic(const std::string& topic_name);

  /**
   * @brief 메시지 발행
   * @param topic_name 토픽 이름
   * @param topic_type 토픽 타입 이름
   * @param message_data 직렬화된 메시지 데이터
   * @return 발행 성공 여부
   */
  bool publishMessage(const std::string& topic_name, 
                     const std::string& topic_type,
                     const std::vector<uint8_t>& message_data);

  /**
   * @brief 토픽 정보 목록 가져오기
   * @return 토픽 정보 목록
   */
  std::vector<TopicInfo> getTopicInfoList() const;

  /**
   * @brief 토픽 목록 가져오기
   * @return 토픽 목록
   */
  std::vector<std::pair<std::string, std::string>> getTopicList() const;

  /**
   * @brief 토픽 업데이트
   * 강제로 토픽 목록 갱신
   */
  void updateTopics();

  /**
   * @brief 도메인 패턴 설정
   * @param domains 도메인 ID 목록
   */
  void setDomainPatterns(const std::vector<int>& domains);

  /**
   * @brief 네임스페이스 패턴 설정
   * @param namespaces 네임스페이스 목록
   */
  void setNamespacePatterns(const std::vector<std::string>& namespaces);

  /**
   * @brief 구성 관리자 설정
   * @param config_manager 구성 관리자 포인터
   */
  void setConfigurationManager(std::shared_ptr<ConfigurationManager> config_manager);

  /**
   * @brief 현재 도메인 ID 가져오기
   * @return 현재 도메인 ID
   */
  int getCurrentDomainId() const;

  /**
   * @brief 도메인 ID 설정
   * @param domain_id 도메인 ID
   */
  void setDomainId(int domain_id);

signals:
  /**
   * @brief 메시지 수신 시그널
   * @param topic_name 토픽 이름
   * @param message_data 직렬화된 메시지 데이터
   */
  void messageReceived(const std::string& topic_name, const std::vector<uint8_t>& message_data);

  /**
   * @brief 토픽 목록 변경 시그널
   * @param topics 토픽 정보 목록
   */
  void topicsChanged(const std::vector<TopicInfo>& topics);

  /**
   * @brief 토픽 발견 시그널
   * @param topic_info 발견된 토픽 정보
   */
  void topicDiscovered(const TopicInfo& topic_info);

  /**
   * @brief 토픽 사라짐 시그널
   * @param topic_name 사라진 토픽 이름
   */
  void topicLost(const std::string& topic_name);

  /**
   * @brief 토픽 상태 변경 시그널
   * @param topic_name 토픽 이름
   * @param is_active 활성 상태
   */
  void topicStatusChanged(const std::string& topic_name, bool is_active);

private slots:
  /**
   * @brief 토픽 스캔 슬롯
   * 타이머에 의해 호출되어 토픽 목록 갱신
   */
  void scanTopics();

private:
  /**
   * @brief 토픽 타입 이름으로 구독자 생성
   * @param topic_name 토픽 이름
   * @param topic_type 토픽 타입 이름
   * @return 구독 성공 여부
   */
  bool createSubscriptionForType(const std::string& topic_name, const std::string& topic_type);

  /**
   * @brief 메시지 콜백 함수
   * @param topic_name 토픽 이름
   * @param serialized_msg 직렬화된 메시지
   */
  void genericMessageCallback(const std::string& topic_name, 
                            std::shared_ptr<rclcpp::SerializedMessage> serialized_msg);

  /**
   * @brief 토픽이 패턴에 매칭되는지 확인
   * @param topic_name 토픽 이름
   * @return 매칭 여부
   */
  bool matchesPattern(const std::string& topic_name) const;

  /**
   * @brief 네임스페이스 추출
   * @param topic_name 토픽 이름
   * @return 네임스페이스
   */
  std::string extractNamespace(const std::string& topic_name) const;

  rclcpp::Node::SharedPtr node_; ///< ROS 노드
  QTimer* scan_timer_; ///< 토픽 스캔 타이머
  std::shared_ptr<ConfigurationManager> config_manager_; ///< 설정 관리자

  // 구독자 관리
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;

  // 토픽 정보 관리
  mutable std::mutex topic_mutex_;
  std::map<std::string, TopicInfo> topic_infos_;

  // 패턴 매칭 관련
  std::vector<int> domain_patterns_;
  std::vector<std::string> namespace_patterns_;

  bool is_scanning_; ///< 스캔 중 여부
  int current_domain_id_; ///< 현재 도메인 ID
};

} // namespace robot_debugger_ui

#endif // TOPIC_MANAGER_HPP 