#ifndef MESSAGE_STORE_HPP
#define MESSAGE_STORE_HPP

#include <QObject>
#include <deque>
#include <unordered_map>
#include <string>
#include <vector>
#include <mutex>
#include <memory>

namespace robot_debugger_ui {

/**
 * @brief 메시지 저장소 클래스
 * 
 * 구독한 토픽 메시지를 저장하는 클래스입니다.
 * 토픽별로 순환 버퍼를 유지합니다.
 */
class MessageStore : public QObject {
  Q_OBJECT

public:
  /**
   * @brief 시간과 메시지를 포함하는 구조체
   */
  struct TimedMessage {
    double timestamp; ///< 메시지 수신 시간 (초)
    std::vector<uint8_t> data; ///< 직렬화된 메시지 데이터
  };

  /**
   * @brief 생성자
   * @param max_history_per_topic 토픽별 최대 메시지 보존 개수
   * @param parent 부모 QObject
   */
  explicit MessageStore(size_t max_history_per_topic = 100, QObject* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~MessageStore() override;

  /**
   * @brief 최신 메시지 조회
   * @param topic_name 토픽 이름
   * @return 최신 메시지 (없으면 빈 메시지)
   */
  TimedMessage getLatestMessage(const std::string& topic_name);
  
  /**
   * @brief 특정 토픽의 메시지 이력 조회
   * @param topic_name 토픽 이름
   * @param count 조회할 메시지 개수 (0이면 전체)
   * @return 최신 순서로 정렬된 메시지 목록
   */
  std::vector<TimedMessage> getMessageHistory(const std::string& topic_name, size_t count = 0);
  
  /**
   * @brief 모든 토픽 이름 조회
   * @return 저장소에 있는 모든 토픽 이름
   */
  std::vector<std::string> getTopicNames();
  
  /**
   * @brief 토픽 저장 용량 설정
   * @param topic_name 토픽 이름
   * @param capacity 저장 용량 (메시지 개수)
   */
  void setTopicCapacity(const std::string& topic_name, size_t capacity);

public slots:
  /**
   * @brief 메시지 저장
   * @param topic_name 토픽 이름
   * @param data 직렬화된 메시지 데이터
   */
  void storeMessage(const std::string& topic_name, const std::vector<uint8_t>& data);

signals:
  /**
   * @brief 새 메시지 저장 시 발생하는 시그널
   * @param topic_name 토픽 이름
   */
  void messageStored(const std::string& topic_name);

private:
  std::unordered_map<std::string, std::deque<TimedMessage>> message_buffers_; ///< 토픽별 메시지 버퍼
  std::unordered_map<std::string, size_t> topic_capacities_; ///< 토픽별 저장 용량
  size_t default_capacity_; ///< 기본 저장 용량
  std::mutex buffer_mutex_; ///< 버퍼 접근 뮤텍스
};

} // namespace robot_debugger_ui

#endif // MESSAGE_STORE_HPP 