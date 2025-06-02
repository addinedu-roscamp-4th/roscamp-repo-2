#ifndef MESSAGE_STORE_HPP
#define MESSAGE_STORE_HPP

#include <QObject>
#include <deque>
#include <map>
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <QVariant>
#include <QSqlDatabase>

namespace robot_debugger_ui {

/**
 * @brief 메시지 저장소 클래스
 * 
 * ROS 2 토픽 메시지를 저장하고 관리하는 저장소입니다.
 * 순환 버퍼를 사용하여 메모리 사용량을 제한하고,
 * 중요 이벤트는 SQLite 데이터베이스에 영구 저장합니다.
 */
class MessageStore : public QObject {
  Q_OBJECT

public:
  /**
   * @brief 저장된 메시지 구조체
   */
  struct StoredMessage {
    std::string topic_name; ///< 토픽 이름
    std::string type_name; ///< 메시지 타입 이름
    std::vector<uint8_t> data; ///< 직렬화된 메시지 데이터
    std::chrono::system_clock::time_point timestamp; ///< 저장 시각
    int log_level; ///< 로그 레벨 (0: DEBUG, 1: INFO, 2: WARN, 3: ERROR, 4: CRITICAL)
    std::string source; ///< 메시지 소스 (예: 네임스페이스, 로봇 이름)
    
    /**
     * @brief StoredMessage를 QVariant로 변환
     */
    QVariant toVariant() const;
    
    /**
     * @brief QVariant에서 StoredMessage로 변환
     */
    static StoredMessage fromVariant(const QVariant& variant);
  };

  /**
   * @brief 생성자
   * @param max_messages_per_topic 토픽별 최대 메시지 수
   * @param db_path 데이터베이스 파일 경로 (비어있으면 메모리 내 데이터베이스 사용)
   * @param parent 부모 QObject
   */
  explicit MessageStore(size_t max_messages_per_topic = 100, 
                       const std::string& db_path = "", 
                       QObject* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~MessageStore() override;

  /**
   * @brief 메시지 저장
   * @param topic_name 토픽 이름
   * @param data 직렬화된 메시지 데이터
   * @param type_name 메시지 타입 이름
   * @param log_level 로그 레벨
   * @param source 메시지 소스
   */
  void storeMessage(const std::string& topic_name, 
                    const std::vector<uint8_t>& data,
                    const std::string& type_name = "",
                    int log_level = 1,
                    const std::string& source = "");

  /**
   * @brief 최신 메시지 가져오기
   * @param topic_name 토픽 이름
   * @return 최신 저장 메시지 (없으면 빈 메시지)
   */
  StoredMessage getLatestMessage(const std::string& topic_name) const;

  /**
   * @brief 메시지 이력 가져오기
   * @param topic_name 토픽 이름
   * @param count 가져올 메시지 수 (기본값: 모든 메시지)
   * @return 메시지 이력 벡터
   */
  std::vector<StoredMessage> getMessageHistory(const std::string& topic_name, size_t count = 0) const;

  /**
   * @brief 지정된 시간 범위의 메시지 가져오기
   * @param topic_name 토픽 이름
   * @param start_time 시작 시간
   * @param end_time 종료 시간 (기본값: 현재 시간)
   * @return 시간 범위 내 메시지 벡터
   */
  std::vector<StoredMessage> getMessagesByTimeRange(
      const std::string& topic_name,
      const std::chrono::system_clock::time_point& start_time,
      const std::chrono::system_clock::time_point& end_time = std::chrono::system_clock::now()) const;

  /**
   * @brief 특정 로그 레벨 이상의 메시지 가져오기
   * @param min_level 최소 로그 레벨
   * @param count 가져올 메시지 수 (기본값: 모든 메시지)
   * @return 로그 레벨 조건을 만족하는 메시지 벡터
   */
  std::vector<StoredMessage> getMessagesByLogLevel(int min_level, size_t count = 0) const;

  /**
   * @brief 저장된 모든 토픽 이름 가져오기
   * @return 토픽 이름 벡터
   */
  std::vector<std::string> getStoredTopics() const;

  /**
   * @brief 토픽별 메시지 수 가져오기
   * @return 토픽별 메시지 수 맵
   */
  std::map<std::string, size_t> getMessageCounts() const;

  /**
   * @brief 메시지 저장소 초기화
   */
  void clear();

  /**
   * @brief 특정 토픽의 메시지 초기화
   * @param topic_name 토픽 이름
   */
  void clearTopic(const std::string& topic_name);

  /**
   * @brief 토픽별 최대 메시지 수 설정
   * @param max_messages 최대 메시지 수
   */
  void setMaxMessagesPerTopic(size_t max_messages);

  /**
   * @brief 데이터베이스에서 이벤트 로드
   * @param start_time 시작 시간
   * @param end_time 종료 시간
   * @param min_level 최소 로그 레벨
   * @return 저장된 이벤트 벡터
   */
  std::vector<StoredMessage> loadEventsFromDatabase(
      const std::chrono::system_clock::time_point& start_time,
      const std::chrono::system_clock::time_point& end_time,
      int min_level = 2) const;

signals:
  /**
   * @brief 새 메시지 저장 시 발생하는 시그널
   * @param message 저장된 메시지
   */
  void messageStored(const StoredMessage& message);
  
  /**
   * @brief 중요 이벤트 발생 시 시그널
   * @param message 이벤트 메시지
   */
  void significantEvent(const StoredMessage& message);

private:
  /**
   * @brief 데이터베이스 초기화
   * @return 초기화 성공 여부
   */
  bool initializeDatabase();
  
  /**
   * @brief 중요 이벤트를 데이터베이스에 저장
   * @param message 이벤트 메시지
   */
  void storeEventInDatabase(const StoredMessage& message);

  // 토픽별 메시지 저장소 (순환 버퍼로 구현)
  mutable std::mutex mutex_;
  std::map<std::string, std::deque<StoredMessage>> message_store_;
  size_t max_messages_per_topic_;
  
  // SQLite 데이터베이스 관련
  QSqlDatabase db_;
  std::string db_path_;
  bool use_database_;
};

} // namespace robot_debugger_ui

#endif // MESSAGE_STORE_HPP 