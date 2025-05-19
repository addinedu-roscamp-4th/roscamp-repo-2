#ifndef CONFIGURATION_MANAGER_HPP
#define CONFIGURATION_MANAGER_HPP

#include <QObject>
#include <QSettings>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

namespace robot_debugger_ui {

/**
 * @brief 설정 관리자 클래스
 * 
 * 도메인, 네임스페이스, QoS, 로그 설정 등을 관리하는 클래스입니다.
 */
class ConfigurationManager : public QObject {
  Q_OBJECT

public:
  /**
   * @brief QoS 설정 구조체
   */
  struct QosConfiguration {
    enum class Reliability {
      BEST_EFFORT,
      RELIABLE
    };
    
    enum class Durability {
      VOLATILE,
      TRANSIENT_LOCAL
    };
    
    Reliability reliability; ///< 신뢰성 설정
    Durability durability; ///< 지속성 설정
    int history_depth; ///< 히스토리 깊이
    int timeout_ms; ///< 타임아웃 (밀리초)
  };
  
  /**
   * @brief 로그 설정 구조체
   */
  struct LogConfiguration {
    enum class LogLevel {
      DEBUG,
      INFO,
      WARN,
      ERROR,
      CRITICAL
    };
    
    LogLevel default_level; ///< 기본 로그 레벨
    int max_entries; ///< 최대 로그 항목 수
    bool save_to_file; ///< 파일에 저장할지 여부
    std::string log_file_path; ///< 로그 파일 경로
  };

public:
  /**
   * @brief 생성자
   * @param config_path 설정 파일 경로
   * @param parent 부모 QObject
   */
  explicit ConfigurationManager(const std::string& config_path = "", QObject* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~ConfigurationManager() override;

  /**
   * @brief 설정 로드
   * @return 성공 여부
   */
  bool loadConfiguration();
  
  /**
   * @brief 설정 저장
   * @return 성공 여부
   */
  bool saveConfiguration();

  /**
   * @brief 도메인 패턴 가져오기
   * @return 도메인 패턴 목록
   */
  std::vector<int> getDomainPatterns() const;
  
  /**
   * @brief 도메인 패턴 설정
   * @param domains 도메인 패턴 목록
   */
  void setDomainPatterns(const std::vector<int>& domains);

  /**
   * @brief 네임스페이스 패턴 가져오기
   * @return 네임스페이스 패턴 목록
   */
  std::vector<std::string> getNamespacePatterns() const;
  
  /**
   * @brief 네임스페이스 패턴 설정
   * @param namespaces 네임스페이스 패턴 목록
   */
  void setNamespacePatterns(const std::vector<std::string>& namespaces);

  /**
   * @brief 토픽 QoS 설정 가져오기
   * @param topic_name 토픽 이름
   * @return QoS 설정
   */
  QosConfiguration getQosConfiguration(const std::string& topic_name) const;
  
  /**
   * @brief 토픽 QoS 설정
   * @param topic_name 토픽 이름
   * @param config QoS 설정
   */
  void setQosConfiguration(const std::string& topic_name, const QosConfiguration& config);

  /**
   * @brief 로그 설정 가져오기
   * @return 로그 설정
   */
  LogConfiguration getLogConfiguration() const;
  
  /**
   * @brief 로그 설정
   * @param config 로그 설정
   */
  void setLogConfiguration(const LogConfiguration& config);

signals:
  /**
   * @brief 설정 변경 시 발생하는 시그널
   */
  void configurationChanged();

private:
  std::unique_ptr<QSettings> settings_; ///< Qt 설정 객체
  std::string config_path_; ///< 설정 파일 경로
  
  std::vector<int> domain_patterns_; ///< 도메인 패턴 목록
  std::vector<std::string> namespace_patterns_; ///< 네임스페이스 패턴 목록
  std::unordered_map<std::string, QosConfiguration> topic_qos_settings_; ///< 토픽별 QoS 설정
  LogConfiguration log_config_; ///< 로그 설정
};

} // namespace robot_debugger_ui

#endif // CONFIGURATION_MANAGER_HPP 