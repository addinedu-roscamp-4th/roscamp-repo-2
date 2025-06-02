#ifndef CONFIGURATION_MANAGER_HPP
#define CONFIGURATION_MANAGER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <QObject>
#include <QSettings>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVariant>

namespace robot_debugger_ui {

/**
 * @brief 설정 관리자 클래스
 * 
 * 애플리케이션의 설정을 관리합니다:
 * - 도메인 패턴
 * - 네임스페이스 패턴
 * - 토픽 설정
 * - QoS 설정
 * - 로그 설정
 * - UI 레이아웃 저장/복원
 */
class ConfigurationManager : public QObject {
  Q_OBJECT

public:
  ConfigurationManager();
  virtual ~ConfigurationManager();

  /**
   * @brief 설정 객체 반환
   * @return 설정 객체에 대한 참조
   */
  QSettings& getSettings() { return *settings_; }

  /**
   * @brief 설정을 파일에서 로드
   * @param filename 설정 파일 경로
   * @return 성공 여부
   */
  bool loadFromFile(const std::string& filename);

  /**
   * @brief 설정을 파일에 저장
   * @param filename 설정 파일 경로
   * @return 성공 여부
   */
  bool saveToFile(const std::string& filename);

  /**
   * @brief 도메인 패턴 설정
   * @param domains 도메인 ID 목록
   */
  void setDomainPatterns(const std::vector<int>& domains);

  /**
   * @brief 도메인 패턴 가져오기
   * @return 도메인 ID 목록
   */
  std::vector<int> getDomainPatterns() const;

  /**
   * @brief 네임스페이스 패턴 설정
   * @param namespaces 네임스페이스 목록
   */
  void setNamespacePatterns(const std::vector<std::string>& namespaces);

  /**
   * @brief 네임스페이스 패턴 가져오기
   * @return 네임스페이스 목록
   */
  std::vector<std::string> getNamespacePatterns() const;

  /**
   * @brief 토픽 설정 저장
   * @param topic_name 토픽 이름
   * @param enabled 활성화 여부
   * @param qos QoS 설정 객체
   */
  void setTopicConfig(const std::string& topic_name, bool enabled, const QVariantMap& qos);

  /**
   * @brief 토픽 설정 가져오기
   * @param topic_name 토픽 이름
   * @return 토픽 설정 객체
   */
  QVariantMap getTopicConfig(const std::string& topic_name) const;

  /**
   * @brief 모든 토픽 설정 가져오기
   * @return 토픽 이름을 키로 하는 설정 맵
   */
  std::map<std::string, QVariantMap> getAllTopicConfigs() const;

  /**
   * @brief 레이아웃 저장
   * @param window_name 윈도우 이름
   * @param layout 레이아웃 데이터
   */
  void saveLayout(const std::string& window_name, const QByteArray& layout);

  /**
   * @brief 레이아웃 불러오기
   * @param window_name 윈도우 이름
   * @return 레이아웃 데이터
   */
  QByteArray loadLayout(const std::string& window_name) const;

  /**
   * @brief 로그 레벨 설정
   * @param level 로그 레벨 (0: DEBUG, 1: INFO, 2: WARN, 3: ERROR, 4: CRITICAL)
   */
  void setLogLevel(int level);

  /**
   * @brief 로그 레벨 가져오기
   * @return 로그 레벨
   */
  int getLogLevel() const;

signals:
  /**
   * @brief 설정 변경 시그널
   */
  void configurationChanged();

private:
  // 기본 설정 초기화
  void initDefaults();

  // 도메인 및 네임스페이스 패턴
  std::vector<int> domain_patterns_;
  std::vector<std::string> namespace_patterns_;

  // 토픽 설정
  std::map<std::string, QVariantMap> topic_configs_;

  // 레이아웃 설정
  std::map<std::string, QByteArray> layouts_;

  // 로그 설정
  int log_level_;

  // 설정 저장 객체
  QSettings* settings_;
};

} // namespace robot_debugger_ui

#endif // CONFIGURATION_MANAGER_HPP 