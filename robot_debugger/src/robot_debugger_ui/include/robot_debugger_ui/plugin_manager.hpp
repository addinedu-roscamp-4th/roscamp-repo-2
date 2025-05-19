#ifndef PLUGIN_MANAGER_HPP
#define PLUGIN_MANAGER_HPP

#include <QObject>
#include <QPluginLoader>
#include <QDir>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

namespace robot_debugger_ui {

/**
 * @brief 플러그인 인터페이스
 * 
 * 확장 모듈 구현을 위한 인터페이스입니다.
 * Qt의 플러그인 시스템과 일관성을 위해 QObject에서 파생되지 않습니다.
 */
class PluginInterface {
public:
  /**
   * @brief 소멸자
   */
  virtual ~PluginInterface() = default;

  /**
   * @brief 플러그인 이름 반환
   * @return 플러그인 이름
   */
  virtual std::string getName() const = 0;
  
  /**
   * @brief 플러그인 설명 반환
   * @return 플러그인 설명
   */
  virtual std::string getDescription() const = 0;
  
  /**
   * @brief 플러그인 버전 반환
   * @return 플러그인 버전
   */
  virtual std::string getVersion() const = 0;
  
  /**
   * @brief 플러그인 초기화
   * @return 초기화 성공 여부
   */
  virtual bool initialize() = 0;
  
  /**
   * @brief 플러그인 종료
   */
  virtual void shutdown() = 0;
  
  /**
   * @brief 위젯 생성
   * @return 플러그인 위젯
   */
  virtual QWidget* createWidget() = 0;
};

/**
 * @brief 플러그인 관리자 클래스
 * 
 * 확장 모듈을 로드하고 관리하는 클래스입니다.
 */
class PluginManager : public QObject {
  Q_OBJECT

public:
  /**
   * @brief 생성자
   * @param plugin_dir 플러그인 디렉토리 경로
   * @param parent 부모 QObject
   */
  explicit PluginManager(const std::string& plugin_dir = "", QObject* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~PluginManager() override;

  /**
   * @brief 플러그인 로드
   * @return 로드된 플러그인 개수
   */
  int loadPlugins();
  
  /**
   * @brief 모든 플러그인 언로드
   */
  void unloadPlugins();

  /**
   * @brief 플러그인 목록 반환
   * @return 로드된 플러그인 이름 목록
   */
  std::vector<std::string> getPluginNames() const;
  
  /**
   * @brief 플러그인 가져오기
   * @param name 플러그인 이름
   * @return 플러그인 인터페이스 (없으면 nullptr)
   */
  PluginInterface* getPlugin(const std::string& name) const;
  
  /**
   * @brief 플러그인 위젯 생성
   * @param name 플러그인 이름
   * @return 플러그인 위젯 (없으면 nullptr)
   */
  QWidget* createPluginWidget(const std::string& name);

signals:
  /**
   * @brief 플러그인 로드 시 발생하는 시그널
   * @param name 플러그인 이름
   */
  void pluginLoaded(const std::string& name);
  
  /**
   * @brief 플러그인 언로드 시 발생하는 시그널
   * @param name 플러그인 이름
   */
  void pluginUnloaded(const std::string& name);

private:
  std::string plugin_dir_; ///< 플러그인 디렉토리 경로
  std::unordered_map<std::string, std::unique_ptr<QPluginLoader>> plugin_loaders_; ///< 플러그인 로더
  std::unordered_map<std::string, PluginInterface*> plugins_; ///< 로드된 플러그인
};

} // namespace robot_debugger_ui

#endif // PLUGIN_MANAGER_HPP 