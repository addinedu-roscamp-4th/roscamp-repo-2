#ifndef PLUGIN_MANAGER_HPP
#define PLUGIN_MANAGER_HPP

#include <QObject>
#include <QPluginLoader>
#include <QString>
#include <QDir>
#include <QMap>
#include <QVariant>
#include <string>
#include <vector>
#include <memory>
#include <map>

namespace robot_debugger_ui {

/**
 * @brief 플러그인 인터페이스
 * 
 * 플러그인이 구현해야 하는 인터페이스입니다.
 */
class PluginInterface {
public:
  virtual ~PluginInterface() = default;

  /**
   * @brief 플러그인 초기화
   * @param parent_widget 부모 위젯
   * @param settings 플러그인 설정
   * @return 초기화 성공 여부
   */
  virtual bool initialize(QWidget* parent_widget, const QVariantMap& settings) = 0;

  /**
   * @brief 플러그인 이름 가져오기
   * @return 플러그인 이름
   */
  virtual QString name() const = 0;

  /**
   * @brief 플러그인 버전 가져오기
   * @return 플러그인 버전
   */
  virtual QString version() const = 0;

  /**
   * @brief 플러그인 설명 가져오기
   * @return 플러그인 설명
   */
  virtual QString description() const = 0;

  /**
   * @brief 플러그인 위젯 가져오기
   * @return 플러그인 위젯
   */
  virtual QWidget* widget() = 0;

  /**
   * @brief 플러그인 설정 가져오기
   * @return 플러그인 설정
   */
  virtual QVariantMap settings() const = 0;

  /**
   * @brief 플러그인 종료
   */
  virtual void shutdown() = 0;
};

/**
 * @brief 플러그인 관리자 클래스
 * 
 * 플러그인을 로드하고 관리하는 클래스입니다.
 */
class PluginManager : public QObject {
  Q_OBJECT

public:
  /**
   * @brief 플러그인 정보 구조체
   */
  struct PluginInfo {
    QString id;              ///< 플러그인 ID
    QString name;            ///< 플러그인 이름
    QString version;         ///< 플러그인 버전
    QString description;     ///< 플러그인 설명
    QString file_path;       ///< 플러그인 파일 경로
    bool is_loaded;          ///< 로드 여부
    bool is_enabled;         ///< 활성화 여부
    QVariantMap settings;    ///< 플러그인 설정
  };

  /**
   * @brief 생성자
   * @param parent 부모 QObject
   */
  explicit PluginManager(QObject* parent = nullptr);
  
  /**
   * @brief 소멸자
   */
  ~PluginManager() override;

  /**
   * @brief 플러그인 디렉토리 검색
   * @param dir_path 플러그인 디렉토리 경로
   * @return 발견된 플러그인 정보 목록
   */
  std::vector<PluginInfo> scanPluginDirectory(const QString& dir_path);

  /**
   * @brief 플러그인 로드
   * @param plugin_id 플러그인 ID
   * @param parent_widget 부모 위젯
   * @return 로드된 플러그인 인터페이스
   */
  PluginInterface* loadPlugin(const QString& plugin_id, QWidget* parent_widget);

  /**
   * @brief 플러그인 언로드
   * @param plugin_id 플러그인 ID
   * @return 언로드 성공 여부
   */
  bool unloadPlugin(const QString& plugin_id);

  /**
   * @brief 플러그인 활성화
   * @param plugin_id 플러그인 ID
   * @return 활성화 성공 여부
   */
  bool enablePlugin(const QString& plugin_id);

  /**
   * @brief 플러그인 비활성화
   * @param plugin_id 플러그인 ID
   * @return 비활성화 성공 여부
   */
  bool disablePlugin(const QString& plugin_id);

  /**
   * @brief 플러그인 설정 저장
   * @param plugin_id 플러그인 ID
   * @param settings 플러그인 설정
   * @return 저장 성공 여부
   */
  bool savePluginSettings(const QString& plugin_id, const QVariantMap& settings);

  /**
   * @brief 플러그인 정보 가져오기
   * @param plugin_id 플러그인 ID
   * @return 플러그인 정보
   */
  PluginInfo getPluginInfo(const QString& plugin_id) const;

  /**
   * @brief 모든 플러그인 정보 가져오기
   * @return 플러그인 정보 목록
   */
  std::vector<PluginInfo> getAllPluginInfo() const;

  /**
   * @brief 플러그인 위젯 가져오기
   * @param plugin_id 플러그인 ID
   * @return 플러그인 위젯
   */
  QWidget* getPluginWidget(const QString& plugin_id) const;

  /**
   * @brief 플러그인 설정 가져오기
   * @param plugin_id 플러그인 ID
   * @return 플러그인 설정
   */
  QVariantMap getPluginSettings(const QString& plugin_id) const;

signals:
  /**
   * @brief 플러그인 로드 시그널
   * @param plugin_id 플러그인 ID
   * @param success 성공 여부
   */
  void pluginLoaded(const QString& plugin_id, bool success);

  /**
   * @brief 플러그인 언로드 시그널
   * @param plugin_id 플러그인 ID
   */
  void pluginUnloaded(const QString& plugin_id);

  /**
   * @brief 플러그인 활성화 시그널
   * @param plugin_id 플러그인 ID
   */
  void pluginEnabled(const QString& plugin_id);

  /**
   * @brief 플러그인 비활성화 시그널
   * @param plugin_id 플러그인 ID
   */
  void pluginDisabled(const QString& plugin_id);

  /**
   * @brief 플러그인 설정 변경 시그널
   * @param plugin_id 플러그인 ID
   */
  void pluginSettingsChanged(const QString& plugin_id);

private:
  /**
   * @brief 플러그인 정보 목록
   */
  std::map<QString, PluginInfo> plugin_infos_;

  /**
   * @brief 로드된 플러그인 로더
   */
  std::map<QString, QPluginLoader*> plugin_loaders_;

  /**
   * @brief 로드된 플러그인 인터페이스
   */
  std::map<QString, PluginInterface*> loaded_plugins_;

  /**
   * @brief 플러그인 설정 파일 저장
   * @return 저장 성공 여부
   */
  bool savePluginInfos();

  /**
   * @brief 플러그인 설정 파일 로드
   * @return 로드 성공 여부
   */
  bool loadPluginInfos();
};

} // namespace robot_debugger_ui

#endif // PLUGIN_MANAGER_HPP 