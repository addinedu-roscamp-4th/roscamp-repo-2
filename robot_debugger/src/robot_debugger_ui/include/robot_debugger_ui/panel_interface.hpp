#ifndef PANEL_INTERFACE_HPP
#define PANEL_INTERFACE_HPP

#include <QWidget>
#include <QString>
#include <QVariantMap>
#include <QIcon>
#include <memory>

namespace robot_debugger_ui {

/**
 * @brief 패널 인터페이스 클래스
 * 
 * 대시보드에 추가할 수 있는 패널 위젯의 인터페이스입니다.
 */
class PanelInterface : public QWidget {
  Q_OBJECT

public:
  /**
   * @brief 생성자
   * @param parent 부모 위젯
   */
  explicit PanelInterface(QWidget* parent = nullptr) : QWidget(parent) {}
  
  /**
   * @brief 소멸자
   */
  ~PanelInterface() override = default;

  /**
   * @brief 패널 ID 가져오기
   * @return 패널 ID
   */
  virtual QString id() const = 0;

  /**
   * @brief 패널 타이틀 가져오기
   * @return 패널 타이틀
   */
  virtual QString title() const = 0;

  /**
   * @brief 패널 설명 가져오기
   * @return 패널 설명
   */
  virtual QString description() const = 0;

  /**
   * @brief 패널 아이콘 가져오기
   * @return 패널 아이콘
   */
  virtual QIcon icon() const = 0;

  /**
   * @brief 패널 설정 가져오기
   * @return 패널 설정
   */
  virtual QVariantMap settings() const = 0;

  /**
   * @brief 패널 설정 저장
   * @param settings 패널 설정
   */
  virtual void saveSettings(const QVariantMap& settings) = 0;

  /**
   * @brief 패널 초기화
   * @param settings 패널 설정
   * @return 초기화 성공 여부
   */
  virtual bool initialize(const QVariantMap& settings) = 0;

  /**
   * @brief 패널 상태 갱신
   */
  virtual void update() = 0;

  /**
   * @brief 패널 정지
   */
  virtual void stop() = 0;

  /**
   * @brief 패널 종료
   */
  virtual void shutdown() = 0;

  /**
   * @brief 패널에 설정 대화상자가 있는지 여부
   * @return 설정 대화상자 존재 여부
   */
  virtual bool hasConfigDialog() const {
    return false;
  }

  /**
   * @brief 설정 대화상자 표시
   * @return 대화상자 수락 여부
   */
  virtual bool showConfigDialog() {
    return false;
  }

signals:
  /**
   * @brief 패널 설정 변경 시그널
   * @param settings 새 설정
   */
  void settingsChanged(const QVariantMap& settings);

  /**
   * @brief 패널 상태 변경 시그널
   * @param status 상태 메시지
   */
  void statusChanged(const QString& status);
};

} // namespace robot_debugger_ui

#endif // PANEL_INTERFACE_HPP 