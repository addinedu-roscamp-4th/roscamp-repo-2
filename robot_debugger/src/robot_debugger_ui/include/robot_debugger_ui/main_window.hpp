#ifndef ROBOT_DEBUGGER_UI_MAIN_WINDOW_HPP_
#define ROBOT_DEBUGGER_UI_MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QStackedWidget>
#include <QDockWidget>
#include <QListWidget>
#include <QMap>
#include <QDateTime>
#include <QStandardItemModel>
#include <QCheckBox>
#include <QTreeWidgetItem>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "robot_debugger_ui/configuration_manager.hpp"
#include "robot_debugger_ui/message_store.hpp"
#include "robot_debugger_ui/plugin_manager.hpp"
#include "robot_debugger_ui/topic_manager.hpp"

namespace robot_debugger_ui {

// 로봇 유형 열거형
enum class RobotType {
    JETCOBOT = 0,
    PINKY = 1
};

// 로봇 정보 구조체
struct RobotInfo {
    QString name;
    RobotType type;
    int domain_id;
    QString namespace_name;
    bool connected{false};
    bool emergency_stop{false};
    float battery_level{0.8f};  // 기본값
    QMap<QString, QString> position;  // x, y, z 위치 정보
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(rclcpp::Node::SharedPtr node = nullptr, QWidget* parent = nullptr);
    virtual ~MainWindow();

private slots:
    void onReconnectClicked();
    void onEmergencyClicked();
    void onAddRobotClicked();
    void onRemoveRobotClicked();
    void onRobotSelectionChanged();
    void onSaveConfigClicked();
    void onRobotCheckStateChanged(int state);
    void onCoordsRealReceived(const std::string& topic, const std::vector<uint8_t>& msg);
    void onMessageReceived(const std::string& topic, const std::vector<uint8_t>& msg);
    void onLogLevelChanged(int level);
    void onScanTopicsClicked();
    void onTopicItemChanged(QTreeWidgetItem* item, int column);
    void onTopicFilterChanged(const QString& text);

private:
    void initializeUi();
    void initializeRos();
    
    void setupStatusBar();
    void setupSidebar();
    void setupDashboardTab();
    void setupJetcobotTab();
    void setupPinkyTab();
    void setupSettingsTab();
    
    void loadRobotConfigurations();
    void saveRobotConfigurations();
    QWidget* createRobotCard(const RobotInfo& robot_info);
    void updateRobotCard(const RobotInfo& robot_info);
    void updateRobotTabCheckboxes();
    
    // 이벤트 로깅 함수
    void logEvent(const QString& robotName, const QString& eventType, const QString& description);
    
private:
    // UI 위젯들
    QStackedWidget* central_widget_;
    QDockWidget* sidebar_;
    
    QWidget* dashboard_;
    QWidget* fleet_overview_tab_;
    QWidget* settings_tab_;
    
    QListWidget* robot_list_;
    
    // 각종 관리자 클래스
    std::shared_ptr<ConfigurationManager> config_manager_;
    std::shared_ptr<MessageStore> message_store_;
    std::shared_ptr<PluginManager> plugin_manager_;
    std::shared_ptr<TopicManager> topic_manager_;
    rclcpp::Node::SharedPtr node_;  // ROS 노드
    
    // 로봇 정보 관련 맵
    QMap<QString, RobotInfo> robots_;
    QMap<QString, QCheckBox*> robot_checkboxes_;
    QMap<QString, QWidget*> robot_cards_;
};

}  // namespace robot_debugger_ui

#endif  // ROBOT_DEBUGGER_UI_MAIN_WINDOW_HPP_ 