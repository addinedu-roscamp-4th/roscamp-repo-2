#include "robot_debugger_ui/main_window.hpp"
#include <QApplication>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>
#include <iostream>
#include <QUiLoader>
#include <QFile>
#include <QLabel>
#include <QVBoxLayout>

namespace robot_debugger_ui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui_(nullptr)  // Qt Designer를 통해 생성된 UI를 로드하지 않으므로 nullptr로 설정
    , central_widget_(new QStackedWidget(this))
    , sidebar_(new QDockWidget("사이드바", this))
{
    initializeUi();
    initializeRos();

    // 윈도우 크기 및 제목 설정
    resize(1024, 768);
    setWindowTitle("로봇 디버깅 도구");
}

MainWindow::~MainWindow()
{
    // ui_ 포인터는 nullptr이므로 삭제하지 않음
    // 나머지 위젯들은 Qt가 부모-자식 관계에 따라 자동으로 해제
}

void MainWindow::initializeUi()
{
    // 중앙 위젯 설정
    setCentralWidget(central_widget_);

    // 상태바 설정
    setupStatusBar();

    // 사이드바 설정
    setupSidebar();

    // 탭 설정
    setupDashboardTab();
    setupJetcobotTab();
    setupPinkyTab();

    // 탭 전환 버튼 설정
    QToolBar* tabToolBar = new QToolBar("탭 전환", this);
    addToolBar(Qt::TopToolBarArea, tabToolBar);
    
    // 대시보드 탭 버튼
    QAction* dashboardAction = tabToolBar->addAction("대시보드");
    connect(dashboardAction, &QAction::triggered, [this]() {
        central_widget_->setCurrentIndex(0);
        statusBar()->showMessage("대시보드");
    });
    
    // Jetcobot 탭 버튼
    QAction* jetcobotAction = tabToolBar->addAction("Jetcobot");
    connect(jetcobotAction, &QAction::triggered, [this]() {
        central_widget_->setCurrentIndex(1);
        statusBar()->showMessage("Jetcobot 패널");
    });
    
    // Pinky 탭 버튼
    QAction* pinkyAction = tabToolBar->addAction("Pinky");
    connect(pinkyAction, &QAction::triggered, [this]() {
        central_widget_->setCurrentIndex(2);
        statusBar()->showMessage("Pinky 패널");
    });

    // 최초 탭 선택
    central_widget_->setCurrentIndex(0);
}

void MainWindow::initializeRos()
{
    // 설정 관리자 초기화
    config_manager_ = std::make_shared<ConfigurationManager>();
    
    // 메시지 저장소 초기화
    message_store_ = std::make_shared<MessageStore>();

    // 플러그인 관리자 초기화
    plugin_manager_ = std::make_shared<PluginManager>();

    // 토픽 관리자는 ROS 노드가 필요하므로 실제 구현에서는 노드 파라미터를 전달해야 함
    // 여기서는 간단히 nullptr로 초기화
    topic_manager_ = std::make_shared<TopicManager>(nullptr);

    // 시그널 연결
    connect(topic_manager_.get(), &TopicManager::messageReceived,
            this, &MainWindow::onMessageReceived);
}

void MainWindow::setupStatusBar()
{
    statusBar()->showMessage("준비 완료");
}

void MainWindow::setupSidebar()
{
    // 사이드바 설정
    sidebar_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    addDockWidget(Qt::LeftDockWidgetArea, sidebar_);
    
    // 사이드바의 내용 설정 (여기서는 간단히 빈 위젯으로 설정)
    QWidget* sidebar_content = new QWidget(sidebar_);
    sidebar_->setWidget(sidebar_content);
}

void MainWindow::setupDashboardTab()
{
    // 대시보드 UI 파일 로드
    QUiLoader loader;
    
    // 설치된 패키지 경로에서 UI 파일 찾기
    QString ui_file_path;
    
    // 1. 먼저 환경 변수에서 경로 찾기
    QString ament_prefix_path = QString::fromStdString(qgetenv("AMENT_PREFIX_PATH").toStdString());
    QStringList prefixes = ament_prefix_path.split(':');
    
    // 기본 경로 구성
    for (const QString& prefix : prefixes) {
        QString potential_path = prefix + "/share/robot_debugger_ui/ui/dashboard.ui";
        if (QFile::exists(potential_path)) {
            ui_file_path = potential_path;
            break;
        }
    }
    
    // 경로를 찾을 수 없는 경우 상대 경로 시도
    if (ui_file_path.isEmpty()) {
        ui_file_path = "../share/robot_debugger_ui/ui/dashboard.ui";
        if (!QFile::exists(ui_file_path)) {
            ui_file_path = "share/robot_debugger_ui/ui/dashboard.ui";
        }
    }
    
    // 경로 확인 및 로드
    QFile file(ui_file_path);
    statusBar()->showMessage("UI 파일 경로: " + ui_file_path);
    
    if (!file.open(QFile::ReadOnly)) {
        // UI 파일을 찾을 수 없는 경우 빈 위젯 사용
        QMessageBox::warning(this, "경고", "dashboard.ui 파일을 로드할 수 없습니다. 기본 대시보드를 표시합니다. 경로: " + ui_file_path);
        QWidget* dashboard = new QWidget(central_widget_);
        QLabel* label = new QLabel("기본 대시보드", dashboard);
        QVBoxLayout* layout = new QVBoxLayout(dashboard);
        layout->addWidget(label);
        central_widget_->addWidget(dashboard);
        return;
    }
    
    QWidget* dashboard = loader.load(&file, central_widget_);
    file.close();
    if (dashboard) {
        central_widget_->addWidget(dashboard);
        statusBar()->showMessage("대시보드 로드 완료: " + ui_file_path);
    } else {
        // UI 로드 실패 시 빈 위젯 사용
        QWidget* dashboard = new QWidget(central_widget_);
        QLabel* label = new QLabel("대시보드 로드 실패", dashboard);
        QVBoxLayout* layout = new QVBoxLayout(dashboard);
        layout->addWidget(label);
        central_widget_->addWidget(dashboard);
        statusBar()->showMessage("대시보드 로드 실패: " + ui_file_path);
    }
}

void MainWindow::setupJetcobotTab()
{
    // Jetcobot UI 파일 로드
    QUiLoader loader;
    
    // 설치된 패키지 경로에서 UI 파일 찾기
    QString ui_file_path;
    
    // 환경 변수에서 경로 찾기
    QString ament_prefix_path = QString::fromStdString(qgetenv("AMENT_PREFIX_PATH").toStdString());
    QStringList prefixes = ament_prefix_path.split(':');
    
    // 기본 경로 구성
    for (const QString& prefix : prefixes) {
        QString potential_path = prefix + "/share/robot_debugger_ui/ui/jetcobot_tab.ui";
        if (QFile::exists(potential_path)) {
            ui_file_path = potential_path;
            break;
        }
    }
    
    // 경로를 찾을 수 없는 경우 상대 경로 시도
    if (ui_file_path.isEmpty()) {
        ui_file_path = "../share/robot_debugger_ui/ui/jetcobot_tab.ui";
        if (!QFile::exists(ui_file_path)) {
            ui_file_path = "share/robot_debugger_ui/ui/jetcobot_tab.ui";
        }
    }
    
    // 경로 확인 및 로드
    QFile file(ui_file_path);
    
    if (!file.open(QFile::ReadOnly)) {
        // UI 파일을 찾을 수 없는 경우 빈 위젯 사용
        QWidget* jetcobot_tab = new QWidget(central_widget_);
        QLabel* label = new QLabel("기본 Jetcobot 패널", jetcobot_tab);
        QVBoxLayout* layout = new QVBoxLayout(jetcobot_tab);
        layout->addWidget(label);
        central_widget_->addWidget(jetcobot_tab);
        return;
    }
    
    QWidget* jetcobot_tab = loader.load(&file, central_widget_);
    file.close();
    if (jetcobot_tab) {
        central_widget_->addWidget(jetcobot_tab);
    } else {
        // UI 로드 실패 시 빈 위젯 사용
        QWidget* jetcobot_tab = new QWidget(central_widget_);
        QLabel* label = new QLabel("Jetcobot 패널 로드 실패", jetcobot_tab);
        QVBoxLayout* layout = new QVBoxLayout(jetcobot_tab);
        layout->addWidget(label);
        central_widget_->addWidget(jetcobot_tab);
    }
}

void MainWindow::setupPinkyTab()
{
    // Pinky UI 파일 로드
    QUiLoader loader;
    
    // 설치된 패키지 경로에서 UI 파일 찾기
    QString ui_file_path;
    
    // 환경 변수에서 경로 찾기
    QString ament_prefix_path = QString::fromStdString(qgetenv("AMENT_PREFIX_PATH").toStdString());
    QStringList prefixes = ament_prefix_path.split(':');
    
    // 기본 경로 구성
    for (const QString& prefix : prefixes) {
        QString potential_path = prefix + "/share/robot_debugger_ui/ui/pinky_tab.ui";
        if (QFile::exists(potential_path)) {
            ui_file_path = potential_path;
            break;
        }
    }
    
    // 경로를 찾을 수 없는 경우 상대 경로 시도
    if (ui_file_path.isEmpty()) {
        ui_file_path = "../share/robot_debugger_ui/ui/pinky_tab.ui";
        if (!QFile::exists(ui_file_path)) {
            ui_file_path = "share/robot_debugger_ui/ui/pinky_tab.ui";
        }
    }
    
    // 경로 확인 및 로드
    QFile file(ui_file_path);
    
    if (!file.open(QFile::ReadOnly)) {
        // UI 파일을 찾을 수 없는 경우 빈 위젯 사용
        QWidget* pinky_tab = new QWidget(central_widget_);
        QLabel* label = new QLabel("기본 Pinky 패널", pinky_tab);
        QVBoxLayout* layout = new QVBoxLayout(pinky_tab);
        layout->addWidget(label);
        central_widget_->addWidget(pinky_tab);
        return;
    }
    
    QWidget* pinky_tab = loader.load(&file, central_widget_);
    file.close();
    if (pinky_tab) {
        central_widget_->addWidget(pinky_tab);
    } else {
        // UI 로드 실패 시 빈 위젯 사용
        QWidget* pinky_tab = new QWidget(central_widget_);
        QLabel* label = new QLabel("Pinky 패널 로드 실패", pinky_tab);
        QVBoxLayout* layout = new QVBoxLayout(pinky_tab);
        layout->addWidget(label);
        central_widget_->addWidget(pinky_tab);
    }
}

void MainWindow::onReconnectClicked()
{
    statusBar()->showMessage("재연결 중...");
    // 재연결 로직 구현
}

void MainWindow::onEmergencyClicked()
{
    QMessageBox::critical(this, "긴급 정지", "모든 로봇에 긴급 정지 명령이 전송되었습니다.");
    // 긴급 정지 로직 구현
}

void MainWindow::onLogLevelChanged(int level)
{
    const char* level_names[] = {"DEBUG", "INFO", "WARN", "ERROR", "CRITICAL"};
    if (level >= 0 && level < 5) {
        statusBar()->showMessage(QString("로그 레벨 변경: %1").arg(level_names[level]));
    }
    // 로그 레벨 변경 로직 구현
}

void MainWindow::onMessageReceived(const std::string& topic, const std::vector<uint8_t>& msg)
{
    // 메시지 저장소에 메시지 저장
    message_store_->storeMessage(topic, msg);
    
    // 상태바 업데이트 (토픽 이름과 시간 표시)
    statusBar()->showMessage(QString("메시지 수신: %1").arg(topic.c_str()));
}

} // namespace robot_debugger_ui 