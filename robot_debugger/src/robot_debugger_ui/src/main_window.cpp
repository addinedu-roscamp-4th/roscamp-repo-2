#include "robot_debugger_ui/main_window.hpp"
#include <QApplication>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>
#include <QToolButton>  // QToolButton 헤더 추가
#include <iostream>
#include <QUiLoader>
#include <QFile>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QSpinBox>
#include <QPushButton>
#include <QGroupBox>
#include <QScrollArea>
#include <QFrame>
#include <QDebug>
#include <QProgressBar>
#include <QCheckBox>
#include <QInputDialog>
#include <QTabWidget>
#include <QPlainTextEdit>
#include <QStandardItemModel>
#include <QTableView>
#include <QHeaderView>  // 추가된 헤더
#include <QCoreApplication>  // 추가된 헤더
#include <algorithm>  // std::max 사용을 위해 추가
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QIcon>

namespace robot_debugger_ui {

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QMainWindow(parent)
    , central_widget_(new QStackedWidget(this))
    , sidebar_(new QDockWidget("로봇 목록", this))
    , dashboard_(nullptr)
    , fleet_overview_tab_(nullptr)
    , settings_tab_(nullptr)
    , robot_list_(new QListWidget())
    , node_(node)
{
    initializeUi();
    initializeRos();
    loadRobotConfigurations();

    // 기본 로봇 선택 (첫 번째 로봇이 있으면)
    if (robot_list_->count() > 0) {
        robot_list_->setCurrentRow(0);
        // 첫 번째 로봇 선택 이벤트 발생
        QListWidgetItem* firstRobotItem = robot_list_->item(0);
        if (firstRobotItem) {
            robot_list_->setCurrentItem(firstRobotItem);
            // 명시적으로 선택 이벤트 호출
            onRobotSelectionChanged();
            qDebug() << "기본 로봇 선택됨: " << firstRobotItem->text();
        }
    }

    // 윈도우 크기 및 제목 설정
    resize(1024, 768);
    setWindowTitle("로봇 디버깅 도구");
}

MainWindow::~MainWindow()
{
    saveRobotConfigurations();
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
    setupDashboardTab();  // 대시보드 탭 활성화 (플릿 오버뷰 포함)
    setupJetcobotTab();
    setupPinkyTab();
    setupSettingsTab();

    // 탭 전환 버튼 설정
    QToolBar* tabToolBar = new QToolBar("탭 전환", this);
    tabToolBar->setIconSize(QSize(24, 24));
    tabToolBar->setMovable(false);
    tabToolBar->setStyleSheet("QToolBar { spacing: 8px; padding: 5px; }");
    addToolBar(Qt::TopToolBarArea, tabToolBar);
    
    // 스타일 설정
    QString activeStyle = "QToolButton { border: none; border-bottom: 3px solid #2196F3; color: #2196F3; padding: 8px; font-weight: bold; }";
    QString inactiveStyle = "QToolButton { border: none; padding: 8px; color: #555; }";
    
    // 대시보드 탭 버튼
    QAction* dashboardAction = tabToolBar->addAction("대시보드");
    QToolButton* dashboardButton = qobject_cast<QToolButton*>(tabToolBar->widgetForAction(dashboardAction));
    if (dashboardButton) {
        dashboardButton->setStyleSheet(activeStyle); // 처음에는 활성화 상태
        dashboardButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    }
    
    // Jetcobot 탭 버튼
    QAction* jetcobotAction = tabToolBar->addAction("Jetcobot");
    QToolButton* jetcobotButton = qobject_cast<QToolButton*>(tabToolBar->widgetForAction(jetcobotAction));
    if (jetcobotButton) {
        jetcobotButton->setStyleSheet(inactiveStyle);
        jetcobotButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    }
    
    // Pinky 탭 버튼
    QAction* pinkyAction = tabToolBar->addAction("Pinky");
    QToolButton* pinkyButton = qobject_cast<QToolButton*>(tabToolBar->widgetForAction(pinkyAction));
    if (pinkyButton) {
        pinkyButton->setStyleSheet(inactiveStyle);
        pinkyButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    }
    
    // 설정 탭 버튼
    QAction* settingsAction = tabToolBar->addAction("설정");
    QToolButton* settingsButton = qobject_cast<QToolButton*>(tabToolBar->widgetForAction(settingsAction));
    if (settingsButton) {
        settingsButton->setStyleSheet(inactiveStyle);
        settingsButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    }
    
    // 클릭 이벤트 연결 및 스타일 업데이트
    connect(dashboardAction, &QAction::triggered, [=]() {
        central_widget_->setCurrentWidget(dashboard_);
        statusBar()->showMessage("대시보드");
        // 버튼 스타일 업데이트
        if (dashboardButton) dashboardButton->setStyleSheet(activeStyle);
        if (jetcobotButton) jetcobotButton->setStyleSheet(inactiveStyle);
        if (pinkyButton) pinkyButton->setStyleSheet(inactiveStyle);
        if (settingsButton) settingsButton->setStyleSheet(inactiveStyle);
    });
    
    connect(jetcobotAction, &QAction::triggered, [=]() {
        central_widget_->setCurrentIndex(1);  // 인덱스 수정
        statusBar()->showMessage("Jetcobot 패널");
        // 버튼 스타일 업데이트
        if (dashboardButton) dashboardButton->setStyleSheet(inactiveStyle);
        if (jetcobotButton) jetcobotButton->setStyleSheet(activeStyle);
        if (pinkyButton) pinkyButton->setStyleSheet(inactiveStyle);
        if (settingsButton) settingsButton->setStyleSheet(inactiveStyle);
    });
    
    connect(pinkyAction, &QAction::triggered, [=]() {
        central_widget_->setCurrentIndex(2);  // 인덱스 수정
        statusBar()->showMessage("Pinky 패널");
        // 버튼 스타일 업데이트
        if (dashboardButton) dashboardButton->setStyleSheet(inactiveStyle);
        if (jetcobotButton) jetcobotButton->setStyleSheet(inactiveStyle);
        if (pinkyButton) pinkyButton->setStyleSheet(activeStyle);
        if (settingsButton) settingsButton->setStyleSheet(inactiveStyle);
    });
    
    connect(settingsAction, &QAction::triggered, [=]() {
        central_widget_->setCurrentWidget(settings_tab_);
        statusBar()->showMessage("설정");
        // 버튼 스타일 업데이트
        if (dashboardButton) dashboardButton->setStyleSheet(inactiveStyle);
        if (jetcobotButton) jetcobotButton->setStyleSheet(inactiveStyle);
        if (pinkyButton) pinkyButton->setStyleSheet(inactiveStyle);
        if (settingsButton) settingsButton->setStyleSheet(activeStyle);
    });

    // 최초 탭 선택
    central_widget_->setCurrentWidget(dashboard_);
}

void MainWindow::initializeRos()
{
    try {
    // 설정 관리자 초기화
    config_manager_ = std::make_shared<ConfigurationManager>();
    
    // 메시지 저장소 초기화
    message_store_ = std::make_shared<MessageStore>();

    // 플러그인 관리자 초기화
    plugin_manager_ = std::make_shared<PluginManager>();

        // 토픽 관리자 초기화 (ROS 노드 전달)
        if (node_) {
            // 안전하게 ROS 노드 포인터 확인 후 전달
            topic_manager_ = std::make_shared<TopicManager>(node_, config_manager_);

    // 시그널 연결
    connect(topic_manager_.get(), &TopicManager::messageReceived,
            this, &MainWindow::onMessageReceived);
            
            // 토픽 스캐너 시작
            topic_manager_->startTopicScan(2000);  // 2초 간격으로 스캔
        } else {
            qWarning() << "ROS 노드가 초기화되지 않았습니다. 토픽 기능이 제한됩니다.";
            topic_manager_ = std::make_shared<TopicManager>(nullptr, config_manager_);
        }
    } catch (const std::exception& e) {
        qCritical() << "ROS 초기화 중 오류 발생:" << e.what();
        // 기본값으로 초기화하여 앱이 최소한으로 동작하도록 함
        if (!config_manager_) config_manager_ = std::make_shared<ConfigurationManager>();
        if (!message_store_) message_store_ = std::make_shared<MessageStore>();
        if (!plugin_manager_) plugin_manager_ = std::make_shared<PluginManager>();
        if (!topic_manager_) topic_manager_ = std::make_shared<TopicManager>(nullptr);
    }
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
    
    // 사이드바의 내용 설정 (로봇 목록과 체크박스)
    QWidget* sidebar_content = new QWidget(sidebar_);
    QVBoxLayout* sidebar_layout = new QVBoxLayout(sidebar_content);
    
    // 로봇 목록 상단에 라벨 추가
    QLabel* listLabel = new QLabel("로봇 선택");
    listLabel->setStyleSheet("font-weight: bold; font-size: 14px;");
    sidebar_layout->addWidget(listLabel);
    
    // 도움말 텍스트 추가
    QLabel* helpLabel = new QLabel("선택한 로봇이 각 탭에서 제어 대상이 됩니다.");
    helpLabel->setWordWrap(true);
    helpLabel->setStyleSheet("color: #555; margin-bottom: 10px;");
    sidebar_layout->addWidget(helpLabel);
    
    // 로봇 목록 및 체크박스 컨테이너
    QWidget* robotListContainer = new QWidget();
    QVBoxLayout* containerLayout = new QVBoxLayout(robotListContainer);
    containerLayout->setContentsMargins(0, 0, 0, 0);
    
    // 체크박스 영역
    QScrollArea* checkboxArea = new QScrollArea();
    checkboxArea->setWidgetResizable(true);
    QWidget* checkboxWidget = new QWidget();
    QVBoxLayout* checkboxLayout = new QVBoxLayout(checkboxWidget);
    
    // 체크박스 영역을 Jetcobot 섹션과 Pinky 섹션으로 구분
    QLabel* jetcobotLabel = new QLabel("Jetcobot");
    jetcobotLabel->setStyleSheet("font-weight: bold; margin-top: 10px;");
    checkboxLayout->addWidget(jetcobotLabel);
    
    // Jetcobot 체크박스 컨테이너
    QWidget* jetcobotContainer = new QWidget();
    QVBoxLayout* jetcobotLayout = new QVBoxLayout(jetcobotContainer);
    jetcobotLayout->setContentsMargins(10, 0, 0, 0);
    jetcobotLayout->setSpacing(5);
    checkboxLayout->addWidget(jetcobotContainer);
    
    // Pinky 레이블
    QLabel* pinkyLabel = new QLabel("Pinky");
    pinkyLabel->setStyleSheet("font-weight: bold; margin-top: 10px;");
    checkboxLayout->addWidget(pinkyLabel);
    
    // Pinky 체크박스 컨테이너
    QWidget* pinkyContainer = new QWidget();
    QVBoxLayout* pinkyLayout = new QVBoxLayout(pinkyContainer);
    pinkyLayout->setContentsMargins(10, 0, 0, 0);
    pinkyLayout->setSpacing(5);
    checkboxLayout->addWidget(pinkyContainer);
    
    // 전체 선택/해제 컨테이너
    QWidget* selectAllContainer = new QWidget();
    QHBoxLayout* selectAllLayout = new QHBoxLayout(selectAllContainer);
    selectAllLayout->setContentsMargins(0, 10, 0, 0);
    
    QPushButton* selectAllButton = new QPushButton("모두 선택");
    QPushButton* deselectAllButton = new QPushButton("모두 해제");
    
    selectAllButton->setStyleSheet("padding: 3px 8px;");
    deselectAllButton->setStyleSheet("padding: 3px 8px;");
    
    connect(selectAllButton, &QPushButton::clicked, [this]() {
        for (auto it = robot_checkboxes_.begin(); it != robot_checkboxes_.end(); ++it) {
            it.value()->setChecked(true);
        }
    });
    
    connect(deselectAllButton, &QPushButton::clicked, [this]() {
        for (auto it = robot_checkboxes_.begin(); it != robot_checkboxes_.end(); ++it) {
            it.value()->setChecked(false);
        }
    });
    
    selectAllLayout->addWidget(selectAllButton);
    selectAllLayout->addWidget(deselectAllButton);
    
    checkboxLayout->addWidget(selectAllContainer);
    
    // 여백 추가
    checkboxLayout->addStretch();
    
    checkboxArea->setWidget(checkboxWidget);
    containerLayout->addWidget(checkboxArea);
    
    sidebar_layout->addWidget(robotListContainer);
    
    // 버튼 영역
    QWidget* buttonWidget = new QWidget();
    QHBoxLayout* buttonLayout = new QHBoxLayout(buttonWidget);
    
    QPushButton* reconnectButton = new QPushButton("재연결");
    QPushButton* emergencyButton = new QPushButton("긴급 정지");
    
    reconnectButton->setStyleSheet("padding: 6px 10px;");
    emergencyButton->setStyleSheet("background-color: #f44336; color: white; padding: 6px 10px;");
    
    buttonLayout->addWidget(reconnectButton);
    buttonLayout->addWidget(emergencyButton);
    
    connect(reconnectButton, &QPushButton::clicked, this, &MainWindow::onReconnectClicked);
    connect(emergencyButton, &QPushButton::clicked, this, &MainWindow::onEmergencyClicked);
    
    sidebar_layout->addWidget(buttonWidget);
    
    sidebar_->setWidget(sidebar_content);
}

void MainWindow::setupDashboardTab()
{
    // 대시보드 탭 생성
    QWidget* dashboard = new QWidget(central_widget_);
    QVBoxLayout* main_layout = new QVBoxLayout(dashboard);
    
    // 제목
    QLabel* title = new QLabel("대시보드");
    title->setStyleSheet("font-size: 18px; font-weight: bold;");
    main_layout->addWidget(title);
    
    // 대시보드 탭 위젯 생성
    QTabWidget* dashboard_tabs = new QTabWidget();
    
    // 탭 위젯 스타일 개선 - 동일한 스타일을 다른 탭 위젯에도 적용하기 위해 분리
    dashboard_tabs->setStyleSheet(
        "QTabWidget::pane { margin-top: 8px; border: 1px solid #ddd; }"
        "QTabBar::tab { height: 30px; padding: 5px 12px; background-color: #f0f0f0; border: 1px solid #ddd; border-bottom: none; margin-right: 2px; }"
        "QTabBar::tab:selected { background-color: white; border-bottom: 3px solid #2196F3; font-weight: bold; }"
        "QTabBar::tab:hover:!selected { background-color: #e0e0e0; }"
    );
    
    main_layout->addWidget(dashboard_tabs);
    
    // 1. 플릿 오버뷰 탭 추가
    QWidget* fleet_overview_tab = new QWidget();
    QVBoxLayout* fleet_layout = new QVBoxLayout(fleet_overview_tab);
    
    // 부제목 제거 (불필요한 중복 제거)
    
    // 로봇 카드 컨테이너 영역
    QScrollArea* cardsScrollArea = new QScrollArea();
    cardsScrollArea->setWidgetResizable(true);
    QWidget* cardsContainer = new QWidget();
    
    // 그리드 레이아웃으로 변경
    QGridLayout* cardsLayout = new QGridLayout(cardsContainer);
    cardsLayout->setContentsMargins(10, 10, 10, 10);
    cardsLayout->setSpacing(15);
    
    // 카드 추가는 나중에 로봇 정보가 로드된 후 수행
    
    cardsScrollArea->setWidget(cardsContainer);
    fleet_layout->addWidget(cardsScrollArea);
    
    // 2. 글로벌 로그 탭 추가
    QWidget* log_tab = new QWidget();
    QVBoxLayout* log_layout = new QVBoxLayout(log_tab);
    
    // 로그 필터 컨트롤
    QHBoxLayout* log_filter_layout = new QHBoxLayout();
    
    QLabel* log_level_label = new QLabel("로그 레벨:");
    QComboBox* log_level_combo = new QComboBox();
    log_level_combo->addItem("디버그", 0);
    log_level_combo->addItem("정보", 1);
    log_level_combo->addItem("경고", 2);
    log_level_combo->addItem("오류", 3);
    log_level_combo->addItem("심각", 4);
    
    // 현재 로그 레벨 설정
    if (config_manager_) {
    log_level_combo->setCurrentIndex(config_manager_->getLogLevel());
    } else {
        log_level_combo->setCurrentIndex(1);  // 기본값은 정보 레벨
    }
    
    connect(log_level_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onLogLevelChanged);
    
    log_filter_layout->addWidget(log_level_label);
    log_filter_layout->addWidget(log_level_combo);
    
    // 검색 필드
    QLabel* search_label = new QLabel("검색:");
    QLineEdit* search_edit = new QLineEdit();
    search_edit->setPlaceholderText("로그 검색...");
    
    log_filter_layout->addWidget(search_label);
    log_filter_layout->addWidget(search_edit);
    log_filter_layout->addStretch();
    
    // 로그 필터 버튼
    QPushButton* clear_log_button = new QPushButton("로그 지우기");
    connect(clear_log_button, &QPushButton::clicked, [this]() {
        // 로그 지우기 로직
        QPlainTextEdit* log_text_edit = dashboard_->findChild<QPlainTextEdit*>("logTextEdit");
        if (log_text_edit) {
            log_text_edit->clear();
        }
    });
    
    log_filter_layout->addWidget(clear_log_button);
    
    log_layout->addLayout(log_filter_layout);
    
    // 로그 텍스트 영역
    QPlainTextEdit* log_text_edit = new QPlainTextEdit();
    log_text_edit->setObjectName("logTextEdit");
    log_text_edit->setReadOnly(true);
    log_text_edit->setFont(QFont("Monospace", 9));
    log_text_edit->setLineWrapMode(QPlainTextEdit::NoWrap);
    
    // 기본 로그 메시지 추가
    log_text_edit->appendPlainText("[INFO] 로봇 디버깅 도구 시작됨");
    log_text_edit->appendPlainText("[INFO] ROS 2 환경 초기화 완료");
    
    log_layout->addWidget(log_text_edit);
    
    // 3. 이벤트 탭 추가
    QWidget* event_tab = new QWidget();
    QVBoxLayout* event_layout = new QVBoxLayout(event_tab);
    
    // 이벤트 필터 컨트롤
    QHBoxLayout* event_filter_layout = new QHBoxLayout();
    
    QLabel* event_type_label = new QLabel("이벤트 유형:");
    QComboBox* event_type_combo = new QComboBox();
    event_type_combo->addItem("모든 이벤트", 0);
    event_type_combo->addItem("연결 이벤트", 1);
    event_type_combo->addItem("장애 이벤트", 2);
    event_type_combo->addItem("명령 이벤트", 3);
    
    event_filter_layout->addWidget(event_type_label);
    event_filter_layout->addWidget(event_type_combo);
    
    // 로봇 필터
    QLabel* robot_filter_label = new QLabel("로봇 필터:");
    QComboBox* robot_filter_combo = new QComboBox();
    robot_filter_combo->addItem("모든 로봇", 0);
    
    event_filter_layout->addWidget(robot_filter_label);
    event_filter_layout->addWidget(robot_filter_combo);
    event_filter_layout->addStretch();
    
    // 이벤트 필터 버튼
    QPushButton* clear_events_button = new QPushButton("이벤트 지우기");
    connect(clear_events_button, &QPushButton::clicked, [this]() {
        // 이벤트 지우기 로직
        QTableView* event_table = dashboard_->findChild<QTableView*>("eventTableView");
        if (event_table && event_table->model()) {
            QAbstractItemModel* model = event_table->model();
            model->removeRows(0, model->rowCount());
        }
    });
    
    event_filter_layout->addWidget(clear_events_button);
    
    event_layout->addLayout(event_filter_layout);
    
    // 이벤트 테이블
    QTableView* event_table = new QTableView();
    event_table->setObjectName("eventTableView");
    event_table->setAlternatingRowColors(true);
    event_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    event_table->horizontalHeader()->setStretchLastSection(true);
    
    // 이벤트 테이블 모델 설정 (간단한 예시)
    QStandardItemModel* event_model = new QStandardItemModel(0, 4, event_table);
    if (event_model) {
    event_model->setHorizontalHeaderLabels(QStringList() << "시간" << "로봇" << "유형" << "설명");
    
    // 예시 이벤트 데이터 추가
    QList<QStandardItem*> event1;
    event1.append(new QStandardItem(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")));
    event1.append(new QStandardItem("Jetcobot_1"));
    event1.append(new QStandardItem("연결"));
    event1.append(new QStandardItem("로봇이 연결되었습니다."));
    event_model->appendRow(event1);
    
    QList<QStandardItem*> event2;
    event2.append(new QStandardItem(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")));
    event2.append(new QStandardItem("Pinky_1"));
    event2.append(new QStandardItem("명령"));
    event2.append(new QStandardItem("경로 이동 명령이 실행되었습니다."));
    event_model->appendRow(event2);
    
    event_table->setModel(event_model);
    }
    
    event_layout->addWidget(event_table);
    
    // 탭에 추가
    dashboard_tabs->addTab(fleet_overview_tab, "플릿 오버뷰");
    dashboard_tabs->addTab(log_tab, "글로벌 로그");
    dashboard_tabs->addTab(event_tab, "이벤트");
    
    // 대시보드 위젯을 중앙 위젯에 추가
    central_widget_->addWidget(dashboard);
    
    // 대시보드 멤버 변수에 저장
    dashboard_ = dashboard;
    
    // 플릿 오버뷰 탭을 멤버 변수로 저장 (기존 fleet_overview_tab_ 대체)
    fleet_overview_tab_ = fleet_overview_tab;
}

void MainWindow::setupJetcobotTab()
{
    // Jetcobot UI 파일 로드
    QUiLoader loader;
    
    // 여러 가능한 경로에서 UI 파일 찾기
    QStringList uiPaths;
    
    // 1. 환경 변수에서 설정된 경로
    QString qtUiPath = QString::fromLocal8Bit(qgetenv("QT_UI_PATH"));
    if (!qtUiPath.isEmpty()) {
        uiPaths << qtUiPath;
    }
    
    // 2. 애플리케이션 디렉토리 기준 경로
    uiPaths << QCoreApplication::applicationDirPath() + "/../share/robot_debugger_ui/ui";
    
    // 3. AMENT_PREFIX_PATH 경로 기반 검색
    QString ament_prefix_path = QString::fromLocal8Bit(qgetenv("AMENT_PREFIX_PATH"));
    QStringList prefixes = ament_prefix_path.split(':');
    for (const QString& prefix : prefixes) {
        uiPaths << prefix + "/share/robot_debugger_ui/ui";
    }
    
    // 4. 상대 경로 추가
    uiPaths << QDir::current().absolutePath() + "/install/robot_debugger_ui/share/robot_debugger_ui/ui";
    uiPaths << QDir::current().absolutePath() + "/src/robot_debugger_ui/ui";
    
    // UI 파일 검색
    QString ui_file_path;
    for (const QString& path : uiPaths) {
        QString potential_path = path + "/jetcobot_tab.ui";
        if (QFile::exists(potential_path)) {
            ui_file_path = potential_path;
            qDebug() << "Jetcobot UI 파일 찾음: " << potential_path;
            break;
        }
    }
    
    // 경로 확인 및 로드
    QFile file(ui_file_path);
    qDebug() << "Jetcobot UI 파일 로드 시도: " << ui_file_path;
    
    if (!file.open(QFile::ReadOnly)) {
        // UI 파일을 찾을 수 없는 경우 빈 위젯 사용
        qDebug() << "Jetcobot UI 파일을 열 수 없음: " << file.errorString();
        qDebug() << "시도한 경로들:";
        for (const QString& path : uiPaths) {
            qDebug() << " - " << path;
        }
        
        QWidget* jetcobot_tab = new QWidget(central_widget_);
        QLabel* label = new QLabel("기본 Jetcobot 패널 (UI 파일을 찾을 수 없음)", jetcobot_tab);
        QVBoxLayout* layout = new QVBoxLayout(jetcobot_tab);
        layout->addWidget(label);
        central_widget_->addWidget(jetcobot_tab);
        return;
    }
    
    QWidget* jetcobot_tab = loader.load(&file, central_widget_);
    file.close();
    if (jetcobot_tab) {
        central_widget_->addWidget(jetcobot_tab);
        qDebug() << "Jetcobot UI 파일 로드 성공";
        
        // 활성화된 로봇 표시 레이블 제거 (요청에 따라 제거)
        QLabel* activeRobotsLabel = jetcobot_tab->findChild<QLabel*>("activeRobotsLabel");
        if (activeRobotsLabel) {
            activeRobotsLabel->hide();
        }
        
        // 긴급 정지 버튼 제거 (요청에 따라 제거)
        QPushButton* emergencyBtn = jetcobot_tab->findChild<QPushButton*>("pushButtonEmergency");
        if (emergencyBtn) {
            emergencyBtn->hide();
        }
        
        // UI에서 제목 라벨 찾기
        QLabel* titleLabel = jetcobot_tab->findChild<QLabel*>("label");
        if (titleLabel && titleLabel->text() == "Jetcobot 제어 패널") {
            // hide 제거하고 보이게 설정
            titleLabel->show();
            titleLabel->setVisible(true);
            
            // 제목 스타일 개선
            QFont font = titleLabel->font();
            font.setPointSize(16);
            font.setBold(true);
            titleLabel->setFont(font);
            titleLabel->setStyleSheet("margin-top: 10px; margin-bottom: 10px; padding: 5px;");
            titleLabel->setContentsMargins(5, 5, 5, 5);
            titleLabel->setMaximumHeight(40);
        }
        
        // 적절한 여백 유지
        QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(jetcobot_tab->layout());
        if (mainLayout) {
            // 상단 여백 복원
            mainLayout->setContentsMargins(10, 10, 10, 10);
            
            // 레이아웃 내 위젯 간 간격 적절히 설정
            mainLayout->setSpacing(8); // 간격을 8로 설정
            
            // 스페이서 위젯 조정
            for (int i = 0; i < mainLayout->count(); ++i) {
                QLayoutItem* item = mainLayout->itemAt(i);
                QSpacerItem* spacer = dynamic_cast<QSpacerItem*>(item);
                if (spacer) {
                    // 스페이서 크기 조정 대신 유지
                    continue;
                }
                
                QWidget* widget = item->widget();
                if (widget) {
                    // 각 위젯 여백 설정
                    widget->setContentsMargins(2, 2, 2, 2);
                    
                    // 위젯 레이아웃 여백 조정
                    if (widget->layout()) {
                        widget->layout()->setContentsMargins(2, 2, 2, 2);
                        widget->layout()->setSpacing(8);
                    }
                }
            }
            
            // 첫 번째 탭 위젯 찾기 (위치 제어, 경로 탐색 등 탭을 포함하는 위젯)
            QTabWidget* tabWidget = jetcobot_tab->findChild<QTabWidget*>();
            if (tabWidget) {
                // 탭 위젯 여백 설정
                tabWidget->setContentsMargins(5, 5, 5, 5);
                
                // 대시보드 스타일과 유사하게 선택된 탭 디자인 개선
                tabWidget->setStyleSheet(
                    "QTabWidget::pane { margin-top: 8px; border: 1px solid #ddd; }"
                    "QTabBar::tab { height: 30px; padding: 5px 12px; background-color: #f0f0f0; border: 1px solid #ddd; border-bottom: none; margin-right: 2px; }"
                    "QTabBar::tab:selected { background-color: white; border-bottom: 3px solid #2196F3; font-weight: bold; }"
                    "QTabBar::tab:hover:!selected { background-color: #e0e0e0; }"
                );
                
                // 탭 위젯 위치 조정 (제목과 적절히 분리되도록)
                QRect geo = tabWidget->geometry();
                geo.moveTop(std::max(0, geo.top() + 15)); // 위치 아래로 조정
                tabWidget->setGeometry(geo);
            }
        }
    } else {
        // UI 로드 실패 시 빈 위젯 사용
        qDebug() << "Jetcobot UI 파일 로드 실패: " << loader.errorString();
        QWidget* jetcobot_tab = new QWidget(central_widget_);
        QLabel* label = new QLabel("Jetcobot 패널 로드 실패: " + loader.errorString(), jetcobot_tab);
        QVBoxLayout* layout = new QVBoxLayout(jetcobot_tab);
        layout->addWidget(label);
        central_widget_->addWidget(jetcobot_tab);
    }
}

void MainWindow::setupPinkyTab()
{
    // Pinky UI 파일 로드
    QUiLoader loader;
    
    // 여러 가능한 경로에서 UI 파일 찾기
    QStringList uiPaths;
    
    // 1. 환경 변수에서 설정된 경로
    QString qtUiPath = QString::fromLocal8Bit(qgetenv("QT_UI_PATH"));
    if (!qtUiPath.isEmpty()) {
        uiPaths << qtUiPath;
    }
    
    // 2. 애플리케이션 디렉토리 기준 경로
    uiPaths << QCoreApplication::applicationDirPath() + "/../share/robot_debugger_ui/ui";
    
    // 3. AMENT_PREFIX_PATH 경로 기반 검색
    QString ament_prefix_path = QString::fromLocal8Bit(qgetenv("AMENT_PREFIX_PATH"));
    QStringList prefixes = ament_prefix_path.split(':');
    for (const QString& prefix : prefixes) {
        uiPaths << prefix + "/share/robot_debugger_ui/ui";
    }
    
    // 4. 상대 경로 추가
    uiPaths << QDir::current().absolutePath() + "/install/robot_debugger_ui/share/robot_debugger_ui/ui";
    uiPaths << QDir::current().absolutePath() + "/src/robot_debugger_ui/ui";
    
    // UI 파일 검색
    QString ui_file_path;
    for (const QString& path : uiPaths) {
        QString potential_path = path + "/pinky_tab.ui";
        if (QFile::exists(potential_path)) {
            ui_file_path = potential_path;
            qDebug() << "Pinky UI 파일 찾음: " << potential_path;
            break;
        }
    }
    
    // 경로 확인 및 로드
    QFile file(ui_file_path);
    qDebug() << "Pinky UI 파일 로드 시도: " << ui_file_path;
    
    if (!file.open(QFile::ReadOnly)) {
        // UI 파일을 찾을 수 없는 경우 빈 위젯 사용
        qDebug() << "Pinky UI 파일을 열 수 없음: " << file.errorString();
        qDebug() << "시도한 경로들:";
        for (const QString& path : uiPaths) {
            qDebug() << " - " << path;
        }
        
        QWidget* pinky_tab = new QWidget(central_widget_);
        QLabel* label = new QLabel("기본 Pinky 패널 (UI 파일을 찾을 수 없음)", pinky_tab);
        QVBoxLayout* layout = new QVBoxLayout(pinky_tab);
        layout->addWidget(label);
        central_widget_->addWidget(pinky_tab);
        return;
    }
    
    QWidget* pinky_tab = loader.load(&file, central_widget_);
    file.close();
    if (pinky_tab) {
        central_widget_->addWidget(pinky_tab);
        qDebug() << "Pinky UI 파일 로드 성공";
        
        // 활성화된 로봇 표시 레이블 제거 (요청에 따라 제거)
        QLabel* activeRobotsLabel = pinky_tab->findChild<QLabel*>("activeRobotsLabel");
        if (activeRobotsLabel) {
            activeRobotsLabel->hide();
        }
        
        // 긴급 정지 버튼 제거 (요청에 따라 제거)
        QPushButton* emergencyBtn = pinky_tab->findChild<QPushButton*>("pushButtonEmergency");
        if (emergencyBtn) {
            emergencyBtn->hide();
        }
        
        // UI에서 제목 라벨 찾기
        QLabel* titleLabel = pinky_tab->findChild<QLabel*>("label");
        if (titleLabel && titleLabel->text() == "Pinky 제어 패널") {
            // hide 제거하고 보이게 설정
            titleLabel->show();
            titleLabel->setVisible(true);
            
            // 제목 스타일 개선
            QFont font = titleLabel->font();
            font.setPointSize(16);
            font.setBold(true);
            titleLabel->setFont(font);
            titleLabel->setStyleSheet("margin-top: 10px; margin-bottom: 10px; padding: 5px;");
            titleLabel->setContentsMargins(5, 5, 5, 5);
            titleLabel->setMaximumHeight(40);
        }
        
        // 적절한 여백 유지
        QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(pinky_tab->layout());
        if (mainLayout) {
            // 상단 여백 복원
            mainLayout->setContentsMargins(10, 10, 10, 10);
            
            // 레이아웃 내 위젯 간 간격 적절히 설정
            mainLayout->setSpacing(8); // 간격을 8로 설정
            
            // 스페이서 위젯 조정
            for (int i = 0; i < mainLayout->count(); ++i) {
                QLayoutItem* item = mainLayout->itemAt(i);
                QSpacerItem* spacer = dynamic_cast<QSpacerItem*>(item);
                if (spacer) {
                    // 스페이서 크기 조정 대신 유지
                    continue;
                }
                
                QWidget* widget = item->widget();
                if (widget) {
                    // 각 위젯 여백 설정
                    widget->setContentsMargins(2, 2, 2, 2);
                    
                    // 위젯 레이아웃 여백 조정
                    if (widget->layout()) {
                        widget->layout()->setContentsMargins(2, 2, 2, 2);
                        widget->layout()->setSpacing(8);
                    }
                }
            }
            
            // 첫 번째 탭 위젯 찾기 (맵 편집기, 경로 컨트롤러 등 탭을 포함하는 위젯)
            QTabWidget* tabWidget = pinky_tab->findChild<QTabWidget*>();
            if (tabWidget) {
                // 탭 위젯 여백 설정
                tabWidget->setContentsMargins(5, 5, 5, 5);
                
                // 대시보드 스타일과 유사하게 선택된 탭 디자인 개선
                tabWidget->setStyleSheet(
                    "QTabWidget::pane { margin-top: 8px; border: 1px solid #ddd; }"
                    "QTabBar::tab { height: 30px; padding: 5px 12px; background-color: #f0f0f0; border: 1px solid #ddd; border-bottom: none; margin-right: 2px; }"
                    "QTabBar::tab:selected { background-color: white; border-bottom: 3px solid #2196F3; font-weight: bold; }"
                    "QTabBar::tab:hover:!selected { background-color: #e0e0e0; }"
                );
                
                // 탭 위젯 위치 조정 (제목과 적절히 분리되도록)
                QRect geo = tabWidget->geometry();
                geo.moveTop(std::max(0, geo.top() + 15)); // 위치 아래로 조정
                tabWidget->setGeometry(geo);
            }
        }
    } else {
        // UI 로드 실패 시 빈 위젯 사용
        qDebug() << "Pinky UI 파일 로드 실패: " << loader.errorString();
        QWidget* pinky_tab = new QWidget(central_widget_);
        QLabel* label = new QLabel("Pinky 패널 로드 실패: " + loader.errorString(), pinky_tab);
        QVBoxLayout* layout = new QVBoxLayout(pinky_tab);
        layout->addWidget(label);
        central_widget_->addWidget(pinky_tab);
    }
}

void MainWindow::setupSettingsTab()
{
    // 설정 탭 생성
    settings_tab_ = new QWidget();
    QVBoxLayout* main_layout = new QVBoxLayout(settings_tab_);
    
    // 제목
    QLabel* title = new QLabel("설정");
    title->setStyleSheet("font-size: 18px; font-weight: bold;");
    main_layout->addWidget(title);
    
    // 설정 탭 내부에 탭 위젯 생성
    QTabWidget* settings_tabs = new QTabWidget();
    settings_tabs->setObjectName("settingsTabWidget");
    
    // 탭 위젯 스타일 설정 (대시보드와 동일)
    settings_tabs->setStyleSheet(
        "QTabWidget::pane { margin-top: 8px; border: 1px solid #ddd; }"
        "QTabBar::tab { height: 30px; padding: 5px 12px; background-color: #f0f0f0; border: 1px solid #ddd; border-bottom: none; margin-right: 2px; }"
        "QTabBar::tab:selected { background-color: white; border-bottom: 3px solid #2196F3; font-weight: bold; }"
        "QTabBar::tab:hover:!selected { background-color: #e0e0e0; }"
    );
    
    // 1. 로봇 설정 탭
    QWidget* robotSettingsTab = new QWidget();
    QVBoxLayout* robotSettingsLayout = new QVBoxLayout(robotSettingsTab);
    
    // 로봇 목록과 설정 영역을 포함하는 컨테이너
    QWidget* settingsContainer = new QWidget();
    settingsContainer->setObjectName("settingsContainer");
    QHBoxLayout* containerLayout = new QHBoxLayout(settingsContainer);
    
    // 왼쪽: 로봇 목록
    QGroupBox* robotListGroup = new QGroupBox("로봇 목록");
    QVBoxLayout* listLayout = new QVBoxLayout(robotListGroup);
    
    // 도움말 레이블 추가
    QLabel* helpLabel = new QLabel("로봇을 선택하여 설정을 확인하고 수정할 수 있습니다.");
    helpLabel->setStyleSheet("color: #555;");
    helpLabel->setWordWrap(true);
    listLayout->addWidget(helpLabel);
    
    // 로봇 목록 위젯
    robot_list_ = new QListWidget();
    robot_list_->setSelectionMode(QAbstractItemView::SingleSelection);
    robot_list_->setStyleSheet("QListWidget::item { padding: 6px; }");
    listLayout->addWidget(robot_list_);
    
    // 로봇 추가/제거 버튼
    QWidget* listButtonsWidget = new QWidget();
    QHBoxLayout* listButtonsLayout = new QHBoxLayout(listButtonsWidget);
    
    QPushButton* addButton = new QPushButton("추가");
    QPushButton* removeButton = new QPushButton("제거");
    
    listButtonsLayout->addWidget(addButton);
    listButtonsLayout->addWidget(removeButton);
    
    connect(addButton, &QPushButton::clicked, this, &MainWindow::onAddRobotClicked);
    connect(removeButton, &QPushButton::clicked, this, &MainWindow::onRemoveRobotClicked);
    
    // 시그널 연결을 itemClicked로 변경 (currentItemChanged 대신)
    connect(robot_list_, &QListWidget::itemClicked, this, &MainWindow::onRobotSelectionChanged);
    // 더블 클릭 시에도 설정 표시 처리
    connect(robot_list_, &QListWidget::itemDoubleClicked, this, &MainWindow::onRobotSelectionChanged);
    
    listLayout->addWidget(listButtonsWidget);
    
    // 오른쪽: 로봇 설정 필드 (처음에는 비어 있고, 로봇 선택 시 내용이 채워짐)
    QGroupBox* settingsGroup = new QGroupBox("로봇 설정");
    settingsGroup->setObjectName("settingsGroup");
    QFormLayout* formLayout = new QFormLayout(settingsGroup);
    
    // 로봇을 선택하라는 메시지 표시
    QLabel* selectRobotLabel = new QLabel("왼쪽에서 로봇을 선택하면 설정이 표시됩니다.");
    selectRobotLabel->setObjectName("selectRobotLabel");
    selectRobotLabel->setAlignment(Qt::AlignCenter);
    selectRobotLabel->setStyleSheet("color: #555;");
    formLayout->addRow(selectRobotLabel);
    
    // 빈 폼 위젯 미리 생성
    QLineEdit* nameEdit = new QLineEdit();
    nameEdit->setObjectName("nameEdit");
    nameEdit->setEnabled(false);
    
    QComboBox* typeCombo = new QComboBox();
    typeCombo->setObjectName("typeCombo");
    typeCombo->addItem("Jetcobot", static_cast<int>(RobotType::JETCOBOT));
    typeCombo->addItem("Pinky", static_cast<int>(RobotType::PINKY));
    typeCombo->setEnabled(false);
    
    QSpinBox* domainIdSpin = new QSpinBox();
    domainIdSpin->setObjectName("domainIdSpin");
    domainIdSpin->setRange(0, 232);
    domainIdSpin->setEnabled(false);
    
    QLineEdit* namespaceEdit = new QLineEdit();
    namespaceEdit->setObjectName("namespaceEdit");
    namespaceEdit->setEnabled(false);
    
    formLayout->addRow("이름:", nameEdit);
    formLayout->addRow("유형:", typeCombo);
    formLayout->addRow("도메인 ID:", domainIdSpin);
    formLayout->addRow("네임스페이스:", namespaceEdit);
    
    // 저장 버튼
    QPushButton* saveButton = new QPushButton("설정 저장");
    saveButton->setEnabled(false);
    saveButton->setObjectName("saveButton");
    connect(saveButton, &QPushButton::clicked, this, &MainWindow::onSaveConfigClicked);
    
    formLayout->addRow("", saveButton);
    
    // 로봇 목록 및 설정 필드 컨테이너에 추가
    containerLayout->addWidget(robotListGroup, 1);
    containerLayout->addWidget(settingsGroup, 2);
    
    robotSettingsLayout->addWidget(settingsContainer);
    
    // 2. 토픽 설정 탭
    QWidget* topicSettingsTab = new QWidget();
    QVBoxLayout* topicSettingsLayout = new QVBoxLayout(topicSettingsTab);
    
    // 토픽 설정 영역 생성
    QGroupBox* topicGroup = new QGroupBox("토픽 구독 설정");
    QVBoxLayout* topicLayout = new QVBoxLayout(topicGroup);
    
    // 도움말 레이블
    QLabel* topicHelpLabel = new QLabel("로봇 별로 구독할 토픽을 선택하여 모니터링할 수 있습니다.");
    topicHelpLabel->setStyleSheet("color: #555;");
    topicHelpLabel->setWordWrap(true);
    topicLayout->addWidget(topicHelpLabel);
    
    // 토픽 목록을 표시할 트리 위젯
    QTreeWidget* topicTree = new QTreeWidget();
    topicTree->setObjectName("topicTree");
    topicTree->setHeaderLabels(QStringList() << "토픽 이름" << "타입" << "구독");
    topicTree->setColumnWidth(0, 250);  // 토픽 이름 열 너비
    topicTree->setColumnWidth(1, 200);  // 토픽 타입 열 너비
    topicTree->setAlternatingRowColors(true);
    topicLayout->addWidget(topicTree);
    
    // 토픽 스캔 버튼 및 필터 영역
    QWidget* topicControlWidget = new QWidget();
    QHBoxLayout* topicControlLayout = new QHBoxLayout(topicControlWidget);
    
    // 필터링
    QLabel* filterLabel = new QLabel("필터:");
    QLineEdit* filterEdit = new QLineEdit();
    filterEdit->setObjectName("topicFilterEdit");
    filterEdit->setPlaceholderText("토픽 이름 필터...");
    
    // 토픽 스캔 버튼
    QPushButton* scanButton = new QPushButton("토픽 스캔");
    scanButton->setObjectName("scanTopicsButton");
    
    // 전체 선택/해제 버튼
    QPushButton* selectAllButton = new QPushButton("모두 선택");
    QPushButton* deselectAllButton = new QPushButton("모두 해제");
    
    // 레이아웃에 위젯 추가
    topicControlLayout->addWidget(filterLabel);
    topicControlLayout->addWidget(filterEdit, 1);  // 필터가 더 넓게
    topicControlLayout->addWidget(scanButton);
    topicControlLayout->addWidget(selectAllButton);
    topicControlLayout->addWidget(deselectAllButton);
    
    topicLayout->addWidget(topicControlWidget);
    
    // 토픽 구독 이벤트 연결
    connect(topicTree, &QTreeWidget::itemChanged, this, &MainWindow::onTopicItemChanged);
    
    // 토픽 스캔 버튼 시그널 연결
    connect(scanButton, &QPushButton::clicked, this, &MainWindow::onScanTopicsClicked);
    
    // 토픽 필터 시그널 연결
    connect(filterEdit, &QLineEdit::textChanged, this, &MainWindow::onTopicFilterChanged);
    
    // 전체 선택/해제 버튼 시그널 연결
    connect(selectAllButton, &QPushButton::clicked, [this, topicTree]() {
        for (int i = 0; i < topicTree->topLevelItemCount(); ++i) {
            QTreeWidgetItem* robotItem = topicTree->topLevelItem(i);
            for (int j = 0; j < robotItem->childCount(); ++j) {
                QTreeWidgetItem* topicItem = robotItem->child(j);
                if (topicItem->checkState(2) != Qt::Checked) {
                    topicItem->setCheckState(2, Qt::Checked);
                }
            }
        }
    });
    
    connect(deselectAllButton, &QPushButton::clicked, [this, topicTree]() {
        for (int i = 0; i < topicTree->topLevelItemCount(); ++i) {
            QTreeWidgetItem* robotItem = topicTree->topLevelItem(i);
            for (int j = 0; j < robotItem->childCount(); ++j) {
                QTreeWidgetItem* topicItem = robotItem->child(j);
                if (topicItem->checkState(2) != Qt::Unchecked) {
                    topicItem->setCheckState(2, Qt::Unchecked);
                }
            }
        }
    });
    
    topicSettingsLayout->addWidget(topicGroup);
    
    // 설정 탭에 서브탭 추가
    settings_tabs->addTab(robotSettingsTab, "로봇 설정");
    settings_tabs->addTab(topicSettingsTab, "토픽 설정");
    
    // 메인 레이아웃에 탭 위젯 추가
    main_layout->addWidget(settings_tabs);
    
    // 중앙 위젯에 추가
    central_widget_->addWidget(settings_tab_);
    
    // 디버깅 로그
    qDebug() << "설정 탭 초기화 완료";
}

void MainWindow::onReconnectClicked()
{
    statusBar()->showMessage("재연결 중...");
    
    // 선택된 로봇 확인
    QStringList selectedRobots;
    for (auto it = robot_checkboxes_.begin(); it != robot_checkboxes_.end(); ++it) {
        if (it.value()->isChecked()) {
            selectedRobots.append(it.key());
        }
    }
    
    if (selectedRobots.isEmpty()) {
        QMessageBox::warning(this, "경고", "재연결할 로봇을 선택해주세요.");
        return;
    }
    
    // 선택된 로봇 재연결 시도
    for (const QString& robotName : selectedRobots) {
        if (robots_.contains(robotName)) {
            RobotInfo& robot = robots_[robotName];
            
            // 연결 시도 (실제로는 ROS 연결 로직 구현 필요)
            robot.connected = true;
            
            // 연결 상태 업데이트
            updateRobotCard(robot);
            
            // 토픽 구독 시작
            if (robot.type == RobotType::JETCOBOT) {
                // Jetcobot coords_real 토픽 구독
                std::string topic = robot.namespace_name.toStdString() + "/coords_real";
                topic_manager_->subscribeTopic(topic, "geometry_msgs/msg/PoseStamped");
                
                // coords_real 토픽 메시지 처리 함수 연결
                connect(topic_manager_.get(), &TopicManager::messageReceived,
                        this, &MainWindow::onCoordsRealReceived);
            }
            
            // 이벤트 로그에 추가
            logEvent(robotName, "연결", "로봇에 재연결되었습니다.");
        }
    }
    
    statusBar()->showMessage(selectedRobots.count() == 1 ? 
                            "로봇에 재연결되었습니다." : 
                            QString("%1개의 로봇에 재연결되었습니다.").arg(selectedRobots.count()));
}

void MainWindow::onEmergencyClicked()
{
    // 선택된 로봇 확인
    QStringList selectedRobots;
    for (auto it = robot_checkboxes_.begin(); it != robot_checkboxes_.end(); ++it) {
        if (it.value()->isChecked()) {
            selectedRobots.append(it.key());
        }
    }
    
    if (selectedRobots.isEmpty()) {
        QMessageBox::warning(this, "경고", "긴급 정지할 로봇을 선택해주세요.");
        return;
    }
    
    // 확인 대화상자
    QString msg = selectedRobots.count() == 1 ? 
                 "선택한 로봇에 긴급 정지 명령을 전송하시겠습니까?" : 
                 QString("선택한 %1개의 로봇에 긴급 정지 명령을 전송하시겠습니까?").arg(selectedRobots.count());
    
    QMessageBox::StandardButton reply = QMessageBox::question(this, "긴급 정지", msg, 
                                                            QMessageBox::Yes | QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        // 선택된 로봇에 긴급 정지 명령 전송
        for (const QString& robotName : selectedRobots) {
            if (robots_.contains(robotName)) {
                RobotInfo& robot = robots_[robotName];
                
                // 긴급 정지 명령 전송 (실제로는 ROS 발행 로직 구현 필요)
                robot.emergency_stop = true;
                
                // 상태 업데이트
                updateRobotCard(robot);
                
                // 이벤트 로그에 추가
                logEvent(robotName, "장애", "긴급 정지 명령이 전송되었습니다.");
            }
        }
        
        QMessageBox::critical(this, "긴급 정지", 
                             selectedRobots.count() == 1 ? 
                             "선택한 로봇에 긴급 정지 명령이 전송되었습니다." : 
                             QString("선택한 %1개의 로봇에 긴급 정지 명령이 전송되었습니다.").arg(selectedRobots.count()));
    }
}

void MainWindow::onAddRobotClicked()
{
    // 로봇 이름 입력 대화상자
    bool ok;
    QString name = QInputDialog::getText(this, "로봇 추가", "로봇 이름:", 
                                        QLineEdit::Normal, "새 로봇", &ok);
    
    if (!ok || name.isEmpty()) {
        return;
    }
    
    // 이름 중복 확인
    if (robots_.contains(name)) {
        QMessageBox::warning(this, "오류", "이미 존재하는 로봇 이름입니다.");
        return;
    }
    
    // 새 로봇 정보 생성
    RobotInfo newRobot;
    newRobot.name = name;
    newRobot.type = RobotType::JETCOBOT;  // 기본값
    newRobot.domain_id = 10;  // 기본값
    newRobot.namespace_name = name.toLower().replace(" ", "_");
    
    // 로봇 정보 저장
    robots_[name] = newRobot;
    
    // 로봇 목록에 추가
    robot_list_->addItem(name);
    
    // 체크박스 추가
    QWidget* checkboxWidget = sidebar_->widget()->findChild<QScrollArea*>()->widget();
    
    // Jetcobot 컨테이너 또는 Pinky 컨테이너 찾기 (로봇 타입에 따라)
    QWidget* containerWidget = nullptr;
    QList<QLabel*> labels = checkboxWidget->findChildren<QLabel*>();
    
    if (newRobot.type == RobotType::JETCOBOT) {
        for (QLabel* label : labels) {
            if (label->text() == "Jetcobot") {
                containerWidget = checkboxWidget->findChildren<QWidget*>().value(
                    checkboxWidget->findChildren<QWidget*>().indexOf(label) + 1);
                break;
            }
        }
    } else {
        for (QLabel* label : labels) {
            if (label->text() == "Pinky") {
                containerWidget = checkboxWidget->findChildren<QWidget*>().value(
                    checkboxWidget->findChildren<QWidget*>().indexOf(label) + 1);
                break;
            }
        }
    }
    
    if (containerWidget) {
        QVBoxLayout* containerLayout = qobject_cast<QVBoxLayout*>(containerWidget->layout());
        if (containerLayout) {
            // "등록된 xxx이 없습니다" 라벨이 있는지 확인하고 있으면 제거
            for (int i = 0; i < containerLayout->count(); ++i) {
                QLayoutItem* item = containerLayout->itemAt(i);
                QLabel* label = qobject_cast<QLabel*>(item->widget());
                if (label && (label->text().contains("등록된") || label->text().contains("없습니다"))) {
                    containerLayout->removeItem(item);
                    delete label;
                    break;
                }
            }
            
            // 체크박스 생성 및 추가
    QCheckBox* checkbox = new QCheckBox(name);
            checkbox->setChecked(true);  // 기본적으로 선택됨
    connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::onRobotCheckStateChanged);
            containerLayout->addWidget(checkbox);
    robot_checkboxes_[name] = checkbox;
        }
    }
    
    // 로봇 카드 생성 및 추가
    QWidget* card = createRobotCard(newRobot);
    robot_cards_[name] = card;
    
    // 그리드 레이아웃에 추가
    QScrollArea* cardsScrollArea = fleet_overview_tab_->findChild<QScrollArea*>();
    QWidget* cardsContainer = cardsScrollArea->widget();
    QGridLayout* cardsLayout = qobject_cast<QGridLayout*>(cardsContainer->layout());
    
    if (cardsLayout) {
        // 현재 그리드의 행과 열 수 계산
        int rowCount = 0;
        int colCount = 2; // 고정 2열 레이아웃
        
        // 그리드 레이아웃의 항목 수 계산
        int itemCount = 0;
        for (int i = 0; i < cardsLayout->count(); ++i) {
            QLayoutItem* item = cardsLayout->itemAt(i);
            if (item && item->widget()) {
                itemCount++;
            }
        }
        
        // 새 위치 계산 (2열 그리드에서의 위치)
        int row = itemCount / colCount;
        int col = itemCount % colCount;
        
        // 카드 추가
        cardsLayout->addWidget(card, row, col);
        
        // 선택된 상태로 테두리 강조 적용
        card->setStyleSheet("QFrame { border: 3px solid #2196F3; border-radius: 5px; background-color: #E3F2FD; }");
    }
    
    // 새 로봇 선택
    robot_list_->setCurrentRow(robot_list_->count() - 1);
    
    // 설정 저장
    saveRobotConfigurations();
    
    statusBar()->showMessage(QString("로봇 '%1'이(가) 추가되었습니다.").arg(name));
}

void MainWindow::onRemoveRobotClicked()
{
    // 현재 선택된 항목 확인
    QListWidgetItem* currentItem = robot_list_->currentItem();
    if (!currentItem) {
        QMessageBox::warning(this, "경고", "제거할 로봇을 선택해주세요.");
        return;
    }
    
    QString name = currentItem->text();
    
    // 확인 대화상자
    QMessageBox::StandardButton reply = QMessageBox::question(this, "로봇 제거", 
                                                            QString("로봇 '%1'을(를) 정말로 제거하시겠습니까?").arg(name), 
                                                            QMessageBox::Yes | QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        // 로봇 목록에서 제거
        delete robot_list_->takeItem(robot_list_->row(currentItem));
        
        // 체크박스 제거
        if (robot_checkboxes_.contains(name)) {
            delete robot_checkboxes_[name];
            robot_checkboxes_.remove(name);
        }
        
        // 카드 제거
        if (robot_cards_.contains(name)) {
            // 카드 위젯 찾기 및 제거
            QScrollArea* cardsScrollArea = fleet_overview_tab_->findChild<QScrollArea*>();
            QWidget* cardsContainer = cardsScrollArea->widget();
            QGridLayout* cardsLayout = qobject_cast<QGridLayout*>(cardsContainer->layout());
            
            if (cardsLayout) {
                // 그리드 레이아웃에서 카드 위치 찾기
                int itemIndex = -1;
                for (int i = 0; i < cardsLayout->count(); ++i) {
                    QLayoutItem* item = cardsLayout->itemAt(i);
                    if (item && item->widget() == robot_cards_[name]) {
                        itemIndex = i;
                        break;
                    }
                }
                
                if (itemIndex >= 0) {
                    // 아이템 제거
                    QLayoutItem* item = cardsLayout->takeAt(itemIndex);
                    if (item) {
                        if (item->widget()) {
                            delete item->widget();
                        }
                        delete item;
                    }
                    
                    // 나머지 카드 재배치
                    QList<QWidget*> remainingCards;
                    
                    // 모든 카드 수집
                    while (cardsLayout->count() > 0) {
                        QLayoutItem* item = cardsLayout->takeAt(0);
                        if (item && item->widget()) {
                            remainingCards.append(item->widget());
                        }
                        delete item;
                    }
                    
                    // 2열 그리드로 다시 배치
                    int row = 0, col = 0;
                    for (QWidget* widget : remainingCards) {
                        cardsLayout->addWidget(widget, row, col);
                        col++;
                        if (col >= 2) {
                            col = 0;
                            row++;
                        }
                    }
                    
                    // 빈 공간 추가하여 레이아웃이 확장되도록 설정
                    cardsLayout->setRowStretch(row + 1, 1);
                    cardsLayout->setColumnStretch(2, 1);
                    
                    // 맵에서 제거
            robot_cards_.remove(name);
                }
            }
        }
        
        // 로봇 정보 맵에서 제거
        robots_.remove(name);
        
        // 설정 저장
        saveRobotConfigurations();
        
        statusBar()->showMessage(QString("로봇 '%1'이(가) 제거되었습니다.").arg(name));
    }
}

void MainWindow::onRobotSelectionChanged()
{
    // 현재 선택된 항목 확인
    QListWidgetItem* currentItem = robot_list_->currentItem();
    if (!currentItem) {
        qDebug() << "선택된 로봇 없음";
        return;
    }
    
    QString name = currentItem->text();
    qDebug() << "로봇 선택됨: \"" + name + "\"";
    
    // 해당 로봇 정보 가져오기
    if (!robots_.contains(name)) {
        qDebug() << "선택된 로봇에 대한 정보 없음:" << name;
        return;
    }
    
    const RobotInfo& robot = robots_[name];
    qDebug() << "로봇 정보 로드됨: \"" + name + "\"" << "유형:" << static_cast<int>(robot.type);
    
    // 설정 탭의 필드 위젯 찾기
    QGroupBox* settingsGroup = settings_tab_->findChild<QGroupBox*>("settingsGroup");
    if (!settingsGroup) {
        qDebug() << "설정 그룹 위젯을 찾을 수 없음";
        return;
    }
    
    // selectRobotLabel 숨기기 (메시지가 있는 경우에만)
    QLabel* selectRobotLabel = settings_tab_->findChild<QLabel*>("selectRobotLabel");
    if (selectRobotLabel) {
        selectRobotLabel->hide();
        qDebug() << "안내 메시지 숨김";
    } else {
        qDebug() << "안내 메시지를 찾을 수 없음";
    }
    
    // 기존 필드 업데이트
    QLineEdit* nameEdit = settings_tab_->findChild<QLineEdit*>("nameEdit");
    QComboBox* typeCombo = settings_tab_->findChild<QComboBox*>("typeCombo");
    QSpinBox* domainIdSpin = settings_tab_->findChild<QSpinBox*>("domainIdSpin");
    QLineEdit* namespaceEdit = settings_tab_->findChild<QLineEdit*>("namespaceEdit");
    QPushButton* saveButton = settings_tab_->findChild<QPushButton*>("saveButton");
    
    if (nameEdit && typeCombo && domainIdSpin && namespaceEdit && saveButton) {
        // 폼 활성화
        nameEdit->setEnabled(true);
        typeCombo->setEnabled(true);
        domainIdSpin->setEnabled(true);
        namespaceEdit->setEnabled(true);
        saveButton->setEnabled(true);
        
        // 데이터 설정
        nameEdit->setText(robot.name);
        typeCombo->setCurrentIndex(static_cast<int>(robot.type));
        domainIdSpin->setValue(robot.domain_id);
        namespaceEdit->setText(robot.namespace_name);
        
        qDebug() << "기존 설정 필드 업데이트:";
        qDebug() << "  - 이름:" << robot.name;
        qDebug() << "  - 유형:" << static_cast<int>(robot.type);
        qDebug() << "  - 도메인 ID:" << robot.domain_id;
        qDebug() << "  - 네임스페이스:" << robot.namespace_name;
    } else {
        qDebug() << "일부 설정 필드를 찾을 수 없음:";
        qDebug() << "  - nameEdit 존재:" << (nameEdit != nullptr);
        qDebug() << "  - typeCombo 존재:" << (typeCombo != nullptr);
        qDebug() << "  - domainIdSpin 존재:" << (domainIdSpin != nullptr);
        qDebug() << "  - namespaceEdit 존재:" << (namespaceEdit != nullptr);
        qDebug() << "  - saveButton 존재:" << (saveButton != nullptr);
    }
    
    // 선택된 로봇 강조 표시
    for (int i = 0; i < robot_list_->count(); ++i) {
        QListWidgetItem* item = robot_list_->item(i);
        if (item->text() == name) {
            item->setBackground(QColor(230, 245, 255));  // 연한 파란색
            item->setForeground(QColor(0, 0, 0));       // 검은색 텍스트
        } else {
            item->setBackground(QColor(255, 255, 255));  // 흰색
            item->setForeground(QColor(0, 0, 0));       // 검은색 텍스트
        }
    }
    
    // 설정 탭이 현재 표시되고 있는지 확인
    if (central_widget_->currentWidget() == settings_tab_) {
        // 화면 업데이트 강제
        settings_tab_->update();
        qDebug() << "설정 탭 업데이트 완료";
    }
}

void MainWindow::onSaveConfigClicked()
{
    // 현재 선택된 항목 확인
    QListWidgetItem* currentItem = robot_list_->currentItem();
    if (!currentItem) {
        QMessageBox::warning(this, "경고", "설정을 저장할 로봇을 선택해주세요.");
        return;
    }
    
    QString name = currentItem->text();
    
    // 해당 로봇 정보 가져오기
    if (!robots_.contains(name)) {
        return;
    }
    
    RobotInfo& robot = robots_[name];
    
    // 설정 탭의 필드 위젯 찾기
    QWidget* settingsGroup = settings_tab_->findChild<QGroupBox*>("settingsGroup");
    if (!settingsGroup) {
        return;
    }
    
    // 필드 값 가져오기
    QList<QLineEdit*> lineEdits = settingsGroup->findChildren<QLineEdit*>();
    QList<QComboBox*> comboBoxes = settingsGroup->findChildren<QComboBox*>();
    QList<QSpinBox*> spinBoxes = settingsGroup->findChildren<QSpinBox*>();
    
    QString newName = robot.name;
    QString newNamespace = robot.namespace_name;
    RobotType newType = robot.type;
    int newDomainId = robot.domain_id;
    
    // 필드 값 읽기
    for (QLineEdit* edit : lineEdits) {
        if (edit->objectName() == "nameEdit") {
            newName = edit->text();
        } else if (edit->objectName() == "namespaceEdit") {
            newNamespace = edit->text();
        }
    }
    
    for (QComboBox* combo : comboBoxes) {
        if (combo->objectName() == "typeCombo") {
            newType = static_cast<RobotType>(combo->currentIndex());
        }
    }
    
    for (QSpinBox* spin : spinBoxes) {
        if (spin->objectName() == "domainIdSpin") {
            newDomainId = spin->value();
        }
    }
    
    bool typeChanged = (newType != robot.type);
    
    // 이름이 변경된 경우
    if (newName != robot.name) {
        // 이름 중복 확인
        if (robots_.contains(newName)) {
            QMessageBox::warning(this, "오류", "이미 존재하는 로봇 이름입니다.");
            return;
        }
        
        // 리스트 아이템 업데이트
        currentItem->setText(newName);
        
        // 체크박스 업데이트
        if (robot_checkboxes_.contains(robot.name)) {
            QCheckBox* checkbox = robot_checkboxes_[robot.name];
            checkbox->setText(newName);
            robot_checkboxes_.remove(robot.name);
            robot_checkboxes_[newName] = checkbox;
        }
        
        // 카드 업데이트
        if (robot_cards_.contains(robot.name)) {
            QWidget* card = robot_cards_[robot.name];
            QList<QLabel*> labels = card->findChildren<QLabel*>();
            for (QLabel* label : labels) {
                if (label->styleSheet().contains("font-size: 16px")) {
                    label->setText(newName);
                    break;
                }
            }
            robot_cards_.remove(robot.name);
            robot_cards_[newName] = card;
        }
        
        // 로봇 정보 맵 업데이트
        RobotInfo newRobot = robot;
        robots_.remove(robot.name);
        newRobot.name = newName;
        robots_[newName] = newRobot;
        
        // 참조 업데이트
        robot = robots_[newName];
    }
    
    // 타입이 변경된 경우 사이드바 카테고리 체크박스 이동
    if (typeChanged) {
        // 기존 체크박스 제거
        if (robot_checkboxes_.contains(robot.name)) {
            QCheckBox* checkbox = robot_checkboxes_[robot.name];
            bool isChecked = checkbox->isChecked(); // 현재 체크 상태 저장
            
            // 기존 체크박스 레이아웃에서 제거
            if (checkbox->parentWidget()) {
                checkbox->parentWidget()->layout()->removeWidget(checkbox);
            }
            
            // 사이드바 위젯 찾기
            QWidget* checkboxWidget = sidebar_->widget()->findChild<QScrollArea*>()->widget();
            
            // 타입에 따라 적절한 컨테이너 찾기
            QWidget* containerWidget = nullptr;
            QList<QLabel*> labels = checkboxWidget->findChildren<QLabel*>();
            
            // 타입에 따라 라벨 텍스트 결정
            QString labelText = (newType == RobotType::JETCOBOT) ? "Jetcobot" : "Pinky";
            
            // 컨테이너 찾기
            for (QLabel* label : labels) {
                if (label->text() == labelText) {
                    containerWidget = checkboxWidget->findChildren<QWidget*>().value(
                        checkboxWidget->findChildren<QWidget*>().indexOf(label) + 1);
                    break;
                }
            }
            
            if (containerWidget) {
                QVBoxLayout* containerLayout = qobject_cast<QVBoxLayout*>(containerWidget->layout());
                if (containerLayout) {
                    // "등록된 xxx이 없습니다" 라벨이 있는지 확인하고 있으면 제거
                    for (int i = 0; i < containerLayout->count(); ++i) {
                        QLayoutItem* item = containerLayout->itemAt(i);
                        QLabel* label = qobject_cast<QLabel*>(item->widget());
                        if (label && (label->text().contains("등록된") || label->text().contains("없습니다"))) {
                            containerLayout->removeItem(item);
                            delete label;
                            break;
                        }
                    }
                    
                    // 새 컨테이너에 체크박스 추가
                    containerLayout->addWidget(checkbox);
                    checkbox->setChecked(isChecked); // 이전 체크 상태 복원
                    
                    qDebug() << "로봇 체크박스 카테고리 이동:" << robot.name << "->" << ((newType == RobotType::JETCOBOT) ? "Jetcobot" : "Pinky");
                }
            }
        }
    }
    
    // 나머지 필드 업데이트
    robot.type = newType;
    robot.domain_id = newDomainId;
    robot.namespace_name = newNamespace;
    
    // 카드 업데이트
    updateRobotCard(robot);
    
    // 로봇 탭 체크박스 업데이트 (타입이 변경된 경우 필요)
    if (typeChanged) {
        updateRobotTabCheckboxes();
    }
    
    // 설정 저장
    saveRobotConfigurations();
    
    statusBar()->showMessage(QString("로봇 '%1'의 설정이 저장되었습니다.").arg(robot.name));
}

void MainWindow::onRobotCheckStateChanged(int state)
{
    // 체크박스 확인
    QCheckBox* checkbox = qobject_cast<QCheckBox*>(sender());
    if (!checkbox) {
        return;
    }
    
    QString robotName = checkbox->text();
    
    // 로봇 카드 상태 업데이트
    if (robot_cards_.contains(robotName)) {
        QWidget* card = robot_cards_[robotName];
        
        // 선택 효과 적용 - 카드 바깥쪽 테두리만 강조하고 내부 컴포넌트는 강조하지 않음
        if (state == Qt::Checked) {
            // 카드 전체에만 테두리 강조 적용
            card->setStyleSheet("QFrame { border: 3px solid #2196F3; border-radius: 5px; background-color: #E3F2FD; }");
            
            // 내부 컴포넌트들의 테두리 제거 스타일을 명시적으로 적용
            QList<QWidget*> children = card->findChildren<QWidget*>();
            for (QWidget* child : children) {
                // 이미 스타일이 있는 경우 테두리 없음 스타일 추가
                if (!child->styleSheet().isEmpty() && !child->styleSheet().contains("border: none")) {
                    child->setStyleSheet(child->styleSheet() + "; border: none;");
                }
                // 배터리 바는 특별 처리
                if (child->objectName() == "batteryBar") {
                    QProgressBar* bar = qobject_cast<QProgressBar*>(child);
                    if (bar) {
                        if (bar->value() > 70) {
                            bar->setStyleSheet("QProgressBar { text-align: center; border: none; } "
                                            "QProgressBar::chunk { background-color: #4CAF50; }");
                        } else if (bar->value() > 30) {
                            bar->setStyleSheet("QProgressBar { text-align: center; border: none; } "
                                            "QProgressBar::chunk { background-color: #FF9800; }");
        } else {
                            bar->setStyleSheet("QProgressBar { text-align: center; border: none; } "
                                            "QProgressBar::chunk { background-color: #F44336; }");
                        }
                    }
                }
            }
        } else {
            // 비선택 시 기본 스타일
            card->setStyleSheet("QFrame { border: 1px solid #ddd; border-radius: 5px; background-color: #f8f8f8; }");
        }
    }
}

void MainWindow::onCoordsRealReceived(const std::string& topic, const std::vector<uint8_t>& msg)
{
    // 토픽에서 로봇 네임스페이스 추출
    std::string ns = topic.substr(0, topic.find("/coords_real"));
    
    // 해당 네임스페이스를 가진 로봇 찾기
    QString qtNs = QString::fromStdString(ns);
    QString robotName;
    
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        if (it.value().namespace_name == qtNs) {
            robotName = it.key();
            break;
        }
    }
    
    if (robotName.isEmpty() || !robots_.contains(robotName)) {
        return;
    }
    
    // 메시지 파싱 (geometry_msgs/msg/PoseStamped)
    // 실제 구현은 ROS 메시지 파싱 로직이 필요합니다.
    // 여기서는 간단한 예시로 처리합니다.
    
    // 예시: 메시지에서 x, y, z 추출
    double x = 0.0, y = 0.0, z = 0.0;
    
    // 실제 메시지 파싱 코드가 여기에 들어갑니다.
    // 아래는 임시 예시입니다.
    if (msg.size() >= 24) {  // 메시지가 최소한의 크기를 가지는지 확인
        // PoseStamped 메시지에서 position 데이터 오프셋 (실제로는 메시지 구조에 따라 달라질 수 있음)
        // 이 부분은 실제 ROS 2 메시지 직렬화/역직렬화 메커니즘에 따라 구현되어야 합니다.
        std::memcpy(&x, &msg[0], sizeof(double));
        std::memcpy(&y, &msg[8], sizeof(double));
        std::memcpy(&z, &msg[16], sizeof(double));
    }
    
    // 로봇 위치 정보 업데이트
    RobotInfo& robot = robots_[robotName];
    robot.position["x"] = QString::number(x, 'f', 2);
    robot.position["y"] = QString::number(y, 'f', 2);
    robot.position["z"] = QString::number(z, 'f', 2);
    
    // 카드 업데이트
    updateRobotCard(robot);
}

void MainWindow::onMessageReceived(const std::string& topic, const std::vector<uint8_t>& msg)
{
    // 메시지 저장소에 메시지 저장
    message_store_->storeMessage(topic, msg);
    
    // 상태바 업데이트 (토픽 이름과 시간 표시)
    statusBar()->showMessage(QString("메시지 수신: %1").arg(topic.c_str()));
    
    // 토픽 이름에서 로봇 네임스페이스 추출
    std::string ns = topic.substr(0, topic.find_last_of("/"));
    ns = ns.substr(0, ns.find_last_of("/") + 1);
    
    // 해당 네임스페이스를 가진 로봇 찾기
    QString qtNs = QString::fromStdString(ns);
    QString robotName;
    
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        if (qtNs.startsWith(it.value().namespace_name)) {
            robotName = it.key();
            break;
        }
    }
    
    // 로봇을 찾았다면 이벤트 로그에 추가
    if (!robotName.isEmpty()) {
        QString topicName = QString::fromStdString(topic.substr(topic.find_last_of("/") + 1));
        logEvent(robotName, "토픽", QString("%1 메시지가 수신되었습니다.").arg(topicName));
    }
}

void MainWindow::loadRobotConfigurations()
{
    // 로봇 정보를 설정 파일에서 로드
    QVariant robotNamesVariant = config_manager_->getSettings().value("robot_names", QStringList());
    QStringList robotNames = robotNamesVariant.toStringList();
    
    // 로봇 정보가 없으면 기본 로봇 생성
    if (robotNames.isEmpty()) {
        // 기본 Jetcobot 추가
        RobotInfo jetcobot;
        jetcobot.name = "Jetcobot_1";
        jetcobot.type = RobotType::JETCOBOT;
        jetcobot.domain_id = 10;
        jetcobot.namespace_name = "jetcobot_1";
        robots_[jetcobot.name] = jetcobot;
        
        // 기본 Pinky 추가
        RobotInfo pinky;
        pinky.name = "Pinky_1";
        pinky.type = RobotType::PINKY;
        pinky.domain_id = 20;
        pinky.namespace_name = "pinky_1";
        robots_[pinky.name] = pinky;
        
        // 로봇 리스트에 추가
        robot_list_->addItem(jetcobot.name);
        robot_list_->addItem(pinky.name);

        // 기본값 저장
        robotNames << jetcobot.name << pinky.name;
    } else {
        // 저장된 로봇 정보 로드
        for (const QString& name : robotNames) {
            RobotInfo robot;
            robot.name = name;
            robot.type = static_cast<RobotType>(config_manager_->getSettings().value(name + "/type", 0).toInt());
            robot.domain_id = config_manager_->getSettings().value(name + "/domain_id", 0).toInt();
            robot.namespace_name = config_manager_->getSettings().value(name + "/namespace", "").toString();
            robots_[name] = robot;
            
            // 로봇 목록에 추가
            robot_list_->addItem(name);
        }
    }
    
    // 사이드바 체크박스 생성
    QWidget* checkboxWidget = sidebar_->widget()->findChild<QScrollArea*>()->widget();
    
    // Jetcobot 컨테이너와 Pinky 컨테이너 찾기
    QWidget* jetcobotContainer = nullptr;
    QWidget* pinkyContainer = nullptr;
    QWidget* selectAllContainer = nullptr;
    
    QList<QLabel*> labels = checkboxWidget->findChildren<QLabel*>();
    for (QLabel* label : labels) {
        if (label->text() == "Jetcobot") {
            jetcobotContainer = checkboxWidget->findChildren<QWidget*>().value(
                checkboxWidget->findChildren<QWidget*>().indexOf(label) + 1);
        } else if (label->text() == "Pinky") {
            pinkyContainer = checkboxWidget->findChildren<QWidget*>().value(
                checkboxWidget->findChildren<QWidget*>().indexOf(label) + 1);
        }
    }
    
    // 전체 선택/해제 버튼 컨테이너 찾기
    selectAllContainer = checkboxWidget->findChild<QWidget*>(QString(), Qt::FindDirectChildrenOnly);
    
    if (!jetcobotContainer || !pinkyContainer || !selectAllContainer) return;
    
    QVBoxLayout* jetcobotLayout = qobject_cast<QVBoxLayout*>(jetcobotContainer->layout());
    QVBoxLayout* pinkyLayout = qobject_cast<QVBoxLayout*>(pinkyContainer->layout());
    
    if (!jetcobotLayout || !pinkyLayout) return;
    
    // 체크박스 생성 전에 기존 항목 제거
    while (jetcobotLayout->count() > 0) {
        QLayoutItem* item = jetcobotLayout->takeAt(0);
        if (item->widget()) {
            delete item->widget();
        }
        delete item;
    }
    
    while (pinkyLayout->count() > 0) {
        QLayoutItem* item = pinkyLayout->takeAt(0);
        if (item->widget()) {
            delete item->widget();
        }
        delete item;
    }
    
    // 로봇별 체크박스 생성
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        QCheckBox* checkbox = new QCheckBox(it.key());
        checkbox->setChecked(true);  // 기본적으로 모든 로봇 선택
        connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::onRobotCheckStateChanged);
        
        // 로봇 타입에 따라 적절한 컨테이너에 추가
        if (it.value().type == RobotType::JETCOBOT) {
            jetcobotLayout->addWidget(checkbox);
        } else {
            pinkyLayout->addWidget(checkbox);
        }
        
        robot_checkboxes_[it.key()] = checkbox;
    }
    
    // 컨테이너가 비어있는 경우 안내 메시지 추가
    if (jetcobotLayout->count() == 0) {
        QLabel* emptyLabel = new QLabel("등록된 Jetcobot이 없습니다.");
        emptyLabel->setStyleSheet("color: #888;");
        jetcobotLayout->addWidget(emptyLabel);
    }
    
    if (pinkyLayout->count() == 0) {
        QLabel* emptyLabel = new QLabel("등록된 Pinky가 없습니다.");
        emptyLabel->setStyleSheet("color: #888;");
        pinkyLayout->addWidget(emptyLabel);
    }
    
    // 대시보드의 플릿 오버뷰 탭에 로봇 카드 추가
    if (dashboard_) {
        QTabWidget* tabWidget = dashboard_->findChild<QTabWidget*>();
        if (tabWidget && tabWidget->count() > 0) {
            QWidget* fleetTab = tabWidget->widget(0);  // 첫 번째 탭이 플릿 오버뷰
            if (fleetTab) {
                QScrollArea* cardsScrollArea = fleetTab->findChild<QScrollArea*>();
                if (cardsScrollArea) {
                    QWidget* cardsContainer = cardsScrollArea->widget();
                    
                    // 기존 레이아웃 제거
                    if (cardsContainer->layout()) {
                        delete cardsContainer->layout();
                    }
                    
                    // 그리드 레이아웃으로 변경 (2열 배치)
                    QGridLayout* cardsLayout = new QGridLayout(cardsContainer);
                    cardsLayout->setContentsMargins(10, 10, 10, 10);
                    cardsLayout->setSpacing(15);
                    
                    // 로봇 카드를 2열 그리드로 배치
                    int row = 0, col = 0;
                    for (const QString& name : robotNames) {
                        QWidget* card = createRobotCard(robots_[name]);
                        robot_cards_[name] = card;
                        cardsLayout->addWidget(card, row, col);
                        
                        // 다음 위치 계산 (2열)
                        col++;
                        if (col >= 2) {
                            col = 0;
                            row++;
                        }
                        
                        // 체크 상태에 맞게 테두리 강조 설정
                        if (robot_checkboxes_.contains(name) && robot_checkboxes_[name]->isChecked()) {
                            card->setStyleSheet("QFrame { border: 3px solid #2196F3; border-radius: 5px; background-color: #E3F2FD; }");
                        }
                    }
                    
                    // 빈 공간을 추가하여 레이아웃이 확장되도록 설정
                    cardsLayout->setRowStretch(row + 1, 1);
                    cardsLayout->setColumnStretch(2, 1);
                }
            }
        }
    }
    
    qDebug() << "로봇 구성 로드 완료: " << robots_.size() << "개의 로봇";
    
    // 로드 후 첫 번째 로봇 선택
    if (robot_list_->count() > 0) {
        QListWidgetItem* firstItem = robot_list_->item(0);
        if (firstItem) {
            qDebug() << "로봇 목록에서 첫 번째 로봇 선택";
            robot_list_->setCurrentItem(firstItem);
            firstItem->setSelected(true);
        }
    }
}

void MainWindow::saveRobotConfigurations()
{
    // 저장할 로봇 이름 목록
    QStringList robotNames;
    
    // 모든 로봇 정보 저장
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        const QString& name = it.key();
        const RobotInfo& robot = it.value();
        
        robotNames.append(name);
        config_manager_->getSettings().setValue(name + "/type", static_cast<int>(robot.type));
        config_manager_->getSettings().setValue(name + "/domain_id", robot.domain_id);
        config_manager_->getSettings().setValue(name + "/namespace", robot.namespace_name);
    }
    
    // 로봇 이름 목록 저장
    config_manager_->getSettings().setValue("robot_names", robotNames);
    
    // 설정 저장
    config_manager_->getSettings().sync();
}

QWidget* MainWindow::createRobotCard(const RobotInfo& robot_info)
{
    // 카드 컨테이너
    QFrame* card = new QFrame();
    card->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
    card->setStyleSheet("QFrame { border: 1px solid #ddd; border-radius: 5px; background-color: #f8f8f8; }");
    
    QVBoxLayout* cardLayout = new QVBoxLayout(card);
    
    // 로봇 이름 및 상태
    QWidget* headerWidget = new QWidget();
    headerWidget->setStyleSheet("border: none;"); // 내부 위젯 테두리 제거
    QHBoxLayout* headerLayout = new QHBoxLayout(headerWidget);
    headerLayout->setContentsMargins(0, 0, 0, 0);
    
    QLabel* nameLabel = new QLabel(robot_info.name);
    nameLabel->setStyleSheet("font-size: 16px; font-weight: bold; border: none;"); // 테두리 제거
    
    QLabel* statusLabel = new QLabel(robot_info.connected ? "연결됨" : "연결 안됨");
    statusLabel->setStyleSheet(robot_info.connected ? 
                              "color: green; font-weight: bold; border: none;" : 
                              "color: red; font-weight: bold; border: none;"); // 테두리 제거
    
    headerLayout->addWidget(nameLabel);
    headerLayout->addStretch();
    headerLayout->addWidget(statusLabel);
    
    cardLayout->addWidget(headerWidget);
    
    // 구분선
    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    line->setStyleSheet("border: none;"); // 구분선 테두리 제거
    cardLayout->addWidget(line);
    
    // 로봇 정보
    QFormLayout* infoLayout = new QFormLayout();
    infoLayout->setContentsMargins(0, 0, 0, 0);
    
    // 로봇 유형
    QString typeStr = (robot_info.type == RobotType::JETCOBOT) ? "Jetcobot" : "Pinky";
    QLabel* typeLabel = new QLabel(typeStr);
    typeLabel->setStyleSheet("border: none;"); // 테두리 제거
    infoLayout->addRow(new QLabel("유형:"), typeLabel);
    infoLayout->itemAt(infoLayout->rowCount()-1, QFormLayout::LabelRole)->widget()->setStyleSheet("border: none;"); // 라벨 테두리 제거
    
    // 도메인 ID
    QLabel* domainIdLabel = new QLabel(QString::number(robot_info.domain_id));
    domainIdLabel->setStyleSheet("border: none;"); // 테두리 제거
    infoLayout->addRow(new QLabel("도메인 ID:"), domainIdLabel);
    infoLayout->itemAt(infoLayout->rowCount()-1, QFormLayout::LabelRole)->widget()->setStyleSheet("border: none;"); // 라벨 테두리 제거
    
    // 네임스페이스
    QLabel* namespaceLabel = new QLabel(robot_info.namespace_name);
    namespaceLabel->setStyleSheet("border: none;"); // 테두리 제거
    infoLayout->addRow(new QLabel("네임스페이스:"), namespaceLabel);
    infoLayout->itemAt(infoLayout->rowCount()-1, QFormLayout::LabelRole)->widget()->setStyleSheet("border: none;"); // 라벨 테두리 제거
    
    // 배터리 레벨
    QProgressBar* batteryBar = new QProgressBar();
    batteryBar->setRange(0, 100);
    batteryBar->setValue(static_cast<int>(robot_info.battery_level * 100));
    batteryBar->setObjectName("batteryBar"); // 객체 이름 설정
    
    // 배터리 색상 설정
    if (robot_info.battery_level > 0.7) {
        batteryBar->setStyleSheet("QProgressBar { text-align: center; border: none; } "
                                "QProgressBar::chunk { background-color: #4CAF50; }");  // 녹색, 테두리 제거
    } else if (robot_info.battery_level > 0.3) {
        batteryBar->setStyleSheet("QProgressBar { text-align: center; border: none; } "
                                "QProgressBar::chunk { background-color: #FF9800; }");  // 주황색, 테두리 제거
    } else {
        batteryBar->setStyleSheet("QProgressBar { text-align: center; border: none; } "
                                "QProgressBar::chunk { background-color: #F44336; }");  // 빨간색, 테두리 제거
    }
    
    QLabel* batteryLabel = new QLabel("배터리:");
    batteryLabel->setStyleSheet("border: none;"); // 테두리 제거
    infoLayout->addRow(batteryLabel, batteryBar);
    
    // 위치 정보
    QLabel* positionLabel = nullptr;
    if (robot_info.position.contains("x") && robot_info.position.contains("y")) {
        QString posStr = QString("X: %1, Y: %2").arg(robot_info.position["x"]).arg(robot_info.position["y"]);
        if (robot_info.position.contains("z")) {
            posStr += QString(", Z: %1").arg(robot_info.position["z"]);
        }
        positionLabel = new QLabel(posStr);
        positionLabel->setStyleSheet("border: none;"); // 테두리 제거
        QLabel* posLabel = new QLabel("위치:");
        posLabel->setStyleSheet("border: none;"); // 테두리 제거
        infoLayout->addRow(posLabel, positionLabel);
    }
    
    // 긴급 정지 상태
    QLabel* emergencyLabel = new QLabel(robot_info.emergency_stop ? "활성화됨" : "비활성화됨");
    emergencyLabel->setStyleSheet(robot_info.emergency_stop ? 
                               "color: red; font-weight: bold; border: none;" : 
                               "color: green; border: none;"); // 테두리 제거
    QLabel* emergencyTitleLabel = new QLabel("긴급 정지:");
    emergencyTitleLabel->setStyleSheet("border: none;"); // 테두리 제거
    infoLayout->addRow(emergencyTitleLabel, emergencyLabel);
    
    cardLayout->addLayout(infoLayout);
    
    // 대시보드 이동 버튼 영역
    QWidget* buttonWidget = new QWidget();
    buttonWidget->setStyleSheet("border: none;"); // 내부 위젯 테두리 제거
    QHBoxLayout* buttonLayout = new QHBoxLayout(buttonWidget);
    
    QPushButton* detailsButton = new QPushButton("상세 정보");
    // 버튼 스타일 설정 (테두리는 유지하되 카드 선택 시 강조되지 않도록)
    detailsButton->setStyleSheet("QPushButton { border: 1px solid #ddd; padding: 5px; background-color: #f0f0f0; } "
                               "QPushButton:hover { background-color: #e0e0e0; }");
    
    // Jetcobot 탭 또는 Pinky 탭으로 이동하는 버튼 연결
    connect(detailsButton, &QPushButton::clicked, [this, robot_info]() {
        if (robot_info.type == RobotType::JETCOBOT) {
            central_widget_->setCurrentIndex(1);  // Jetcobot 탭 인덱스
        } else {
            central_widget_->setCurrentIndex(2);  // Pinky 탭 인덱스
        }
        
        // 해당 로봇 체크박스 선택
        if (robot_checkboxes_.contains(robot_info.name)) {
            robot_checkboxes_[robot_info.name]->setChecked(true);
        }
    });
    
    buttonLayout->addWidget(detailsButton);
    
    cardLayout->addWidget(buttonWidget);
    
    // 드래그 가능하도록 설정
    card->setAcceptDrops(true);
    
    // 최소 크기 설정 (고정 크기 대신 최소 크기만 설정)
    card->setMinimumWidth(250);
    card->setMinimumHeight(200);
    
    return card;
}

void MainWindow::updateRobotCard(const RobotInfo& robot_info)
{
    // 로봇 카드가 존재하는지 확인
    if (!robot_cards_.contains(robot_info.name)) {
        return;
    }
    
    QWidget* card = robot_cards_[robot_info.name];
    
    // 연결 상태 업데이트
    QLabel* statusLabel = card->findChild<QLabel*>();
    if (statusLabel && statusLabel->text().contains("연결")) {
        statusLabel->setText(robot_info.connected ? "연결됨" : "연결 안됨");
        statusLabel->setStyleSheet(robot_info.connected ? 
                                  "color: green; font-weight: bold;" : 
                                  "color: red; font-weight: bold;");
    }
    
    // 배터리 상태 업데이트
    QProgressBar* batteryBar = card->findChild<QProgressBar*>();
    if (batteryBar) {
        batteryBar->setValue(static_cast<int>(robot_info.battery_level * 100));
        
        // 배터리 색상 설정
        if (robot_info.battery_level > 0.7) {
            batteryBar->setStyleSheet("QProgressBar { text-align: center; } "
                                    "QProgressBar::chunk { background-color: #4CAF50; }");  // 녹색
        } else if (robot_info.battery_level > 0.3) {
            batteryBar->setStyleSheet("QProgressBar { text-align: center; } "
                                    "QProgressBar::chunk { background-color: #FF9800; }");  // 주황색
        } else {
            batteryBar->setStyleSheet("QProgressBar { text-align: center; } "
                                    "QProgressBar::chunk { background-color: #F44336; }");  // 빨간색
        }
    }
    
    // 위치 정보 업데이트
    QList<QLabel*> labels = card->findChildren<QLabel*>();
    for (QLabel* label : labels) {
        if (label->text().startsWith("X:")) {
            if (robot_info.position.contains("x") && robot_info.position.contains("y")) {
                QString posStr = QString("X: %1, Y: %2").arg(robot_info.position["x"]).arg(robot_info.position["y"]);
                if (robot_info.position.contains("z")) {
                    posStr += QString(", Z: %1").arg(robot_info.position["z"]);
                }
                label->setText(posStr);
            }
        } else if (label->text() == "활성화됨" || label->text() == "비활성화됨") {
            label->setText(robot_info.emergency_stop ? "활성화됨" : "비활성화됨");
            label->setStyleSheet(robot_info.emergency_stop ? 
                               "color: red; font-weight: bold;" : 
                               "color: green;");
        }
    }
}

void MainWindow::onLogLevelChanged(int level)
{
    // 로그 레벨 변경 및 설정 저장
    config_manager_->setLogLevel(level);
    
    // 상태바에 로그 레벨 변경 표시
    QString levelStr;
    switch (level) {
    case 0:
        levelStr = "디버그";
        break;
    case 1:
        levelStr = "정보";
        break;
    case 2:
        levelStr = "경고";
        break;
    case 3:
        levelStr = "오류";
        break;
    case 4:
        levelStr = "심각";
        break;
    default:
        levelStr = "알 수 없음";
        break;
    }
    
    statusBar()->showMessage(QString("로그 레벨이 %1로 변경되었습니다.").arg(levelStr));
}

void MainWindow::logEvent(const QString& robotName, const QString& eventType, const QString& description)
{
    // 대시보드가 활성화되어 있는지 확인
    if (!dashboard_) return;
    
    // 이벤트 테이블 찾기
    QTableView* eventTable = dashboard_->findChild<QTableView*>("eventTableView");
    if (!eventTable || !eventTable->model()) return;
    
    // 이벤트 추가
    QStandardItemModel* model = qobject_cast<QStandardItemModel*>(eventTable->model());
    if (!model) return;
    
    QList<QStandardItem*> items;
    items.append(new QStandardItem(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")));
    items.append(new QStandardItem(robotName));
    items.append(new QStandardItem(eventType));
    items.append(new QStandardItem(description));
    
    model->insertRow(0, items);  // 최신 이벤트를 맨 위에 추가
    
    // 로그에도 추가
    QPlainTextEdit* logText = dashboard_->findChild<QPlainTextEdit*>("logTextEdit");
    if (logText) {
        QString logMessage;
        if (eventType == "연결") {
            logMessage = QString("[INFO] %1: %2").arg(robotName).arg(description);
        } else if (eventType == "장애") {
            logMessage = QString("[ERROR] %1: %2").arg(robotName).arg(description);
        } else if (eventType == "명령") {
            logMessage = QString("[INFO] %1: %2").arg(robotName).arg(description);
        } else {
            logMessage = QString("[INFO] %1: %2 - %3").arg(robotName).arg(eventType).arg(description);
        }
        
        logText->appendPlainText(logMessage);
    }
}

void MainWindow::updateRobotTabCheckboxes()
{
    // Jetcobot 탭과 Pinky 탭의 체크박스 상태를 업데이트
    QWidget* jetcobot_tab = central_widget_->widget(1); // Jetcobot 탭 인덱스
    QWidget* pinky_tab = central_widget_->widget(2); // Pinky 탭 인덱스
    
    if (jetcobot_tab && pinky_tab) {
        // Jetcobot 탭의 체크박스 컨테이너
        QScrollArea* jetcobot_scroll = jetcobot_tab->findChild<QScrollArea*>("robotSelectionArea");
        if (jetcobot_scroll && jetcobot_scroll->widget()) {
            QWidget* container = jetcobot_scroll->widget();
            QLayout* layout = container->layout();
            
            if (layout) {
                // 기존 체크박스 제거
                while (QLayoutItem* item = layout->takeAt(0)) {
                    if (item->widget()) {
                        delete item->widget();
                    }
                    delete item;
                }
                
                // 등록된 Jetcobot 로봇에 대한 체크박스 추가
                for (auto it = robots_.begin(); it != robots_.end(); ++it) {
                    if (it.value().type == RobotType::JETCOBOT) {
                        QCheckBox* checkbox = new QCheckBox(it.key());
                        layout->addWidget(checkbox);
                        
                        // 체크박스 상태 변경 이벤트 연결
                        connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::onRobotCheckStateChanged);
                        
                        // 멤버 변수 맵에 저장
                        robot_checkboxes_[it.key()] = checkbox;
                    }
                }
            }
        }
        
        // Pinky 탭의 체크박스 컨테이너
        QScrollArea* pinky_scroll = pinky_tab->findChild<QScrollArea*>("robotSelectionArea");
        if (pinky_scroll && pinky_scroll->widget()) {
            QWidget* container = pinky_scroll->widget();
            QLayout* layout = container->layout();
            
            if (layout) {
                // 기존 체크박스 제거
                while (QLayoutItem* item = layout->takeAt(0)) {
                    if (item->widget()) {
                        delete item->widget();
                    }
                    delete item;
                }
                
                // 등록된 Pinky 로봇에 대한 체크박스 추가
                for (auto it = robots_.begin(); it != robots_.end(); ++it) {
                    if (it.value().type == RobotType::PINKY) {
                        QCheckBox* checkbox = new QCheckBox(it.key());
                        layout->addWidget(checkbox);
                        
                        // 체크박스 상태 변경 이벤트 연결
                        connect(checkbox, &QCheckBox::stateChanged, this, &MainWindow::onRobotCheckStateChanged);
                        
                        // 멤버 변수 맵에 저장
                        robot_checkboxes_[it.key()] = checkbox;
                    }
                }
            }
        }
    }
}

void MainWindow::onScanTopicsClicked()
{
    // 토픽 스캔 시작
    statusBar()->showMessage("토픽 스캔 중...");
    
    // 토픽 트리 위젯 찾기
    QTreeWidget* topicTree = settings_tab_->findChild<QTreeWidget*>("topicTree");
    if (!topicTree) {
        qDebug() << "토픽 트리 위젯을 찾을 수 없음";
        return;
    }
    
    // 현재 선택 상태 저장 (토픽 이름 : 체크 상태)
    QMap<QString, bool> topicCheckedState;
    for (int i = 0; i < topicTree->topLevelItemCount(); ++i) {
        QTreeWidgetItem* robotItem = topicTree->topLevelItem(i);
        for (int j = 0; j < robotItem->childCount(); ++j) {
            QTreeWidgetItem* topicItem = robotItem->child(j);
            QString fullTopicName = robotItem->text(0) + "/" + topicItem->text(0);
            topicCheckedState[fullTopicName] = (topicItem->checkState(2) == Qt::Checked);
        }
    }
    
    // 트리 초기화
    topicTree->clear();
    
    // 로봇 도메인 상태 디버깅
    qDebug() << "로봇 도메인 상태 확인:";
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        const RobotInfo& robot = it.value();
        qDebug() << "  로봇:" << robot.name << ", 도메인 ID:" << robot.domain_id 
                 << ", 네임스페이스:" << robot.namespace_name;
    }
    
    // 각 로봇별로 토픽을 스캔하여 트리에 추가
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        const RobotInfo& robot = it.value();
        
        // 로봇 항목 추가
        QTreeWidgetItem* robotItem = new QTreeWidgetItem(topicTree, QStringList() << robot.name);
        robotItem->setIcon(0, QIcon::fromTheme("computer")); // 기본 아이콘
        robotItem->setExpanded(true); // 기본적으로 펼쳐진 상태
        
        // 로봇 네임스페이스와 도메인 ID 디버깅
        qDebug() << "로봇 토픽 스캔:" << robot.name 
                 << "(도메인 ID:" << robot.domain_id
                 << ", 네임스페이스:" << robot.namespace_name << ")";
        
        // 로봇 네임스페이스로 시작하는 토픽만 필터링
        QStringList robotTopics;
        QStringList robotTopicTypes;
        
        // 로봇의 도메인 ID를 사용하여 토픽 목록 가져오기
        if (topic_manager_) {
            try {
                // 로봇의 도메인 ID 설정 (현재 도메인을 백업하고 로봇 도메인으로 임시 변경)
                int current_domain_id = topic_manager_->getCurrentDomainId();
                qDebug() << "현재 도메인 ID:" << current_domain_id << ", 변경할 도메인 ID:" << robot.domain_id;
                
                // 해당 로봇의 도메인 ID로 설정
                topic_manager_->setDomainId(robot.domain_id);
                qDebug() << "도메인 ID를" << robot.domain_id << "로 변경하여 토픽 스캔 시작";
                
                // 토픽 목록 가져오기
                std::vector<std::pair<std::string, std::string>> topics = topic_manager_->getTopicList();
                qDebug() << "스캔된 토픽 수:" << topics.size();
                
                for (const auto& topic_pair : topics) {
                    QString topicName = QString::fromStdString(topic_pair.first);
                    QString topicType = QString::fromStdString(topic_pair.second);
                    
                    // 네임스페이스 필터링 (비어있을 경우 모든 토픽 포함)
                    bool include_topic = true;
                    if (!robot.namespace_name.isEmpty()) {
                        include_topic = topicName.startsWith(robot.namespace_name) || 
                                      (robot.namespace_name == "/" && topicName.startsWith("/"));
                    }
                    
                    if (include_topic) {
                        // 네임스페이스 제거하여 상대 경로로 표시
                        QString relativeTopicName = topicName;
                        if (!robot.namespace_name.isEmpty() && robot.namespace_name != "/" && 
                            topicName.startsWith(robot.namespace_name)) {
                            relativeTopicName = topicName.mid(robot.namespace_name.length());
                            if (relativeTopicName.startsWith("/")) {
                                relativeTopicName = relativeTopicName.mid(1);
                            }
                        }
                        
                        robotTopics << relativeTopicName;
                        robotTopicTypes << topicType;
                        qDebug() << "  토픽 추가:" << relativeTopicName << "(" << topicType << ")";
                    }
                }
                
                // 원래 도메인 ID로 복원
                topic_manager_->setDomainId(current_domain_id);
                qDebug() << "도메인 ID를" << current_domain_id << "로 복원";
                
            } catch (const std::exception& e) {
                qDebug() << "토픽 스캔 중 오류 발생:" << e.what();
                statusBar()->showMessage(QString("토픽 스캔 오류: %1").arg(e.what()));
            }
        } else {
            // 테스트용 데이터 (topic_manager가 없는 경우)
            qDebug() << "토픽 매니저 없음, 테스트 데이터 사용";
            if (robot.type == RobotType::JETCOBOT) {
                robotTopics << "coords_real" << "pose" << "velocity" << "battery" << "motor_state";
                robotTopicTypes << "geometry_msgs/msg/PoseStamped" << "geometry_msgs/msg/Pose" 
                               << "geometry_msgs/msg/Twist" << "std_msgs/msg/Float32" << "std_msgs/msg/String";
            } else if (robot.domain_id == 74) {
                // 74번 도메인에 대한 특별한 테스트 데이터 추가
                robotTopics << "odom" << "cmd_vel" << "map" << "scan" << "goal_pose" 
                           << "global_costmap/costmap" << "local_costmap/costmap" 
                           << "plan" << "tf" << "tf_static" << "robot_description" 
                           << "joint_states" << "pinky_battery_present" << "imu";
                           
                robotTopicTypes << "nav_msgs/msg/Odometry" << "geometry_msgs/msg/Twist"
                               << "nav_msgs/msg/OccupancyGrid" << "sensor_msgs/msg/LaserScan"
                               << "geometry_msgs/msg/PoseStamped" << "nav_msgs/msg/OccupancyGrid"
                               << "nav_msgs/msg/OccupancyGrid" << "nav_msgs/msg/Path"
                               << "tf2_msgs/msg/TFMessage" << "tf2_msgs/msg/TFMessage"
                               << "std_msgs/msg/String" << "sensor_msgs/msg/JointState"
                               << "std_msgs/msg/Bool" << "sensor_msgs/msg/Imu";
            } else {
                // 기본 Pinky 토픽
                robotTopics << "pose" << "map" << "path" << "cmd_vel" << "battery" << "status";
                robotTopicTypes << "geometry_msgs/msg/PoseStamped" << "nav_msgs/msg/OccupancyGrid" 
                               << "nav_msgs/msg/Path" << "geometry_msgs/msg/Twist" 
                               << "std_msgs/msg/Float32" << "std_msgs/msg/String";
            }
        }
        
        if (robotTopics.isEmpty()) {
            // 토픽이 없을 경우 안내 메시지 추가
            QTreeWidgetItem* noTopicItem = new QTreeWidgetItem(robotItem, 
                QStringList() << "토픽 없음" << "로봇에서 토픽을 찾을 수 없습니다." << "");
            noTopicItem->setForeground(0, QBrush(Qt::gray));
            noTopicItem->setForeground(1, QBrush(Qt::gray));
            qDebug() << "  해당 로봇에서 토픽을 찾을 수 없음";
        } else {
            // 로봇 토픽 추가
            for (int i = 0; i < robotTopics.size(); ++i) {
                QTreeWidgetItem* topicItem = new QTreeWidgetItem(robotItem, QStringList() 
                                                             << robotTopics[i] 
                                                             << robotTopicTypes[i]);
                
                // 구독 체크박스 설정
                topicItem->setFlags(topicItem->flags() | Qt::ItemIsUserCheckable);
                
                // 이전 체크 상태 복원
                QString fullTopicName = robot.name + "/" + robotTopics[i];
                bool isChecked = false;
                
                // 이전 상태가 있으면 복원, 없으면 기본값 설정
                if (topicCheckedState.contains(fullTopicName)) {
                    isChecked = topicCheckedState[fullTopicName];
                } else {
                    // 기본적으로 중요 토픽은 체크 상태로 설정
                    QStringList importantTopics = {"coords_real", "pose", "battery", "cmd_vel", "status", "odom", "scan", "map"};
                    isChecked = importantTopics.contains(robotTopics[i]);
                }
                
                topicItem->setCheckState(2, isChecked ? Qt::Checked : Qt::Unchecked);
                
                // 토픽 타입에 따라 아이콘 설정
                QString topicType = robotTopicTypes[i];
                if (topicType.contains("Pose") || topicType.contains("Point") || topicType.contains("Odometry")) {
                    topicItem->setIcon(0, QIcon::fromTheme("go-home"));
                } else if (topicType.contains("Twist") || topicType.contains("Velocity")) {
                    topicItem->setIcon(0, QIcon::fromTheme("media-playback-start"));
                } else if (topicType.contains("Float") || topicType.contains("Int")) {
                    topicItem->setIcon(0, QIcon::fromTheme("accessories-calculator"));
                } else if (topicType.contains("String")) {
                    topicItem->setIcon(0, QIcon::fromTheme("text-x-generic"));
                } else if (topicType.contains("LaserScan") || topicType.contains("PointCloud")) {
                    topicItem->setIcon(0, QIcon::fromTheme("zoom-in"));
                } else if (topicType.contains("OccupancyGrid") || topicType.contains("Map")) {
                    topicItem->setIcon(0, QIcon::fromTheme("map-symbolic"));
                } else if (topicType.contains("Path")) {
                    topicItem->setIcon(0, QIcon::fromTheme("text-x-script"));
                } else if (topicType.contains("TFMessage")) {
                    topicItem->setIcon(0, QIcon::fromTheme("object-flip-horizontal"));
                } else {
                    topicItem->setIcon(0, QIcon::fromTheme("text-plain"));
                }
            }
        }
    }
    
    statusBar()->showMessage("토픽 스캔 완료");
}

void MainWindow::onTopicItemChanged(QTreeWidgetItem* item, int column)
{
    // 구독 체크박스 열에서만 처리
    if (column != 2) return;
    
    // 부모 아이템 (로봇 항목)이 있는지 확인
    QTreeWidgetItem* parentItem = item->parent();
    if (!parentItem) return;
    
    // 로봇 이름과 토픽 이름 가져오기
    QString robotName = parentItem->text(0);
    QString topicName = item->text(0);
    QString topicType = item->text(1);
    
    // 로봇 정보 가져오기
    if (!robots_.contains(robotName)) return;
    const RobotInfo& robot = robots_[robotName];
    
    // 전체 토픽 경로 구성
    QString fullTopicPath = robot.namespace_name;
    if (!fullTopicPath.endsWith("/") && !topicName.startsWith("/")) {
        fullTopicPath += "/";
    }
    fullTopicPath += topicName;
    
    // 체크박스 상태에 따라 토픽 구독/해제
    bool isChecked = (item->checkState(column) == Qt::Checked);
    
    if (isChecked) {
        // 구독 시작
        if (topic_manager_) {
            topic_manager_->subscribeTopic(fullTopicPath.toStdString(), topicType.toStdString());
            qDebug() << "토픽 구독 시작:" << fullTopicPath;
            statusBar()->showMessage(QString("토픽 구독 시작: %1").arg(fullTopicPath));
        }
    } else {
        // 구독 해제
        if (topic_manager_) {
            topic_manager_->unsubscribeTopic(fullTopicPath.toStdString());
            qDebug() << "토픽 구독 해제:" << fullTopicPath;
            statusBar()->showMessage(QString("토픽 구독 해제: %1").arg(fullTopicPath));
        }
    }
}

void MainWindow::onTopicFilterChanged(const QString& text)
{
    // 필터링된 텍스트
    QString filter = text.trimmed().toLower();
    
    // 토픽 트리 위젯 찾기
    QTreeWidget* topicTree = settings_tab_->findChild<QTreeWidget*>("topicTree");
    if (!topicTree) return;
    
    // 빈 필터면 모든 항목 표시
    if (filter.isEmpty()) {
        for (int i = 0; i < topicTree->topLevelItemCount(); ++i) {
            QTreeWidgetItem* robotItem = topicTree->topLevelItem(i);
            robotItem->setHidden(false);
            
            for (int j = 0; j < robotItem->childCount(); ++j) {
                robotItem->child(j)->setHidden(false);
            }
        }
        return;
    }
    
    // 필터에 따라 항목 표시/숨김 처리
    for (int i = 0; i < topicTree->topLevelItemCount(); ++i) {
        QTreeWidgetItem* robotItem = topicTree->topLevelItem(i);
        bool anyChildVisible = false;
        
        for (int j = 0; j < robotItem->childCount(); ++j) {
            QTreeWidgetItem* topicItem = robotItem->child(j);
            
            // 토픽 이름이나 타입에 필터 문자열이 포함되어 있는지 확인
            bool visible = topicItem->text(0).toLower().contains(filter) || 
                           topicItem->text(1).toLower().contains(filter);
            
            topicItem->setHidden(!visible);
            
            if (visible) {
                anyChildVisible = true;
            }
        }
        
        // 로봇 항목은 자식 항목이 하나라도 표시되면 표시, 아니면 숨김
        robotItem->setHidden(!anyChildVisible);
    }
}

} // namespace robot_debugger_ui 