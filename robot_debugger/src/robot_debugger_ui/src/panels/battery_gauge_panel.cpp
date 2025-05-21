#include "robot_debugger_ui/panels/battery_gauge_panel.hpp"
#include <QStyleOption>
#include <QPainter>
#include <QDialog>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QRegularExpression>
#include <QDebug>

// QCustomPlot이 사용 가능한 경우에만 QCustomPlot 헤더 포함
#ifndef NO_QCUSTOMPLOT
#include "qcustomplot.h"
#endif

namespace robot_debugger_ui {

// 정적 멤버 변수 초기화
const QString BatteryGaugePanel::PANEL_ID = "battery_gauge_panel";
const QString BatteryGaugePanel::PANEL_TITLE = "배터리 게이지";
const QString BatteryGaugePanel::PANEL_DESCRIPTION = "로봇의 배터리 상태를 모니터링합니다";

BatteryGaugePanel::BatteryGaugePanel(
    std::shared_ptr<TopicManager> topic_manager,
    std::shared_ptr<MessageStore> message_store,
    QWidget* parent)
    : PanelInterface(parent)
    , topic_manager_(topic_manager)
    , message_store_(message_store)
    , title_label_(nullptr)
    , topic_combo_(nullptr)
    , refresh_button_(nullptr)
    , battery_progress_(nullptr)
    , voltage_label_(nullptr)
    , percentage_label_(nullptr)
    , status_label_(nullptr)
    , update_timer_(nullptr)
    , current_topic_("")
    , last_percentage_(0.0)
    , last_voltage_(0.0)
{
    setupUi();
    
    // 초기 설정 값
    panel_settings_["update_interval"] = 1000;  // 1초
    panel_settings_["topic_pattern"] = "battery_state";
    panel_settings_["low_threshold"] = 20.0;  // 20% 이하일 때 경고
    
    // 시그널/슬롯 연결
    connect(topic_combo_, &QComboBox::currentTextChanged,
            this, &BatteryGaugePanel::onTopicSelected);
    
    connect(refresh_button_, &QPushButton::clicked,
            this, &BatteryGaugePanel::onRefreshTopics);
    
    if (topic_manager_) {
        connect(topic_manager_.get(), &TopicManager::topicsChanged,
                this, &BatteryGaugePanel::onTopicsChanged);
    }
    
    if (message_store_) {
        connect(message_store_.get(), &MessageStore::messageStored,
                this, &BatteryGaugePanel::onMessageReceived);
    }
    
    // 업데이트 타이머 설정
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout,
            this, &BatteryGaugePanel::onUpdateTimer);
    
    // 초기 토픽 목록 갱신
    updateBatteryTopics();
}

BatteryGaugePanel::~BatteryGaugePanel()
{
    if (update_timer_) {
        update_timer_->stop();
    }
}

QString BatteryGaugePanel::id() const
{
    return PANEL_ID;
}

QString BatteryGaugePanel::title() const
{
    return PANEL_TITLE;
}

QString BatteryGaugePanel::description() const
{
    return PANEL_DESCRIPTION;
}

QIcon BatteryGaugePanel::icon() const
{
    // 기본 아이콘 반환
    return QIcon::fromTheme("battery");
}

QVariantMap BatteryGaugePanel::settings() const
{
    return panel_settings_;
}

void BatteryGaugePanel::saveSettings(const QVariantMap& settings)
{
    panel_settings_ = settings;
    
    // 설정 적용
    if (settings.contains("update_interval")) {
        int interval = settings["update_interval"].toInt();
        if (interval > 0) {
            update_timer_->setInterval(interval);
        }
    }
    
    if (settings.contains("topic_pattern")) {
        // 토픽 패턴 변경 시 목록 다시 갱신
        updateBatteryTopics();
    }
    
    // 설정 변경 알림
    emit settingsChanged(panel_settings_);
}

bool BatteryGaugePanel::initialize(const QVariantMap& settings)
{
    saveSettings(settings);
    
    // 타이머 시작
    update_timer_->start(panel_settings_["update_interval"].toInt());
    
    return true;
}

void BatteryGaugePanel::update()
{
    // 현재 선택된 토픽의 최신 메시지로 상태 업데이트
    if (!current_topic_.isEmpty() && message_store_) {
        auto msg = message_store_->getLatestMessage(current_topic_.toStdString());
        if (!msg.data.empty()) {
            // 메시지 데이터에서 배터리 상태 추출
            // 실제 구현에서는 메시지 타입에 따라 적절히 처리해야 함
            // 여기서는 임시로 간단한 예시만 제공
            updateBatteryStatus(msg.topic_name, last_percentage_, last_voltage_);
        }
    }
}

void BatteryGaugePanel::stop()
{
    if (update_timer_) {
        update_timer_->stop();
    }
}

void BatteryGaugePanel::shutdown()
{
    stop();
}

bool BatteryGaugePanel::hasConfigDialog() const
{
    return true;
}

bool BatteryGaugePanel::showConfigDialog()
{
    QDialog dialog(this);
    dialog.setWindowTitle(tr("배터리 게이지 설정"));
    
    QFormLayout* layout = new QFormLayout(&dialog);
    
    // 토픽 패턴 입력
    QLineEdit* topicPatternEdit = new QLineEdit(&dialog);
    topicPatternEdit->setText(panel_settings_["topic_pattern"].toString());
    layout->addRow(tr("토픽 패턴:"), topicPatternEdit);
    
    // 업데이트 간격 입력
    QSpinBox* updateIntervalBox = new QSpinBox(&dialog);
    updateIntervalBox->setRange(100, 10000);  // 100ms ~ 10s
    updateIntervalBox->setSingleStep(100);
    updateIntervalBox->setSuffix(" ms");
    updateIntervalBox->setValue(panel_settings_["update_interval"].toInt());
    layout->addRow(tr("업데이트 간격:"), updateIntervalBox);
    
    // 배터리 부족 경고 임계값
    QSpinBox* lowThresholdBox = new QSpinBox(&dialog);
    lowThresholdBox->setRange(0, 50);  // 0% ~ 50%
    lowThresholdBox->setSingleStep(5);
    lowThresholdBox->setSuffix(" %");
    lowThresholdBox->setValue(panel_settings_["low_threshold"].toDouble());
    layout->addRow(tr("부족 경고 임계값:"), lowThresholdBox);
    
    // 확인/취소 버튼
    QDialogButtonBox* buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addRow(buttonBox);
    
    connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    
    if (dialog.exec() == QDialog::Accepted) {
        // 설정 저장
        QVariantMap new_settings = panel_settings_;
        new_settings["topic_pattern"] = topicPatternEdit->text();
        new_settings["update_interval"] = updateIntervalBox->value();
        new_settings["low_threshold"] = lowThresholdBox->value();
        
        saveSettings(new_settings);
        return true;
    }
    
    return false;
}

void BatteryGaugePanel::onTopicSelected(const QString& topic_name)
{
    if (current_topic_ != topic_name) {
        current_topic_ = topic_name;
        
        // 상태 업데이트
        status_label_->setText(tr("토픽 '%1' 선택됨").arg(topic_name));
        
        // 최신 메시지 가져와서 표시
        update();
    }
}

void BatteryGaugePanel::onUpdateTimer()
{
    update();
}

void BatteryGaugePanel::onRefreshTopics()
{
    if (topic_manager_) {
        topic_manager_->updateTopics();
    }
    
    updateBatteryTopics();
}

void BatteryGaugePanel::onTopicsChanged(const std::vector<TopicManager::TopicInfo>& /*topics*/)
{
    updateBatteryTopics();
}

void BatteryGaugePanel::onMessageReceived(const MessageStore::StoredMessage& message)
{
    // 현재 선택된 토픽에 대한 메시지만 처리
    if (QString::fromStdString(message.topic_name) == current_topic_) {
        // 여기서는 실제 메시지 처리를 위한 임시 예시
        // 실제 코드에서는 메시지 타입에 따라 적절히 처리해야 함
        double percentage = 75.0;  // 임시 예시
        double voltage = 11.5;     // 임시 예시
        
        // ROS 2 메시지 타입에 따라 배터리 정보 추출
        // sensor_msgs/BatteryState 타입인 경우의 예시:
        // if (message.type_name == "sensor_msgs/msg/BatteryState") {
        //     // 메시지 역직렬화 및 처리
        //     ...
        // }
        
        updateBatteryStatus(message.topic_name, percentage, voltage);
    }
}

void BatteryGaugePanel::setupUi()
{
    // 메인 레이아웃
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(4);
    
    // 타이틀
    title_label_ = new QLabel(PANEL_TITLE, this);
    QFont titleFont = title_label_->font();
    titleFont.setBold(true);
    titleFont.setPointSize(titleFont.pointSize() + 1);
    title_label_->setFont(titleFont);
    mainLayout->addWidget(title_label_);
    
    // 토픽 선택 레이아웃
    QHBoxLayout* topicLayout = new QHBoxLayout();
    QLabel* topicLabel = new QLabel(tr("토픽:"), this);
    topic_combo_ = new QComboBox(this);
    topic_combo_->setEditable(false);
    refresh_button_ = new QPushButton(tr("새로고침"), this);
    refresh_button_->setToolTip(tr("토픽 목록 새로고침"));
    
    topicLayout->addWidget(topicLabel);
    topicLayout->addWidget(topic_combo_, 1);
    topicLayout->addWidget(refresh_button_);
    mainLayout->addLayout(topicLayout);
    
    // 배터리 프로그레스바
    QGroupBox* batteryGroup = new QGroupBox(tr("배터리 상태"), this);
    QVBoxLayout* batteryLayout = new QVBoxLayout(batteryGroup);
    
    battery_progress_ = new QProgressBar(this);
    battery_progress_->setRange(0, 100);
    battery_progress_->setValue(0);
    battery_progress_->setFormat("%p%");
    battery_progress_->setTextVisible(true);
    batteryLayout->addWidget(battery_progress_);
    
    // 상태 레이블 레이아웃
    QHBoxLayout* statusLayout = new QHBoxLayout();
    
    percentage_label_ = new QLabel(tr("잔량: 0%"), this);
    voltage_label_ = new QLabel(tr("전압: 0.0V"), this);
    
    statusLayout->addWidget(percentage_label_);
    statusLayout->addStretch(1);
    statusLayout->addWidget(voltage_label_);
    
    batteryLayout->addLayout(statusLayout);
    mainLayout->addWidget(batteryGroup);
    
#ifndef NO_QCUSTOMPLOT
    // QCustomPlot이 있을 때만 그래프 추가
    QGroupBox* graphGroup = new QGroupBox(tr("배터리 그래프"), this);
    QVBoxLayout* graphLayout = new QVBoxLayout(graphGroup);
    
    QCustomPlot* customPlot = new QCustomPlot(this);
    customPlot->setMinimumHeight(150);
    graphLayout->addWidget(customPlot);
    
    // 그래프 설정
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::blue));
    customPlot->xAxis->setLabel(tr("시간 (초)"));
    customPlot->yAxis->setLabel(tr("배터리 (%)"));
    customPlot->xAxis->setRange(0, 60);
    customPlot->yAxis->setRange(0, 100);
    customPlot->setInteraction(QCP::iRangeDrag, true);
    customPlot->setInteraction(QCP::iRangeZoom, true);
    
    mainLayout->addWidget(graphGroup);
#endif
    
    // 상태 레이블
    status_label_ = new QLabel(tr("준비 완료"), this);
    status_label_->setWordWrap(true);
    mainLayout->addWidget(status_label_);
    
    // 여백 추가
    mainLayout->addStretch(1);
    
    // 스타일 설정
    battery_progress_->setStyleSheet(
        "QProgressBar {"
        "   border: 1px solid #AAAAAA;"
        "   border-radius: 5px;"
        "   text-align: center;"
        "   height: 20px;"
        "}"
        "QProgressBar::chunk {"
        "   background-color: qlineargradient(spread:pad, x1:0, y1:0.5, x2:1, y2:0.5, "
        "       stop:0 #FF0000, stop:0.3 #FFFF00, stop:1 #00FF00);"
        "}"
    );
}

void BatteryGaugePanel::updateBatteryTopics()
{
    if (!topic_manager_) {
        return;
    }
    
    QString currentSelection = topic_combo_->currentText();
    topic_combo_->clear();
    
    QString pattern = panel_settings_["topic_pattern"].toString();
    
    // 토픽 목록 가져오기
    auto topics = topic_manager_->getTopicInfoList();
    
    // 패턴에 맞는 토픽 필터링
    QRegularExpression regex(pattern, QRegularExpression::CaseInsensitiveOption);
    
    for (const auto& topic_info : topics) {
        QString topic_name = QString::fromStdString(topic_info.name);
        if (regex.match(topic_name).hasMatch()) {
            topic_combo_->addItem(topic_name);
        }
    }
    
    // 이전 선택 복원
    int index = topic_combo_->findText(currentSelection);
    if (index >= 0) {
        topic_combo_->setCurrentIndex(index);
    } else if (topic_combo_->count() > 0) {
        // 항목이 있으면 첫 번째 항목 선택
        topic_combo_->setCurrentIndex(0);
    }
    
    // 상태 업데이트
    status_label_->setText(tr("토픽 목록 갱신됨: %1개 토픽").arg(topic_combo_->count()));
}

void BatteryGaugePanel::updateBatteryStatus(const std::string& /*topic_name*/, double percentage, double voltage)
{
    last_percentage_ = percentage;
    last_voltage_ = voltage;
    
    // UI 업데이트
    battery_progress_->setValue(static_cast<int>(percentage));
    percentage_label_->setText(tr("잔량: %1%").arg(percentage, 0, 'f', 1));
    voltage_label_->setText(tr("전압: %1V").arg(voltage, 0, 'f', 2));
    
    // 배터리 부족 경고
    double low_threshold = panel_settings_["low_threshold"].toDouble();
    if (percentage <= low_threshold) {
        battery_progress_->setStyleSheet(
            "QProgressBar {"
            "   border: 1px solid #AAAAAA;"
            "   border-radius: 5px;"
            "   text-align: center;"
            "   height: 20px;"
            "}"
            "QProgressBar::chunk {"
            "   background-color: red;"
            "}"
        );
        status_label_->setText(tr("경고: 배터리 부족! (%1%)").arg(percentage, 0, 'f', 1));
    } else {
        battery_progress_->setStyleSheet(
            "QProgressBar {"
            "   border: 1px solid #AAAAAA;"
            "   border-radius: 5px;"
            "   text-align: center;"
            "   height: 20px;"
            "}"
            "QProgressBar::chunk {"
            "   background-color: qlineargradient(spread:pad, x1:0, y1:0.5, x2:1, y2:0.5, "
            "       stop:0 #FF0000, stop:0.3 #FFFF00, stop:1 #00FF00);"
            "}"
        );
        status_label_->setText(tr("정상 상태"));
    }
    
#ifndef NO_QCUSTOMPLOT
    // QCustomPlot이 있을 때만 그래프 업데이트
    // TODO: 그래프 업데이트 코드 추가
    // 자식 위젯에서 QCustomPlot 찾기
    for (QObject* child : this->children()) {
        QCustomPlot* plot = qobject_cast<QCustomPlot*>(child);
        if (plot) {
            // 현재 시간 가져오기
            static double time = 0;
            time += 1.0; // 예시 - 실제로는 ROS 타임스탬프 사용 필요
            
            // 데이터 추가
            plot->graph(0)->addData(time, percentage);
            
            // x축 범위 자동 조정 (60초 범위 유지)
            double key = plot->graph(0)->dataMainKey(plot->graph(0)->dataCount() - 1);
            plot->xAxis->setRange(key, 60, Qt::AlignRight);
            
            // 그래프 갱신
            plot->replot();
            break;
        }
    }
#endif
}

} // namespace robot_debugger_ui 