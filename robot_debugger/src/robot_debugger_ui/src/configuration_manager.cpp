#include "robot_debugger_ui/configuration_manager.hpp"
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QStandardPaths>
#include <QDebug>

namespace robot_debugger_ui {

ConfigurationManager::ConfigurationManager()
    : QObject(nullptr)
    , settings_(nullptr)
    , log_level_(1) // 기본값: INFO
{
    // 애플리케이션 설정 파일 위치 설정
    QString configPath = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir dir(configPath);
    
    // 디렉토리가 없으면 생성
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    
    settings_ = new QSettings(configPath + "/robot_debugger_ui.conf", QSettings::IniFormat);
    
    // 기본 설정 초기화
    initDefaults();
    
    // 기존 설정 로드
    loadFromFile("");
}

ConfigurationManager::~ConfigurationManager()
{
    // 설정 저장
    saveToFile("");
    
    if (settings_) {
        delete settings_;
        settings_ = nullptr;
    }
}

void ConfigurationManager::initDefaults()
{
    // 기본 도메인 패턴: 기본 ROS 2 도메인 ID(0)
    domain_patterns_ = {0};
    
    // 기본 네임스페이스 패턴: Jetcobot 및 Pinky 관련 네임스페이스
    namespace_patterns_ = {"jetcobot_1", "jetcobot_2", "pinky_1", "pinky_2", "pinky_3"};
    
    // 기본 로그 레벨: INFO
    log_level_ = 1;
}

bool ConfigurationManager::loadFromFile(const std::string& filename)
{
    if (!settings_) {
        return false;
    }
    
    // 파일 이름이 지정되면 해당 파일 사용, 그렇지 않으면 기본 설정 파일 사용
    QSettings* settingsToUse = settings_;
    QSettings* tempSettings = nullptr;
    
    if (!filename.empty()) {
        tempSettings = new QSettings(QString::fromStdString(filename), QSettings::IniFormat);
        settingsToUse = tempSettings;
    }
    
    // 도메인 패턴 로드
    int domainCount = settingsToUse->beginReadArray("DomainPatterns");
    domain_patterns_.clear();
    for (int i = 0; i < domainCount; ++i) {
        settingsToUse->setArrayIndex(i);
        domain_patterns_.push_back(settingsToUse->value("domain").toInt());
    }
    settingsToUse->endArray();
    
    // 도메인이 비어있다면 기본값 설정
    if (domain_patterns_.empty()) {
        domain_patterns_ = {0};
    }
    
    // 네임스페이스 패턴 로드
    int nsCount = settingsToUse->beginReadArray("NamespacePatterns");
    namespace_patterns_.clear();
    for (int i = 0; i < nsCount; ++i) {
        settingsToUse->setArrayIndex(i);
        namespace_patterns_.push_back(
            settingsToUse->value("namespace").toString().toStdString());
    }
    settingsToUse->endArray();
    
    // 네임스페이스가 비어있다면 기본값 설정
    if (namespace_patterns_.empty()) {
        namespace_patterns_ = {"jetcobot_1", "jetcobot_2", "pinky_1", "pinky_2", "pinky_3"};
    }
    
    // 토픽 설정 로드
    int topicCount = settingsToUse->beginReadArray("TopicConfigurations");
    topic_configs_.clear();
    for (int i = 0; i < topicCount; ++i) {
        settingsToUse->setArrayIndex(i);
        
        std::string topic_name = settingsToUse->value("topic").toString().toStdString();
        bool enabled = settingsToUse->value("enabled").toBool();
        
        QVariantMap qos;
        qos["reliability"] = settingsToUse->value("reliability").toString();
        qos["durability"] = settingsToUse->value("durability").toString();
        qos["history_depth"] = settingsToUse->value("history_depth").toInt();
        qos["timeout_ms"] = settingsToUse->value("timeout_ms").toInt();
        
        QVariantMap topicConfig;
        topicConfig["enabled"] = enabled;
        topicConfig["qos"] = qos;
        
        topic_configs_[topic_name] = topicConfig;
    }
    settingsToUse->endArray();
    
    // 레이아웃 설정 로드
    int layoutCount = settingsToUse->beginReadArray("Layouts");
    layouts_.clear();
    for (int i = 0; i < layoutCount; ++i) {
        settingsToUse->setArrayIndex(i);
        
        std::string window_name = settingsToUse->value("window").toString().toStdString();
        QByteArray layout = settingsToUse->value("layout").toByteArray();
        
        layouts_[window_name] = layout;
    }
    settingsToUse->endArray();
    
    // 로그 레벨 로드
    log_level_ = settingsToUse->value("LogLevel", 1).toInt();
    
    // 임시 설정 객체 정리
    if (tempSettings) {
        delete tempSettings;
    }
    
    // 설정 변경 시그널 발생
    emit configurationChanged();
    
    return true;
}

bool ConfigurationManager::saveToFile(const std::string& filename)
{
    if (!settings_) {
        return false;
    }
    
    // 파일 이름이 지정되면 해당 파일 사용, 그렇지 않으면 기본 설정 파일 사용
    QSettings* settingsToUse = settings_;
    QSettings* tempSettings = nullptr;
    
    if (!filename.empty()) {
        tempSettings = new QSettings(QString::fromStdString(filename), QSettings::IniFormat);
        settingsToUse = tempSettings;
    }
    
    // 도메인 패턴 저장
    settingsToUse->beginWriteArray("DomainPatterns", static_cast<int>(domain_patterns_.size()));
    for (size_t i = 0; i < domain_patterns_.size(); ++i) {
        settingsToUse->setArrayIndex(static_cast<int>(i));
        settingsToUse->setValue("domain", domain_patterns_[i]);
    }
    settingsToUse->endArray();
    
    // 네임스페이스 패턴 저장
    settingsToUse->beginWriteArray("NamespacePatterns", static_cast<int>(namespace_patterns_.size()));
    for (size_t i = 0; i < namespace_patterns_.size(); ++i) {
        settingsToUse->setArrayIndex(static_cast<int>(i));
        settingsToUse->setValue("namespace", QString::fromStdString(namespace_patterns_[i]));
    }
    settingsToUse->endArray();
    
    // 토픽 설정 저장
    settingsToUse->beginWriteArray("TopicConfigurations", static_cast<int>(topic_configs_.size()));
    int i = 0;
    for (const auto& [topic_name, config] : topic_configs_) {
        settingsToUse->setArrayIndex(i++);
        
        settingsToUse->setValue("topic", QString::fromStdString(topic_name));
        settingsToUse->setValue("enabled", config["enabled"].toBool());
        
        QVariantMap qos = config["qos"].toMap();
        settingsToUse->setValue("reliability", qos["reliability"].toString());
        settingsToUse->setValue("durability", qos["durability"].toString());
        settingsToUse->setValue("history_depth", qos["history_depth"].toInt());
        settingsToUse->setValue("timeout_ms", qos["timeout_ms"].toInt());
    }
    settingsToUse->endArray();
    
    // 레이아웃 설정 저장
    settingsToUse->beginWriteArray("Layouts", static_cast<int>(layouts_.size()));
    i = 0;
    for (const auto& [window_name, layout] : layouts_) {
        settingsToUse->setArrayIndex(i++);
        
        settingsToUse->setValue("window", QString::fromStdString(window_name));
        settingsToUse->setValue("layout", layout);
    }
    settingsToUse->endArray();
    
    // 로그 레벨 저장
    settingsToUse->setValue("LogLevel", log_level_);
    
    // 설정 파일에 즉시 기록
    settingsToUse->sync();
    
    // 임시 설정 객체 정리
    if (tempSettings) {
        delete tempSettings;
    }
    
    return true;
}

void ConfigurationManager::setDomainPatterns(const std::vector<int>& domains)
{
    domain_patterns_ = domains;
    emit configurationChanged();
}

std::vector<int> ConfigurationManager::getDomainPatterns() const
{
    return domain_patterns_;
}

void ConfigurationManager::setNamespacePatterns(const std::vector<std::string>& namespaces)
{
    namespace_patterns_ = namespaces;
    emit configurationChanged();
}

std::vector<std::string> ConfigurationManager::getNamespacePatterns() const
{
    return namespace_patterns_;
}

void ConfigurationManager::setTopicConfig(const std::string& topic_name, bool enabled, const QVariantMap& qos)
{
    QVariantMap config;
    config["enabled"] = enabled;
    config["qos"] = qos;
    
    topic_configs_[topic_name] = config;
    emit configurationChanged();
}

QVariantMap ConfigurationManager::getTopicConfig(const std::string& topic_name) const
{
    auto it = topic_configs_.find(topic_name);
    if (it != topic_configs_.end()) {
        return it->second;
    }
    
    // 기본 설정 반환
    QVariantMap config;
    config["enabled"] = true;
    
    QVariantMap qos;
    qos["reliability"] = "RELIABLE";
    qos["durability"] = "VOLATILE";
    qos["history_depth"] = 10;
    qos["timeout_ms"] = 500;
    
    config["qos"] = qos;
    
    return config;
}

std::map<std::string, QVariantMap> ConfigurationManager::getAllTopicConfigs() const
{
    return topic_configs_;
}

void ConfigurationManager::saveLayout(const std::string& window_name, const QByteArray& layout)
{
    layouts_[window_name] = layout;
    emit configurationChanged();
}

QByteArray ConfigurationManager::loadLayout(const std::string& window_name) const
{
    auto it = layouts_.find(window_name);
    if (it != layouts_.end()) {
        return it->second;
    }
    
    return QByteArray();
}

void ConfigurationManager::setLogLevel(int level)
{
    if (level >= 0 && level <= 4) {
        log_level_ = level;
        emit configurationChanged();
    }
}

int ConfigurationManager::getLogLevel() const
{
    return log_level_;
}

} // namespace robot_debugger_ui 