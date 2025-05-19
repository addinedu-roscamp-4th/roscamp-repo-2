#include "robot_debugger_ui/configuration_manager.hpp"
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>

namespace robot_debugger_ui {

ConfigurationManager::ConfigurationManager(const std::string& config_path, QObject* parent)
    : QObject(parent)
    , config_path_(config_path)
{
    // 기본 설정 파일 경로 설정
    if (config_path_.empty()) {
        QString config_dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
        QDir().mkpath(config_dir); // 디렉토리가 없으면 생성
        config_path_ = (config_dir + "/config.ini").toStdString();
    }
    
    // 설정 객체 초기화
    settings_ = std::make_unique<QSettings>(QString::fromStdString(config_path_), QSettings::IniFormat);
    
    // 설정 로드
    loadConfiguration();
}

ConfigurationManager::~ConfigurationManager() = default;

bool ConfigurationManager::loadConfiguration()
{
    // 도메인 패턴 로드
    QVariant domains_variant = settings_->value("Domains/Patterns");
    if (domains_variant.isValid()) {
        QStringList domains_str_list = domains_variant.toStringList();
        domain_patterns_.clear();
        for (const QString& domain_str : domains_str_list) {
            bool ok = false;
            int domain_id = domain_str.toInt(&ok);
            if (ok) {
                domain_patterns_.push_back(domain_id);
            }
        }
    } else {
        // 기본 도메인 패턴
        domain_patterns_ = {10, 20};
    }
    
    // 네임스페이스 패턴 로드
    QVariant namespaces_variant = settings_->value("Namespaces/Patterns");
    if (namespaces_variant.isValid()) {
        QStringList namespaces_str_list = namespaces_variant.toStringList();
        namespace_patterns_.clear();
        for (const QString& ns_str : namespaces_str_list) {
            namespace_patterns_.push_back(ns_str.toStdString());
        }
    } else {
        // 기본 네임스페이스 패턴
        namespace_patterns_ = {"jetcobot_1", "jetcobot_2", "pinky_1", "pinky_2", "pinky_3"};
    }
    
    // QoS 설정 로드
    settings_->beginGroup("QoS");
    QStringList topics = settings_->childGroups();
    for (const QString& topic : topics) {
        settings_->beginGroup(topic);
        
        QosConfiguration qos;
        qos.reliability = static_cast<QosConfiguration::Reliability>(
            settings_->value("Reliability", static_cast<int>(QosConfiguration::Reliability::RELIABLE)).toInt());
        qos.durability = static_cast<QosConfiguration::Durability>(
            settings_->value("Durability", static_cast<int>(QosConfiguration::Durability::VOLATILE)).toInt());
        qos.history_depth = settings_->value("HistoryDepth", 10).toInt();
        qos.timeout_ms = settings_->value("TimeoutMs", 1000).toInt();
        
        topic_qos_settings_[topic.toStdString()] = qos;
        
        settings_->endGroup();
    }
    settings_->endGroup();
    
    // 로그 설정 로드
    settings_->beginGroup("Log");
    log_config_.default_level = static_cast<LogConfiguration::LogLevel>(
        settings_->value("DefaultLevel", static_cast<int>(LogConfiguration::LogLevel::INFO)).toInt());
    log_config_.max_entries = settings_->value("MaxEntries", 1000).toInt();
    log_config_.save_to_file = settings_->value("SaveToFile", false).toBool();
    log_config_.log_file_path = settings_->value("LogFilePath", "").toString().toStdString();
    settings_->endGroup();
    
    // 설정 변경 알림
    emit configurationChanged();
    
    return true;
}

bool ConfigurationManager::saveConfiguration()
{
    // 도메인 패턴 저장
    QStringList domains_str_list;
    for (int domain_id : domain_patterns_) {
        domains_str_list.append(QString::number(domain_id));
    }
    settings_->setValue("Domains/Patterns", domains_str_list);
    
    // 네임스페이스 패턴 저장
    QStringList namespaces_str_list;
    for (const std::string& ns : namespace_patterns_) {
        namespaces_str_list.append(QString::fromStdString(ns));
    }
    settings_->setValue("Namespaces/Patterns", namespaces_str_list);
    
    // QoS 설정 저장
    settings_->beginGroup("QoS");
    for (const auto& entry : topic_qos_settings_) {
        const std::string& topic = entry.first;
        const QosConfiguration& qos = entry.second;
        
        settings_->beginGroup(QString::fromStdString(topic));
        settings_->setValue("Reliability", static_cast<int>(qos.reliability));
        settings_->setValue("Durability", static_cast<int>(qos.durability));
        settings_->setValue("HistoryDepth", qos.history_depth);
        settings_->setValue("TimeoutMs", qos.timeout_ms);
        settings_->endGroup();
    }
    settings_->endGroup();
    
    // 로그 설정 저장
    settings_->beginGroup("Log");
    settings_->setValue("DefaultLevel", static_cast<int>(log_config_.default_level));
    settings_->setValue("MaxEntries", log_config_.max_entries);
    settings_->setValue("SaveToFile", log_config_.save_to_file);
    settings_->setValue("LogFilePath", QString::fromStdString(log_config_.log_file_path));
    settings_->endGroup();
    
    // 설정 동기화
    settings_->sync();
    
    return (settings_->status() == QSettings::NoError);
}

std::vector<int> ConfigurationManager::getDomainPatterns() const
{
    return domain_patterns_;
}

void ConfigurationManager::setDomainPatterns(const std::vector<int>& domains)
{
    domain_patterns_ = domains;
    emit configurationChanged();
}

std::vector<std::string> ConfigurationManager::getNamespacePatterns() const
{
    return namespace_patterns_;
}

void ConfigurationManager::setNamespacePatterns(const std::vector<std::string>& namespaces)
{
    namespace_patterns_ = namespaces;
    emit configurationChanged();
}

ConfigurationManager::QosConfiguration ConfigurationManager::getQosConfiguration(const std::string& topic_name) const
{
    auto it = topic_qos_settings_.find(topic_name);
    if (it != topic_qos_settings_.end()) {
        return it->second;
    }
    
    // 기본 QoS 설정 반환
    QosConfiguration default_qos;
    default_qos.reliability = QosConfiguration::Reliability::RELIABLE;
    default_qos.durability = QosConfiguration::Durability::VOLATILE;
    default_qos.history_depth = 10;
    default_qos.timeout_ms = 1000;
    
    return default_qos;
}

void ConfigurationManager::setQosConfiguration(const std::string& topic_name, const QosConfiguration& config)
{
    topic_qos_settings_[topic_name] = config;
    emit configurationChanged();
}

ConfigurationManager::LogConfiguration ConfigurationManager::getLogConfiguration() const
{
    return log_config_;
}

void ConfigurationManager::setLogConfiguration(const LogConfiguration& config)
{
    log_config_ = config;
    emit configurationChanged();
}

} // namespace robot_debugger_ui 