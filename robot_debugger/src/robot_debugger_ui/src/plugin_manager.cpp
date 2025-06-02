#include "robot_debugger_ui/plugin_manager.hpp"
#include <QApplication>
#include <QDebug>

namespace robot_debugger_ui {

PluginManager::PluginManager(QObject* parent)
    : QObject(parent)
{
    // 플러그인 정보 로드
    loadPluginInfos();
}

PluginManager::~PluginManager()
{
    // 모든 로드된 플러그인 언로드
    for (auto it = loaded_plugins_.begin(); it != loaded_plugins_.end(); ++it) {
        unloadPlugin(it->first);
    }
}

std::vector<PluginManager::PluginInfo> PluginManager::scanPluginDirectory(const QString& dir_path)
{
    std::vector<PluginInfo> discovered_plugins;
    
    QDir plugin_dir(dir_path);
    QStringList entry_list = plugin_dir.entryList(QDir::Files);
    
    for (const QString& file_name : entry_list) {
        // 플러그인 파일 (확장자에 따라 필터링)
        if (QLibrary::isLibrary(file_name)) {
            QString absolute_path = plugin_dir.absoluteFilePath(file_name);
            
            // 플러그인 로더 생성하여 메타데이터 확인
            QPluginLoader loader(absolute_path);
            QJsonObject metadata = loader.metaData().value("MetaData").toObject();
            
            if (!metadata.isEmpty()) {
                PluginInfo info;
                info.id = metadata.value("id").toString();
                info.name = metadata.value("name").toString();
                info.version = metadata.value("version").toString();
                info.description = metadata.value("description").toString();
                info.file_path = absolute_path;
                info.is_loaded = false;
                info.is_enabled = false;
                
                // 이미 알려진 플러그인인지 확인하고 정보 업데이트
                auto it = plugin_infos_.find(info.id);
                if (it != plugin_infos_.end()) {
                    // 기존 설정 유지
                    info.settings = it->second.settings;
                    info.is_enabled = it->second.is_enabled;
                }
                
                discovered_plugins.push_back(info);
                plugin_infos_[info.id] = info;
            }
        }
    }
    
    // 플러그인 정보 저장
    savePluginInfos();
    
    return discovered_plugins;
}

PluginInterface* PluginManager::loadPlugin(const QString& plugin_id, QWidget* parent_widget)
{
    // 이미 로드되었는지 확인
    auto it = loaded_plugins_.find(plugin_id);
    if (it != loaded_plugins_.end()) {
        return it->second;
    }
    
    // 플러그인 정보 찾기
    auto info_it = plugin_infos_.find(plugin_id);
    if (info_it == plugin_infos_.end()) {
        qWarning() << "플러그인 ID를 찾을 수 없음:" << plugin_id;
        return nullptr;
    }
    
    PluginInfo& info = info_it->second;
    
    // 이미 로더가 있는지 확인
    QPluginLoader* loader = nullptr;
    auto loader_it = plugin_loaders_.find(plugin_id);
    
    if (loader_it == plugin_loaders_.end()) {
        // 새 로더 생성
        loader = new QPluginLoader(info.file_path);
        plugin_loaders_[plugin_id] = loader;
    } else {
        loader = loader_it->second;
    }
    
    // 플러그인 로드
    if (!loader->isLoaded() && !loader->load()) {
        qWarning() << "플러그인 로드 실패:" << info.file_path;
        qWarning() << "오류:" << loader->errorString();
        return nullptr;
    }
    
    // 인스턴스 가져오기
    QObject* instance = loader->instance();
    if (!instance) {
        qWarning() << "플러그인 인스턴스 생성 실패:" << info.file_path;
        return nullptr;
    }
    
    // 인터페이스 캐스팅
    PluginInterface* plugin = dynamic_cast<PluginInterface*>(instance);
    if (!plugin) {
        qWarning() << "플러그인 인터페이스 불일치:" << info.file_path;
        loader->unload();
        return nullptr;
    }
    
    // 플러그인 초기화
    if (!plugin->initialize(parent_widget, info.settings)) {
        qWarning() << "플러그인 초기화 실패:" << info.file_path;
        loader->unload();
        return nullptr;
    }
    
    // 플러그인 저장
    loaded_plugins_[plugin_id] = plugin;
    info.is_loaded = true;
    
    emit pluginLoaded(plugin_id, true);
    qInfo() << "플러그인 로드 성공:" << plugin_id;
    
    return plugin;
}

bool PluginManager::unloadPlugin(const QString& plugin_id)
{
    // 플러그인 찾기
    auto plugin_it = loaded_plugins_.find(plugin_id);
    if (plugin_it == loaded_plugins_.end()) {
        return false; // 로드되지 않음
    }
    
    // 로더 찾기
    auto loader_it = plugin_loaders_.find(plugin_id);
    if (loader_it == plugin_loaders_.end()) {
        return false; // 로더 없음
    }
    
    PluginInterface* plugin = plugin_it->second;
    QPluginLoader* loader = loader_it->second;
    
    // 플러그인 설정 저장
    QVariantMap settings = plugin->settings();
    savePluginSettings(plugin_id, settings);
    
    // 플러그인 종료
    plugin->shutdown();
    
    // 로더 언로드
    bool success = loader->unload();
    
    if (success) {
        // 맵에서 제거
        loaded_plugins_.erase(plugin_it);
        
        // 플러그인 정보 업데이트
        auto info_it = plugin_infos_.find(plugin_id);
        if (info_it != plugin_infos_.end()) {
            info_it->second.is_loaded = false;
        }
        
        emit pluginUnloaded(plugin_id);
    }
    
    return success;
}

bool PluginManager::enablePlugin(const QString& plugin_id)
{
    auto it = plugin_infos_.find(plugin_id);
    if (it == plugin_infos_.end()) {
        return false;
    }
    
    it->second.is_enabled = true;
    savePluginInfos();
    emit pluginEnabled(plugin_id);
    
    return true;
}

bool PluginManager::disablePlugin(const QString& plugin_id)
{
    auto it = plugin_infos_.find(plugin_id);
    if (it == plugin_infos_.end()) {
        return false;
    }
    
    // 먼저 언로드
    if (it->second.is_loaded) {
        unloadPlugin(plugin_id);
    }
    
    it->second.is_enabled = false;
    savePluginInfos();
    emit pluginDisabled(plugin_id);
    
    return true;
}

bool PluginManager::savePluginSettings(const QString& plugin_id, const QVariantMap& settings)
{
    auto it = plugin_infos_.find(plugin_id);
    if (it == plugin_infos_.end()) {
        return false;
    }
    
    it->second.settings = settings;
    savePluginInfos();
    emit pluginSettingsChanged(plugin_id);
    
    return true;
}

PluginManager::PluginInfo PluginManager::getPluginInfo(const QString& plugin_id) const
{
    auto it = plugin_infos_.find(plugin_id);
    if (it != plugin_infos_.end()) {
        return it->second;
    }
    
    return PluginInfo();
}

std::vector<PluginManager::PluginInfo> PluginManager::getAllPluginInfo() const
{
    std::vector<PluginInfo> infos;
    infos.reserve(plugin_infos_.size());
    
    for (const auto& pair : plugin_infos_) {
        infos.push_back(pair.second);
    }
    
    return infos;
}

QWidget* PluginManager::getPluginWidget(const QString& plugin_id) const
{
    auto it = loaded_plugins_.find(plugin_id);
    if (it != loaded_plugins_.end()) {
        return it->second->widget();
    }
    
    return nullptr;
}

QVariantMap PluginManager::getPluginSettings(const QString& plugin_id) const
{
    auto it = plugin_infos_.find(plugin_id);
    if (it != plugin_infos_.end()) {
        return it->second.settings;
    }
    
    return QVariantMap();
}

bool PluginManager::savePluginInfos()
{
    // 파일에 플러그인 정보 저장 구현
    // TODO: 설정 파일에 저장 구현
    return true;
}

bool PluginManager::loadPluginInfos()
{
    // 파일에서 플러그인 정보 로드 구현
    // TODO: 설정 파일에서 로드 구현
    return true;
}

} // namespace robot_debugger_ui 