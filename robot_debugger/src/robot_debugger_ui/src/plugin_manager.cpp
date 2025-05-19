#include "robot_debugger_ui/plugin_manager.hpp"
#include <QApplication>
#include <QDebug>

namespace robot_debugger_ui {

PluginManager::PluginManager(const std::string& plugin_dir, QObject* parent)
    : QObject(parent)
    , plugin_dir_(plugin_dir)
{
    // 기본 플러그인 디렉토리 설정
    if (plugin_dir_.empty()) {
        plugin_dir_ = QApplication::applicationDirPath().toStdString() + "/plugins";
    }
}

PluginManager::~PluginManager()
{
    unloadPlugins();
}

int PluginManager::loadPlugins()
{
    int loaded_count = 0;
    
    QDir plugin_dir(QString::fromStdString(plugin_dir_));
    QStringList entry_list = plugin_dir.entryList(QDir::Files);
    
    for (const QString& file_name : entry_list) {
        // 플러그인 파일 (확장자에 따라 필터링)
        if (QLibrary::isLibrary(file_name)) {
            QString absolute_path = plugin_dir.absoluteFilePath(file_name);
            
            // 이미 로드된 플러그인인지 확인
            std::string file_path = absolute_path.toStdString();
            if (plugin_loaders_.find(file_path) != plugin_loaders_.end()) {
                continue; // 이미 로드됨
            }
            
            // 플러그인 로더 생성
            auto loader = std::make_unique<QPluginLoader>(absolute_path);
            
            // 플러그인 로드
            if (loader->load()) {
                QObject* plugin_instance = loader->instance();
                if (plugin_instance) {
                    // 인터페이스로 캐스팅 (동적 캐스팅 사용)
                    // 참고: 플러그인은 PluginInterface를 구현하고 QObject에서 파생된 클래스여야 함
                    PluginInterface* plugin = dynamic_cast<PluginInterface*>(plugin_instance);
                    if (plugin) {
                        // 플러그인 초기화
                        if (plugin->initialize()) {
                            std::string plugin_name = plugin->getName();
                            
                            // 플러그인 저장
                            plugins_[plugin_name] = plugin;
                            plugin_loaders_[file_path] = std::move(loader);
                            
                            // 로드 알림
                            emit pluginLoaded(plugin_name);
                            loaded_count++;
                            
                            qInfo() << "플러그인 로드 성공:" << QString::fromStdString(plugin_name);
                        } else {
                            // 초기화 실패
                            loader->unload();
                            qWarning() << "플러그인 초기화 실패:" << absolute_path;
                        }
                    } else {
                        // 캐스팅 실패
                        loader->unload();
                        qWarning() << "플러그인 인터페이스 불일치:" << absolute_path;
                    }
                } else {
                    // 인스턴스 생성 실패
                    qWarning() << "플러그인 인스턴스 생성 실패:" << absolute_path;
                    qWarning() << "오류:" << loader->errorString();
                }
            } else {
                // 로드 실패
                qWarning() << "플러그인 로드 실패:" << absolute_path;
                qWarning() << "오류:" << loader->errorString();
            }
        }
    }
    
    return loaded_count;
}

void PluginManager::unloadPlugins()
{
    // 모든 플러그인 종료
    for (auto& entry : plugins_) {
        PluginInterface* plugin = entry.second;
        if (plugin) {
            try {
                plugin->shutdown();
                emit pluginUnloaded(entry.first);
            } catch (const std::exception& e) {
                qWarning() << "플러그인 종료 중 오류 발생:" << QString::fromStdString(entry.first) << "-" << e.what();
            }
        }
    }
    
    // 맵 정리
    plugins_.clear();
    
    // 모든 플러그인 언로드
    for (auto& entry : plugin_loaders_) {
        QPluginLoader* loader = entry.second.get();
        if (loader) {
            loader->unload();
        }
    }
    
    // 로더 맵 정리
    plugin_loaders_.clear();
}

std::vector<std::string> PluginManager::getPluginNames() const
{
    std::vector<std::string> names;
    names.reserve(plugins_.size());
    
    for (const auto& entry : plugins_) {
        names.push_back(entry.first);
    }
    
    return names;
}

PluginInterface* PluginManager::getPlugin(const std::string& name) const
{
    auto it = plugins_.find(name);
    if (it != plugins_.end()) {
        return it->second;
    }
    
    return nullptr;
}

QWidget* PluginManager::createPluginWidget(const std::string& name)
{
    PluginInterface* plugin = getPlugin(name);
    if (plugin) {
        try {
            return plugin->createWidget();
        } catch (const std::exception& e) {
            qWarning() << "플러그인 위젯 생성 중 오류 발생:" << QString::fromStdString(name) << "-" << e.what();
        }
    }
    
    return nullptr;
}

} // namespace robot_debugger_ui 