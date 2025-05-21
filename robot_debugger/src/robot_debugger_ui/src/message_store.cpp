#include "robot_debugger_ui/message_store.hpp"
#include <QDateTime>
#include <QSqlQuery>
#include <QSqlError>
#include <QVariantMap>
#include <QDebug>
#include <chrono>
#include <algorithm>

namespace robot_debugger_ui {

QVariant MessageStore::StoredMessage::toVariant() const {
    QVariantMap map;
    map["topic_name"] = QString::fromStdString(topic_name);
    map["type_name"] = QString::fromStdString(type_name);
    
    // 데이터를 QByteArray로 변환
    QByteArray byte_array(reinterpret_cast<const char*>(data.data()), static_cast<int>(data.size()));
    map["data"] = byte_array;
    
    // 타임스탬프를 QDateTime으로 변환
    auto time_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(timestamp);
    auto epoch = time_ms.time_since_epoch();
    qint64 msecs = epoch.count();
    map["timestamp"] = QDateTime::fromMSecsSinceEpoch(msecs);
    
    map["log_level"] = log_level;
    map["source"] = QString::fromStdString(source);
    
    return map;
}

MessageStore::StoredMessage MessageStore::StoredMessage::fromVariant(const QVariant& variant) {
    QVariantMap map = variant.toMap();
    StoredMessage msg;
    
    msg.topic_name = map["topic_name"].toString().toStdString();
    msg.type_name = map["type_name"].toString().toStdString();
    
    // QByteArray를 std::vector<uint8_t>로 변환
    QByteArray byte_array = map["data"].toByteArray();
    const char* data_ptr = byte_array.constData();
    msg.data.assign(data_ptr, data_ptr + byte_array.size());
    
    // QDateTime을 std::chrono::system_clock::time_point로 변환
    QDateTime datetime = map["timestamp"].toDateTime();
    qint64 msecs = datetime.toMSecsSinceEpoch();
    msg.timestamp = std::chrono::system_clock::time_point(
        std::chrono::milliseconds(msecs));
    
    msg.log_level = map["log_level"].toInt();
    msg.source = map["source"].toString().toStdString();
    
    return msg;
}

MessageStore::MessageStore(size_t max_messages_per_topic, const std::string& db_path, QObject* parent)
    : QObject(parent)
    , max_messages_per_topic_(max_messages_per_topic)
    , db_path_(db_path)
    , use_database_(!db_path.empty())
{
    // 데이터베이스 초기화
    if (use_database_) {
        if (!initializeDatabase()) {
            qCritical() << "Failed to initialize message store database";
            use_database_ = false;
        }
    }
}

MessageStore::~MessageStore() {
    // 데이터베이스 연결 종료
    if (db_.isOpen()) {
        db_.close();
    }
}

bool MessageStore::initializeDatabase() {
    // 데이터베이스 연결 설정
    db_ = QSqlDatabase::addDatabase("QSQLITE", "MessageStoreDB");
    
    if (db_path_ == ":memory:") {
        // 메모리 내 데이터베이스 사용
        db_.setDatabaseName(":memory:");
    } else {
        // 파일 기반 데이터베이스 사용
        db_.setDatabaseName(QString::fromStdString(db_path_));
    }
    
    // 데이터베이스 연결
    if (!db_.open()) {
        qCritical() << "Failed to open database:" << db_.lastError().text();
        return false;
    }
    
    // 이벤트 테이블 생성
    QSqlQuery query(db_);
    bool success = query.exec(
        "CREATE TABLE IF NOT EXISTS events ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "topic_name TEXT,"
        "type_name TEXT,"
        "data BLOB,"
        "timestamp INTEGER,"
        "log_level INTEGER,"
        "source TEXT"
        ")");
    
    if (!success) {
        qCritical() << "Failed to create events table:" << query.lastError().text();
        return false;
    }
    
    // 타임스탬프 인덱스 생성
    success = query.exec(
        "CREATE INDEX IF NOT EXISTS idx_events_timestamp ON events (timestamp)");
    
    if (!success) {
        qCritical() << "Failed to create timestamp index:" << query.lastError().text();
        return false;
    }
    
    // 로그 레벨 인덱스 생성
    success = query.exec(
        "CREATE INDEX IF NOT EXISTS idx_events_log_level ON events (log_level)");
    
    if (!success) {
        qCritical() << "Failed to create log level index:" << query.lastError().text();
        return false;
    }
    
    return true;
}

void MessageStore::storeMessage(const std::string& topic_name, 
                               const std::vector<uint8_t>& data,
                               const std::string& type_name,
                               int log_level,
                               const std::string& source) {
    // 메시지 구조체 생성
    StoredMessage msg;
    msg.topic_name = topic_name;
    msg.data = data;
    msg.type_name = type_name;
    msg.timestamp = std::chrono::system_clock::now();
    msg.log_level = log_level;
    msg.source = source;
    
    // 메시지 저장소에 추가
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto& topic_queue = message_store_[topic_name];
        
        // 버퍼가 가득 찼으면 가장 오래된 메시지 제거
        if (topic_queue.size() >= max_messages_per_topic_) {
            topic_queue.pop_front();
        }
        
        // 새 메시지 추가
        topic_queue.push_back(msg);
    }
    
    // 중요 이벤트면 데이터베이스에 저장
    if (log_level >= 2 && use_database_) {  // WARN 이상 레벨
        storeEventInDatabase(msg);
        
        // 중요 이벤트 시그널 발생
        emit significantEvent(msg);
    }
    
    // 메시지 저장 시그널 발생
    emit messageStored(msg);
}

void MessageStore::storeEventInDatabase(const StoredMessage& message) {
    if (!db_.isOpen()) return;
    
    QSqlQuery query(db_);
    query.prepare(
        "INSERT INTO events (topic_name, type_name, data, timestamp, log_level, source) "
        "VALUES (:topic_name, :type_name, :data, :timestamp, :log_level, :source)");
    
    query.bindValue(":topic_name", QString::fromStdString(message.topic_name));
    query.bindValue(":type_name", QString::fromStdString(message.type_name));
    
    // 데이터를 QByteArray로 변환
    QByteArray data_blob(reinterpret_cast<const char*>(message.data.data()), 
                        static_cast<int>(message.data.size()));
    query.bindValue(":data", data_blob);
    
    // 타임스탬프를 에포크 밀리초로 변환
    auto time_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(
        message.timestamp).time_since_epoch();
    qint64 timestamp_ms = time_ms.count();
    query.bindValue(":timestamp", timestamp_ms);
    
    query.bindValue(":log_level", message.log_level);
    query.bindValue(":source", QString::fromStdString(message.source));
    
    if (!query.exec()) {
        qWarning() << "Failed to store event in database:" << query.lastError().text();
    }
}

MessageStore::StoredMessage MessageStore::getLatestMessage(const std::string& topic_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = message_store_.find(topic_name);
    if (it != message_store_.end() && !it->second.empty()) {
        return it->second.back();  // 가장 최근 메시지 반환
    }
    
    // 토픽이 없거나 메시지가 없는 경우 빈 메시지 반환
    return StoredMessage{};
}

std::vector<MessageStore::StoredMessage> MessageStore::getMessageHistory(
    const std::string& topic_name, size_t count) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<StoredMessage> result;
    
    auto it = message_store_.find(topic_name);
    if (it != message_store_.end()) {
        const auto& queue = it->second;
        
        // 요청한 개수가 0이거나 전체 메시지 수보다 많으면 전체 반환
        size_t n = (count == 0 || count > queue.size()) ? queue.size() : count;
        
        // 최신 메시지부터 가져오기
        auto start_it = queue.end() - n;
        result.assign(start_it, queue.end());
    }
    
    return result;
}

std::vector<MessageStore::StoredMessage> MessageStore::getMessagesByTimeRange(
    const std::string& topic_name,
    const std::chrono::system_clock::time_point& start_time,
    const std::chrono::system_clock::time_point& end_time) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<StoredMessage> result;
    
    auto it = message_store_.find(topic_name);
    if (it != message_store_.end()) {
        const auto& queue = it->second;
        
        // 시간 범위에 맞는 메시지만 필터링
        for (const auto& msg : queue) {
            if (msg.timestamp >= start_time && msg.timestamp <= end_time) {
                result.push_back(msg);
            }
        }
    }
    
    return result;
}

std::vector<MessageStore::StoredMessage> MessageStore::getMessagesByLogLevel(
    int min_level, size_t count) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<StoredMessage> result;
    
    // 모든 토픽의 메시지를 검사
    for (const auto& [topic, queue] : message_store_) {
        for (const auto& msg : queue) {
            if (msg.log_level >= min_level) {
                result.push_back(msg);
            }
        }
    }
    
    // 타임스탬프로 정렬
    std::sort(result.begin(), result.end(), 
              [](const StoredMessage& a, const StoredMessage& b) {
                  return a.timestamp < b.timestamp;
              });
    
    // 요청한 개수만큼 반환
    if (count > 0 && count < result.size()) {
        // 최신 메시지만 반환
        result.erase(result.begin(), result.end() - count);
    }
    
    return result;
}

std::vector<std::string> MessageStore::getStoredTopics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<std::string> topics;
    topics.reserve(message_store_.size());
    
    for (const auto& entry : message_store_) {
        topics.push_back(entry.first);
    }
    
    return topics;
}

std::map<std::string, size_t> MessageStore::getMessageCounts() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::map<std::string, size_t> counts;
    
    for (const auto& [topic, queue] : message_store_) {
        counts[topic] = queue.size();
    }
    
    return counts;
}

void MessageStore::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    message_store_.clear();
}

void MessageStore::clearTopic(const std::string& topic_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = message_store_.find(topic_name);
    if (it != message_store_.end()) {
        it->second.clear();
    }
}

void MessageStore::setMaxMessagesPerTopic(size_t max_messages) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    max_messages_per_topic_ = max_messages;
    
    // 각 토픽의 버퍼 크기 조정
    for (auto& [topic, queue] : message_store_) {
        while (queue.size() > max_messages_per_topic_) {
            queue.pop_front();  // 가장 오래된 메시지부터 제거
        }
    }
}

std::vector<MessageStore::StoredMessage> MessageStore::loadEventsFromDatabase(
    const std::chrono::system_clock::time_point& start_time,
    const std::chrono::system_clock::time_point& end_time,
    int min_level) const {
    std::vector<StoredMessage> events;
    
    if (!use_database_ || !db_.isOpen()) {
        return events;
    }
    
    // 시작 및 종료 시간을 밀리초로 변환
    auto start_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(
        start_time).time_since_epoch().count();
    auto end_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(
        end_time).time_since_epoch().count();
    
    QSqlQuery query(db_);
    query.prepare(
        "SELECT topic_name, type_name, data, timestamp, log_level, source "
        "FROM events "
        "WHERE timestamp >= :start_time AND timestamp <= :end_time "
        "AND log_level >= :min_level "
        "ORDER BY timestamp DESC");
    
    query.bindValue(":start_time", static_cast<qint64>(start_ms));
    query.bindValue(":end_time", static_cast<qint64>(end_ms));
    query.bindValue(":min_level", min_level);
    
    if (!query.exec()) {
        qWarning() << "Failed to load events from database:" << query.lastError().text();
        return events;
    }
    
    while (query.next()) {
        StoredMessage event;
        event.topic_name = query.value(0).toString().toStdString();
        event.type_name = query.value(1).toString().toStdString();
        
        // BLOB 데이터 변환
        QByteArray data_blob = query.value(2).toByteArray();
        const char* data_ptr = data_blob.constData();
        event.data.assign(data_ptr, data_ptr + data_blob.size());
        
        // 타임스탬프 변환
        qint64 timestamp_ms = query.value(3).toLongLong();
        event.timestamp = std::chrono::system_clock::time_point(
            std::chrono::milliseconds(timestamp_ms));
        
        event.log_level = query.value(4).toInt();
        event.source = query.value(5).toString().toStdString();
        
        events.push_back(event);
    }
    
    return events;
}

} // namespace robot_debugger_ui 