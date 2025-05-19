#include "robot_debugger_ui/message_store.hpp"
#include <QDateTime>
#include <algorithm>

namespace robot_debugger_ui {

MessageStore::MessageStore(size_t max_history_per_topic, QObject* parent)
    : QObject(parent)
    , default_capacity_(max_history_per_topic)
{
}

MessageStore::~MessageStore() = default;

MessageStore::TimedMessage MessageStore::getLatestMessage(const std::string& topic_name)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    auto it = message_buffers_.find(topic_name);
    if (it != message_buffers_.end() && !it->second.empty()) {
        return it->second.front(); // 가장 최근 메시지는 deque의 front에 있음
    }
    
    // 토픽이 없거나 비어있으면 빈 메시지 반환
    return TimedMessage{0.0, std::vector<uint8_t>()};
}

std::vector<MessageStore::TimedMessage> MessageStore::getMessageHistory(
    const std::string& topic_name, size_t count)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    std::vector<TimedMessage> result;
    
    auto it = message_buffers_.find(topic_name);
    if (it != message_buffers_.end()) {
        const auto& buffer = it->second;
        
        // count가 0이면 모든 메시지 반환, 그렇지 않으면 지정된 개수 반환
        size_t num_messages = (count == 0 || count > buffer.size()) ? buffer.size() : count;
        
        // deque에서 가장 최근 메시지가 front에 있으므로, 그 순서대로 복사
        result.reserve(num_messages);
        std::copy_n(buffer.begin(), num_messages, std::back_inserter(result));
    }
    
    return result;
}

std::vector<std::string> MessageStore::getTopicNames()
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    std::vector<std::string> result;
    result.reserve(message_buffers_.size());
    
    for (const auto& entry : message_buffers_) {
        result.push_back(entry.first);
    }
    
    return result;
}

void MessageStore::setTopicCapacity(const std::string& topic_name, size_t capacity)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    topic_capacities_[topic_name] = capacity;
    
    // 이미 버퍼가 있고, 새 용량보다 크기가 크면 줄이기
    auto buffer_it = message_buffers_.find(topic_name);
    if (buffer_it != message_buffers_.end() && buffer_it->second.size() > capacity) {
        // 뒤에서부터 삭제 (오래된 메시지부터 삭제)
        while (buffer_it->second.size() > capacity) {
            buffer_it->second.pop_back();
        }
    }
}

void MessageStore::storeMessage(const std::string& topic_name, const std::vector<uint8_t>& data)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // 현재 시간 구하기 (UTC 기준)
    double timestamp = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    
    // 새 메시지 생성
    TimedMessage new_message{timestamp, data};
    
    // 해당 토픽의 버퍼 찾기 또는 생성
    auto& buffer = message_buffers_[topic_name];
    
    // 버퍼 앞에 새 메시지 추가 (최근 메시지가 맨 앞에 오도록)
    buffer.push_front(new_message);
    
    // 용량 제한 확인
    size_t capacity = default_capacity_;
    auto capacity_it = topic_capacities_.find(topic_name);
    if (capacity_it != topic_capacities_.end()) {
        capacity = capacity_it->second;
    }
    
    // 버퍼 크기가 용량을 초과하면 가장 오래된 메시지 제거
    while (buffer.size() > capacity) {
        buffer.pop_back();
    }
    
    // 메시지 저장 알림
    emit messageStored(topic_name);
}

} // namespace robot_debugger_ui 