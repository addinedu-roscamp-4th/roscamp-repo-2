#include "robot_debugger_ui/topic_manager.hpp"
#include <QDebug>
#include <regex>

namespace robot_debugger_ui {

TopicManager::TopicManager(std::shared_ptr<rclcpp::Node> node, QObject* parent)
    : QObject(parent)
    , node_(node)
{
    // 스캔 타이머 설정 (1초마다 토픽 스캔)
    connect(&scan_timer_, &QTimer::timeout, this, &TopicManager::onScanTimer);
    scan_timer_.setInterval(1000); // 1초 간격
}

TopicManager::~TopicManager()
{
    stopScan();
}

void TopicManager::startScan()
{
    if (!scan_timer_.isActive()) {
        scan_timer_.start();
    }
}

void TopicManager::stopScan()
{
    if (scan_timer_.isActive()) {
        scan_timer_.stop();
    }
    
    // 모든 구독 정리
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    subscriptions_.clear();
}

void TopicManager::setDomainPatterns(const std::vector<int>& domains)
{
    domain_patterns_ = domains;
}

void TopicManager::setNamespacePatterns(const std::vector<std::string>& namespaces)
{
    namespace_patterns_ = namespaces;
}

void TopicManager::onScanTimer()
{
    if (!node_) {
        qWarning() << "토픽 관리자: ROS 노드가 초기화되지 않았습니다.";
        return;
    }
    
    try {
        // 사용 가능한 모든 토픽 조회
        auto topic_names_and_types = node_->get_topic_names_and_types();
        
        // 구독할 토픽 목록
        std::vector<std::string> matched_topics;
        
        // 패턴에 맞는 토픽 필터링
        for (const auto& topic_info : topic_names_and_types) {
            const auto& topic_name = topic_info.first;
            const auto& type_names = topic_info.second;
            
            if (!type_names.empty() && matchesPattern(topic_name)) {
                matched_topics.push_back(topic_name);
                
                // 아직 구독하지 않은 토픽이면 구독
                std::lock_guard<std::mutex> lock(subscriptions_mutex_);
                if (subscriptions_.find(topic_name) == subscriptions_.end()) {
                    subscribeToTopic(topic_name, type_names[0]);
                }
            }
        }
        
        // 토픽 목록 변경 알림
        emit topicsChanged(matched_topics);
        
    } catch (const std::exception& e) {
        qWarning() << "토픽 스캔 중 오류 발생:" << e.what();
    }
}

bool TopicManager::matchesPattern(const std::string& topic_name)
{
    // 네임스페이스 패턴 확인
    if (!namespace_patterns_.empty()) {
        bool matches_namespace = false;
        for (const auto& ns_pattern : namespace_patterns_) {
            // 와일드카드(*) 지원
            std::string regex_pattern = ns_pattern;
            // 와일드카드를 정규식으로 변환
            std::regex_replace(regex_pattern, std::regex("\\*"), ".*");
            std::regex ns_regex(regex_pattern);
            
            if (std::regex_search(topic_name, ns_regex)) {
                matches_namespace = true;
                break;
            }
        }
        
        if (!matches_namespace) {
            return false;
        }
    }
    
    // 현재는 도메인 패턴을 직접적으로 확인할 수 없음
    // ROS 2에서 도메인 ID는 토픽 이름에 포함되지 않음
    
    return true;
}

void TopicManager::subscribeToTopic(const std::string& topic_name, const std::string& type_name)
{
    if (!node_) {
        return;
    }
    
    qInfo() << "토픽 구독:" << topic_name.c_str() << "(" << type_name.c_str() << ")";
    
    // 제네릭 구독자 생성
    try {
        auto subscription_callback = [this, topic_name](const std::shared_ptr<rclcpp::SerializedMessage> msg) {
            // 메시지 데이터를 std::vector<uint8_t>로 변환
            const auto& rcl_serialized_msg = msg->get_rcl_serialized_message();
            std::vector<uint8_t> data(rcl_serialized_msg.buffer, rcl_serialized_msg.buffer + rcl_serialized_msg.buffer_length);
            
            // 메시지 수신 시그널 방출
            emit messageReceived(topic_name, data);
        };

        rclcpp::QoS qos(10); // 기본적으로 히스토리 크기 10 사용
        auto options = rclcpp::SubscriptionOptions();
        
        auto subscription = node_->create_generic_subscription(
            topic_name,
            type_name,
            qos,
            subscription_callback,
            options
        );
        
        // 구독 맵에 저장
        subscriptions_[topic_name] = subscription;
        
    } catch (const std::exception& e) {
        qWarning() << "토픽 구독 중 오류 발생:" << topic_name.c_str() << ":" << e.what();
    }
}

} // namespace robot_debugger_ui 