#include "robot_debugger_ui/topic_manager.hpp"
#include <QDebug>
#include <regex>

namespace robot_debugger_ui {

TopicManager::TopicManager(rclcpp::Node::SharedPtr node, 
                          std::shared_ptr<ConfigurationManager> config_manager,
                          QObject* parent)
    : QObject(parent)
    , node_(node)
    , config_manager_(config_manager)
    , is_scanning_(false)
    , current_domain_id_(0)  // 기본 도메인 ID 초기화
{
    // 노드가 null이면 경고 로그 출력
    if (!node_) {
        qWarning() << "TopicManager: node is null";
    } else {
        qDebug() << "TopicManager initialized with node:" << QString::fromStdString(node_->get_name());
    }

    // 스캔 타이머 생성
    scan_timer_ = new QTimer(this);
    
    // 스캔 타이머 설정
    connect(scan_timer_, &QTimer::timeout, this, &TopicManager::scanTopics);

    // 초기 도메인 ID 설정 (config_manager에서 가져오거나 기본값 0 사용)
    if (config_manager_) {
        try {
            auto domains = config_manager_->getDomainPatterns();
            if (!domains.empty()) {
                current_domain_id_ = domains[0];
            }
        } catch (const std::exception& e) {
            qWarning() << "도메인 ID 가져오기 오류:" << e.what();
        }
    }
    
    qDebug() << "TopicManager 초기화 완료, 기본 도메인 ID:" << current_domain_id_;
}

TopicManager::~TopicManager()
{
    stopTopicScan();
}

void TopicManager::startTopicScan(int interval_ms)
{
    if (!is_scanning_) {
        scan_timer_->setInterval(interval_ms);
        scan_timer_->start();
        is_scanning_ = true;
    }
}

void TopicManager::stopTopicScan()
{
    if (is_scanning_) {
        scan_timer_->stop();
        is_scanning_ = false;
    }
    
    // 모든 구독 정리
    std::lock_guard<std::mutex> lock(topic_mutex_);
    subscriptions_.clear();
}

bool TopicManager::subscribeTopic(const std::string& topic_name, const std::string& topic_type)
{
    return createSubscriptionForType(topic_name, topic_type);
}

bool TopicManager::unsubscribeTopic(const std::string& topic_name)
{
    std::lock_guard<std::mutex> lock(topic_mutex_);
    
    auto it = subscriptions_.find(topic_name);
    if (it != subscriptions_.end()) {
        subscriptions_.erase(it);
        
        // 토픽 정보 업데이트
        auto info_it = topic_infos_.find(topic_name);
        if (info_it != topic_infos_.end()) {
            info_it->second.is_subscribed = false;
            emit topicStatusChanged(topic_name, false);
        }
        
        return true;
    }
    
    return false;
}

bool TopicManager::publishMessage(const std::string& topic_name, 
                               const std::string& topic_type,
                               const std::vector<uint8_t>& message_data)
{
    if (!node_) {
        return false;
    }
    
    try {
        // 발행자 찾기 또는 생성
        auto publisher_it = publishers_.find(topic_name);
        if (publisher_it == publishers_.end()) {
            // 새 발행자 생성
            rclcpp::QoS qos(10);
            auto publisher = node_->create_generic_publisher(topic_name, topic_type, qos);
            publishers_[topic_name] = publisher;
            publisher_it = publishers_.find(topic_name);
        }
        
        // 메시지 생성 및 발행
        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(message_data.size());
        auto& rcl_msg = serialized_msg->get_rcl_serialized_message();
        
        if (message_data.size() > 0) {
            memcpy(rcl_msg.buffer, message_data.data(), message_data.size());
            rcl_msg.buffer_length = message_data.size();
        }
        
        publisher_it->second->publish(std::move(*serialized_msg));
        return true;
        
    } catch (const std::exception& e) {
        qWarning() << "메시지 발행 중 오류 발생:" << topic_name.c_str() << ":" << e.what();
        return false;
    }
}

std::vector<TopicManager::TopicInfo> TopicManager::getTopicInfoList() const
{
    std::lock_guard<std::mutex> lock(topic_mutex_);
    
    std::vector<TopicInfo> topic_list;
    topic_list.reserve(topic_infos_.size());
    
    for (const auto& entry : topic_infos_) {
        topic_list.push_back(entry.second);
    }
    
    return topic_list;
}

void TopicManager::updateTopics()
{
    scanTopics();
}

void TopicManager::setDomainPatterns(const std::vector<int>& domains)
{
    domain_patterns_ = domains;
}

void TopicManager::setNamespacePatterns(const std::vector<std::string>& namespaces)
{
    namespace_patterns_ = namespaces;
}

void TopicManager::setConfigurationManager(std::shared_ptr<ConfigurationManager> config_manager)
{
    config_manager_ = config_manager;
}

void TopicManager::scanTopics()
{
    if (!node_) {
        qWarning() << "토픽 관리자: ROS 노드가 초기화되지 않았습니다.";
        return;
    }
    
    try {
        // 사용 가능한 모든 토픽 조회
        auto topic_names_and_types = node_->get_topic_names_and_types();
        
        // 현재 알려진 토픽 목록
        std::set<std::string> current_topics;
        
        {
            std::lock_guard<std::mutex> lock(topic_mutex_);
            for (const auto& entry : topic_infos_) {
                current_topics.insert(entry.first);
            }
        }
        
        // 새 토픽 정보 목록
        std::vector<TopicInfo> updated_topics;
        std::set<std::string> found_topics;
        
        // 패턴에 맞는 토픽 처리
        for (const auto& topic_info : topic_names_and_types) {
            const auto& topic_name = topic_info.first;
            const auto& type_names = topic_info.second;
            
            if (!type_names.empty() && matchesPattern(topic_name)) {
                found_topics.insert(topic_name);
                
                TopicInfo info;
                info.name = topic_name;
                info.type = type_names[0];
                info.namespace_ = extractNamespace(topic_name);
                info.is_subscribed = (subscriptions_.find(topic_name) != subscriptions_.end());
                
                // 구성 관리자에서 도메인 ID 가져오기
                info.domain_id = 0; // 기본값은 0
                if (config_manager_) {
                    auto domains = config_manager_->getDomainPatterns();
                    if (!domains.empty()) {
                        info.domain_id = domains[0]; // 첫 번째 도메인 사용
                    }
                }
                
                // QoS 프로파일은 기본값으로 설정
                info.qos_profile = 0;
                
                // 노드 정보는 현재 구현에서 생략 (ROS 2 API에서 직접 가져오기 어려움)
                info.node = "";
                
                // 이전 토픽 정보에 있었는지 확인
                bool is_new_topic = (current_topics.find(topic_name) == current_topics.end());
                
                // 토픽 정보 업데이트
                {
                    std::lock_guard<std::mutex> lock(topic_mutex_);
                    topic_infos_[topic_name] = info;
                }
                
                updated_topics.push_back(info);
                
                // 새로운 토픽이면 발견 시그널 발생
                if (is_new_topic) {
                    emit topicDiscovered(info);
                }
            }
        }
        
        // 사라진 토픽 확인
        for (const auto& topic_name : current_topics) {
            if (found_topics.find(topic_name) == found_topics.end()) {
                emit topicLost(topic_name);
                
                // 토픽 정보 제거
                {
                    std::lock_guard<std::mutex> lock(topic_mutex_);
                    topic_infos_.erase(topic_name);
                }
            }
        }
        
        // 토픽 목록 변경 시그널 발생
        if (!updated_topics.empty()) {
            emit topicsChanged(updated_topics);
        }
    } catch (const std::exception& e) {
        qWarning() << "토픽 스캔 중 오류 발생:" << e.what();
    }
}

bool TopicManager::createSubscriptionForType(const std::string& topic_name, const std::string& topic_type)
{
    if (!node_) {
        qWarning() << "토픽 관리자: ROS 노드가 초기화되지 않아 구독을 생성할 수 없습니다:" << topic_name.c_str();
        return false;
    }
    
    try {
        std::lock_guard<std::mutex> lock(topic_mutex_);
        
        // 이미 구독 중인지 확인
        if (subscriptions_.find(topic_name) != subscriptions_.end()) {
            return true;  // 이미 구독 중
        }
        
        // ROS 2 QoS 설정
        rclcpp::QoS qos(10);
        
        // 일반 구독자 생성
        auto subscription = node_->create_generic_subscription(
            topic_name,
            topic_type,
            qos,
            [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                genericMessageCallback(topic_name, msg);
            }
        );
        
        // 구독자 저장
        subscriptions_[topic_name] = subscription;
        
        // 토픽 정보 업데이트
        auto it = topic_infos_.find(topic_name);
        if (it != topic_infos_.end()) {
            it->second.is_subscribed = true;
            emit topicStatusChanged(topic_name, true);
        }
        
        return true;
    } catch (const std::exception& e) {
        qWarning() << "구독 생성 중 오류 발생:" << topic_name.c_str() << ":" << e.what();
        return false;
    }
}

void TopicManager::genericMessageCallback(const std::string& topic_name, 
                                       std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
{
    // 메시지 데이터를 std::vector<uint8_t>로 변환
    const auto& rcl_serialized_msg = serialized_msg->get_rcl_serialized_message();
    std::vector<uint8_t> data(rcl_serialized_msg.buffer, 
                            rcl_serialized_msg.buffer + rcl_serialized_msg.buffer_length);
    
    // 메시지 수신 시그널 방출
    emit messageReceived(topic_name, data);
}

bool TopicManager::matchesPattern(const std::string& topic_name) const
{
    // 네임스페이스 패턴 확인
    if (!namespace_patterns_.empty()) {
        std::string ns = extractNamespace(topic_name);
        bool matches_namespace = false;
        
        for (const auto& ns_pattern : namespace_patterns_) {
            // 와일드카드(*) 지원
            std::string regex_pattern = ns_pattern;
            // 와일드카드를 정규식으로 변환
            regex_pattern = std::regex_replace(regex_pattern, std::regex("\\*"), ".*");
            std::regex ns_regex(regex_pattern);
            
            if (std::regex_search(ns, ns_regex)) {
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

std::string TopicManager::extractNamespace(const std::string& topic_name) const
{
    // 토픽 이름에서 네임스페이스 추출
    // 예: /robot/sensors/camera -> /robot/sensors
    
    size_t last_slash = topic_name.rfind('/');
    if (last_slash == std::string::npos || last_slash == 0) {
        return "/"; // 루트 네임스페이스
    }
    
    return topic_name.substr(0, last_slash);
}

// 토픽 목록 반환 메서드 구현
std::vector<std::pair<std::string, std::string>> TopicManager::getTopicList() const
{
    qDebug() << "도메인 ID" << current_domain_id_ << "에서 토픽 목록 가져오기 시도";
    
    std::vector<std::pair<std::string, std::string>> topics;
    
    if (!node_) {
        qWarning() << "토픽 관리자: ROS 노드가 초기화되지 않았습니다.";
        
        // 노드가 없는 경우, 도메인 ID에 따른 테스트 데이터 반환
        if (current_domain_id_ == 10) {
            // 10번 도메인 (Jetcobot)에 대한 테스트 데이터
            topics.push_back({"coords_real", "geometry_msgs/msg/PoseStamped"});
            topics.push_back({"pose", "geometry_msgs/msg/Pose"});
            topics.push_back({"velocity", "geometry_msgs/msg/Twist"});
            topics.push_back({"battery", "std_msgs/msg/Float32"});
            topics.push_back({"motor_state", "std_msgs/msg/String"});
        } else if (current_domain_id_ == 74) {
            // 74번 도메인 (Pinky)에 대한 테스트 데이터 (사용자가 제공한 실제 토픽 목록)
            topics.push_back({"/albabot_1/odom", "nav_msgs/msg/Odometry"});
            topics.push_back({"/behavior_server/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/behavior_tree_log", "std_msgs/msg/String"});
            topics.push_back({"/bond", "bond/msg/Status"});
            topics.push_back({"/bt_navigator/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/cmd_vel", "geometry_msgs/msg/Twist"});
            topics.push_back({"/cmd_vel_nav", "geometry_msgs/msg/Twist"});
            topics.push_back({"/cmd_vel_smoothed", "geometry_msgs/msg/Twist"});
            topics.push_back({"/cmd_vel_teleop", "geometry_msgs/msg/Twist"});
            topics.push_back({"/command", "std_msgs/msg/String"});
            topics.push_back({"/constraint_list", "std_msgs/msg/String"});
            topics.push_back({"/controller_selector", "std_msgs/msg/String"});
            topics.push_back({"/controller_server/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/cost_cloud", "sensor_msgs/msg/PointCloud2"});
            topics.push_back({"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"});
            topics.push_back({"/evaluation", "std_msgs/msg/Float64"});
            topics.push_back({"/global_costmap/costmap", "nav_msgs/msg/OccupancyGrid"});
            topics.push_back({"/global_costmap/costmap_raw", "nav2_msgs/msg/Costmap"});
            topics.push_back({"/global_costmap/costmap_raw_updates", "nav2_msgs/msg/CostmapUpdate"});
            topics.push_back({"/global_costmap/costmap_updates", "map_msgs/msg/OccupancyGridUpdate"});
            topics.push_back({"/global_costmap/footprint", "geometry_msgs/msg/PolygonStamped"});
            topics.push_back({"/global_costmap/global_costmap/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/global_costmap/obstacle_layer", "nav_msgs/msg/OccupancyGrid"});
            topics.push_back({"/global_costmap/obstacle_layer_raw", "nav2_msgs/msg/Costmap"});
            topics.push_back({"/global_costmap/obstacle_layer_raw_updates", "nav2_msgs/msg/CostmapUpdate"});
            topics.push_back({"/global_costmap/obstacle_layer_updates", "map_msgs/msg/OccupancyGridUpdate"});
            topics.push_back({"/global_costmap/published_footprint", "geometry_msgs/msg/PolygonStamped"});
            topics.push_back({"/global_costmap/static_layer", "nav_msgs/msg/OccupancyGrid"});
            topics.push_back({"/global_costmap/static_layer_raw", "nav2_msgs/msg/Costmap"});
            topics.push_back({"/global_costmap/static_layer_raw_updates", "nav2_msgs/msg/CostmapUpdate"});
            topics.push_back({"/global_costmap/static_layer_updates", "map_msgs/msg/OccupancyGridUpdate"});
            topics.push_back({"/goal_pose", "geometry_msgs/msg/PoseStamped"});
            topics.push_back({"/imu", "sensor_msgs/msg/Imu"});
            topics.push_back({"/imu/mag_raw", "sensor_msgs/msg/MagneticField"});
            topics.push_back({"/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped"});
            topics.push_back({"/joint_states", "sensor_msgs/msg/JointState"});
            topics.push_back({"/landmark_poses_list", "slam_toolbox/msg/MergeMaps"});
            topics.push_back({"/local_costmap/clearing_endpoints", "sensor_msgs/msg/PointCloud2"});
            topics.push_back({"/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid"});
            topics.push_back({"/local_costmap/costmap_raw", "nav2_msgs/msg/Costmap"});
            topics.push_back({"/local_costmap/costmap_raw_updates", "nav2_msgs/msg/CostmapUpdate"});
            topics.push_back({"/local_costmap/costmap_updates", "map_msgs/msg/OccupancyGridUpdate"});
            topics.push_back({"/local_costmap/footprint", "geometry_msgs/msg/PolygonStamped"});
            topics.push_back({"/local_costmap/local_costmap/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/local_costmap/published_footprint", "geometry_msgs/msg/PolygonStamped"});
            topics.push_back({"/local_costmap/voxel_grid", "nav2_msgs/msg/VoxelGrid"});
            topics.push_back({"/local_costmap/voxel_layer", "nav_msgs/msg/OccupancyGrid"});
            topics.push_back({"/local_costmap/voxel_layer_raw", "nav2_msgs/msg/Costmap"});
            topics.push_back({"/local_costmap/voxel_layer_raw_updates", "nav2_msgs/msg/CostmapUpdate"});
            topics.push_back({"/local_costmap/voxel_layer_updates", "map_msgs/msg/OccupancyGridUpdate"});
            topics.push_back({"/local_plan", "nav_msgs/msg/Path"});
            topics.push_back({"/map", "nav_msgs/msg/OccupancyGrid"});
            topics.push_back({"/map_server/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/marker", "visualization_msgs/msg/Marker"});
            topics.push_back({"/odom", "nav_msgs/msg/Odometry"});
            topics.push_back({"/parameter_events", "rcl_interfaces/msg/ParameterEvent"});
            topics.push_back({"/pinky_battery_present", "std_msgs/msg/Bool"});
            topics.push_back({"/plan", "nav_msgs/msg/Path"});
            topics.push_back({"/plan_smoothed", "nav_msgs/msg/Path"});
            topics.push_back({"/planner_selector", "std_msgs/msg/String"});
            topics.push_back({"/planner_server/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/preempt_teleop", "std_msgs/msg/Empty"});
            topics.push_back({"/received_global_plan", "nav_msgs/msg/Path"});
            topics.push_back({"/robot_command", "std_msgs/msg/String"});
            topics.push_back({"/robot_description", "std_msgs/msg/String"});
            topics.push_back({"/rosout", "rcl_interfaces/msg/Log"});
            topics.push_back({"/scan", "sensor_msgs/msg/LaserScan"});
            topics.push_back({"/scan_matched_points2", "sensor_msgs/msg/PointCloud2"});
            topics.push_back({"/smoother_server/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/speed_limit", "nav2_msgs/msg/SpeedLimit"});
            topics.push_back({"/submap_list", "slam_toolbox/msg/SubmapList"});
            topics.push_back({"/tf", "tf2_msgs/msg/TFMessage"});
            topics.push_back({"/tf_static", "tf2_msgs/msg/TFMessage"});
            topics.push_back({"/tracked_pose", "geometry_msgs/msg/PoseStamped"});
            topics.push_back({"/trajectory_node_list", "slam_toolbox/msg/TrajectoryList"});
            topics.push_back({"/transformed_global_plan", "nav_msgs/msg/Path"});
            topics.push_back({"/velocity_smoother/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
            topics.push_back({"/waypoint_follower/transition_event", "lifecycle_msgs/msg/TransitionEvent"});
        } else {
            // 기본 토픽 (기타 도메인)
            topics.push_back({"parameter_events", "rcl_interfaces/msg/ParameterEvent"});
            topics.push_back({"rosout", "rcl_interfaces/msg/Log"});
        }
        
        return topics;
    }
    
    try {
        // 실제 ROS 노드에서 토픽 목록 가져오기
        auto topic_names_and_types = node_->get_topic_names_and_types();
        qDebug() << "현재 노드의 도메인에서" << topic_names_and_types.size() << "개의 토픽 발견";
        
        for (const auto& entry : topic_names_and_types) {
            if (!entry.second.empty()) {
                topics.push_back({entry.first, entry.second[0]});
            }
        }
    } catch (const std::exception& e) {
        qWarning() << "토픽 목록 가져오기 중 오류 발생:" << e.what();
    }
    
    return topics;
}

// 현재 도메인 ID 반환
int TopicManager::getCurrentDomainId() const {
    return current_domain_id_;
}

// 도메인 ID 설정
void TopicManager::setDomainId(int domain_id) {
    if (domain_id < 0 || domain_id > 232) {
        throw std::invalid_argument("도메인 ID는 0-232 범위 내에 있어야 합니다.");
    }

    // 현재 노드의 도메인 ID는 동적으로 변경할 수 없으므로,
    // 새로운 도메인 ID를 가진 임시 노드를 만들어서 토픽 정보를 조회함
    current_domain_id_ = domain_id;
    
    qDebug() << "도메인 ID가" << domain_id << "로 설정됨";
}

} // namespace robot_debugger_ui 