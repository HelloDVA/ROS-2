#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/utilities.hpp>

#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <string>

#include "chatnode.hpp"

using namespace std::chrono_literals;

ChatNode::ChatNode(const std::string& name) : Node(name), node_name_(name) {
    // 根据name生成唯一id 
    auto now = std::chrono::system_clock::now();  
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count(); 
    node_id_ = node_name_ + "_" + std::to_string(time);

    publisher_ = create_publisher<ChatMessage>("chat", 10);
    subscription_ = create_subscription<ChatMessage>("chat", 10, std::bind(&ChatNode::ChatCallback, this, std::placeholders::_1));

    register_node_client_ = create_client<RegisterNode>("register_node"); 
    get_nodes_client_ = create_client<GetNodes>("get_nodes");

    // 等待服务可用
    while (!register_node_client_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "等待发现服务...");
    }
    while (!get_nodes_client_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "等待节点列表服务...");
    }

    timer_ = create_wall_timer(5s, std::bind(&ChatNode::HeartBeat, this));

    running_ = true;

    RCLCPP_INFO(get_logger(), "聊天节点 %s 初始化完成", node_name_.c_str());
}

ChatNode::~ChatNode() {
    running_ = false;
    if (input_thread_.joinable()) {
        input_thread_.join();
    }
}

void ChatNode::Run() {
    // 注册到发现服务
    if (!RegisterNodeService()) {
        RCLCPP_ERROR(this->get_logger(), "注册到发现服务失败");
        return;
    }

    // 打印欢迎信息
    std::cout << "\n=== 聊天系统 ===" << std::endl;
    std::cout << "节点: " << node_name_ << std::endl;
    std::cout << "可用命令:" << std::endl;
    std::cout << "  /list - 查看在线节点" << std::endl;
    std::cout << "  /quit - 退出聊天" << std::endl;
    std::cout << "  @目标节点 消息 - 私聊消息" << std::endl;
    std::cout << "  普通消息 - 群聊广播\n" << std::endl;

    input_thread_ = std::thread(&ChatNode::InputThreadFunc, this);

    if (input_thread_.joinable()) {
        // 将线程设置为分离状态，让它在后台运行
        input_thread_.detach();
        RCLCPP_INFO(this->get_logger(), "输入线程已启动（分离模式）");
    } else {
        RCLCPP_ERROR(this->get_logger(), "输入线程启动失败");
    }
    
    rclcpp::spin(shared_from_this());
}

void ChatNode::InputThreadFunc() { 
    std::string input;

    while (running_ && rclcpp::ok()) {
        std::cout << node_name_ << ">>";
        std::getline(std::cin, input);

        // 特殊命令
        if (input == "/quit") {
            rclcpp::shutdown();             
            break;
        } else if (input == "/list") {
            GetNodesService();
            continue;
        } else if (input.empty()) {
            continue;
        }        
        
        // 构造消息
        auto message = std::make_shared<ChatMessage>();
        message->sender_name = node_name_;
        message->sender_id = node_id_;
        message->timestamp = get_clock()->now();
            
        // 根据是否有@符号判断是群发还是私发送 
        // @node1 Hello
        if (input.find('@') != std::string::npos) {
            size_t index = input.find(' ');

            if (index != std::string::npos) {
                std::string recive_name = input.substr(1, index - 1);
                std::string content = input.substr(index + 1, input.size());

                message->recive_name = recive_name;
                message->content = content;
                message->is_private = true;
            } else {
                std::cout << "the input not ok, please input like this : @name content";
                continue;
            }
        } else {
            message->recive_name = "all";
            message->content = input;
            message->is_private = false;
        }

        publisher_->publish(*message);
    }
}

void ChatNode::HeartBeat() {
    auto request = std::make_shared<RegisterNode::Request>();
    request->node_name = node_name_;
    request->node_id = node_id_;
    
    auto future = register_node_client_->async_send_request(request);
}

void ChatNode::ChatCallback(const ChatMessage::SharedPtr message) {
    if (message->sender_name == node_name_) {
        return;
    }
    
    // 格式化时间
    auto timestamp = message->timestamp; 
    std::string time_str = TimeToString(timestamp);

    // 分发私发与群聊消息
    if (message->is_private) {
        if (message->recive_name == node_name_) {
            std::cout << time_str << " 悄悄话：" << message->sender_name << ": " << message->content << std::endl;
        } 
    } else {
        std::cout << time_str << " " << message->sender_name << ": " << message->content << std::endl;
    }
        
    // 提示输入
    std::cout << node_name_ << ">>";
}

std::string ChatNode::TimeToString(const rclcpp::Time& timestamp) {
    auto nanoseconds = std::chrono::nanoseconds(timestamp.nanoseconds());
    auto time_point = std::chrono::system_clock::time_point(nanoseconds);

    auto time = std::chrono::system_clock::to_time_t(time_point);

    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_point.time_since_epoch()) % 1000;
    
    // 注意localtime不是线程安全的
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << milliseconds.count();
    
    return ss.str();
}

bool ChatNode::RegisterNodeService() {
    auto request = std::make_shared<RegisterNode::Request>();
    request->node_name = node_name_;
    request->node_id = node_id_;
    
    auto future = register_node_client_->async_send_request(request);

    auto result = rclcpp::spin_until_future_complete(this->shared_from_this(), future, std::chrono::seconds(5));

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        try {
            auto response = future.get();
            if (response->success)
                return true;
            else 
                return false;
        } catch (const std::exception& e) {
            std::cerr << "服务调用异常\n"; 
        }
    } else if (result == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "Request timeout");
    } else {
        RCLCPP_ERROR(get_logger(), "Request failed");
    }

    return false;
}

void ChatNode::GetNodesService() {
    auto request = std::make_shared<GetNodes::Request>();

    auto future = get_nodes_client_->async_send_request(request);

    while (true) {
        auto state = future.wait_for(10ms);

        if (state == std::future_status::ready) {
            try {
                auto response = future.get();
                std::cout << "\n=== 在线节点 " << response->online_nodes.size() << " ===" << std::endl;
                for (auto& online_node : response->online_nodes) {
                    std::string name = node_name_ == online_node.node_name ? "you" : online_node.node_name;
                    std::cout << "Name: " << name << std::endl;
                }
            } catch(const std::exception& e) {
                std::cerr << "服务调用异常\n"; 
            }
            break;
        } else {
            std::cerr << "服务调用失败\n"; 
            break;
        } 
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "usage: ros2 run xxx xxx [name]\n";
        return 0;
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ChatNode>(argv[1]);

    node->Run();

    rclcpp::shutdown();
    return 0;
} 
