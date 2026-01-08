#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "discoveryserver.hpp"

using namespace std::chrono_literals;

DiscoveryServer::DiscoveryServer() : Node("discoveryserver") {
    // 注册服务
    register_service_ = create_service<RegisterNode>(
        "register_node",
        [this](const std::shared_ptr<RegisterNode::Request> request,
               std::shared_ptr<RegisterNode::Response> response) {
            this->HandleRegistration(request, response);
     });

    get_nodes_service_ = create_service<GetNodes>(
        "get_nodes",
        [this](const std::shared_ptr<GetNodes::Request> request, std::shared_ptr<GetNodes::Response> response) {
            this->HandleGetNodes(request, response);
     });
    
    clean_timer_ = create_wall_timer(10s, std::bind(&DiscoveryServer::CleanTimer, this));
}

void DiscoveryServer::HandleRegistration(const std::shared_ptr<RegisterNode::Request> request, std::shared_ptr<RegisterNode::Response> response) {
    std::string node_id = request->node_id;

    NodeInfo node;
    node.node_id = request->node_id;
    node.node_name = request->node_name;
    node.last_seen = now();

    std::lock_guard<std::mutex> lock(nodes_mutex_);
    
    // 节点存在更新时间戳
    if (nodes_.find(node_id) != nodes_.end()) {
        nodes_[node_id].last_seen = now();
    } else {
        nodes_[node_id] = node;
        RCLCPP_INFO(get_logger(), "节点注册: %s -> %s", request->node_name.c_str(), node_id.c_str());     
    }

    response->success = true;
}

void DiscoveryServer::HandleGetNodes(const std::shared_ptr<GetNodes::Request> request, std::shared_ptr<GetNodes::Response> response) {
    // 消除警告信息
    (void)request;

    response->online_nodes.clear();
    
    std::lock_guard<std::mutex> lock(nodes_mutex_);

    response->online_nodes.reserve(nodes_.size());

    for(const auto& online_node : nodes_) {
        auto& node = response->online_nodes.emplace_back();
        node.node_id = online_node.second.node_id;
        node.node_name = online_node.second.node_name;
        node.last_seen = online_node.second.last_seen;
    }
}

void DiscoveryServer::CleanTimer() {
    auto now = get_clock()->now(); 

    std::lock_guard<std::mutex> lock(nodes_mutex_);

    for (auto it = nodes_.begin(); it != nodes_.end();) {
        rclcpp::Time last_seen = it->second.last_seen;
        double elapsed = (now - last_seen).seconds();
        const double timeout_threshold = 30.0;

        if (elapsed > timeout_threshold) {
            // 超时节点删除
            RCLCPP_INFO(get_logger(), "清除失效节点: %s", it->second.node_name.c_str());     
            it = nodes_.erase(it);
        } else {
            ++it;
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto service = std::make_shared<DiscoveryServer>(); 
    std::cout << "Discovery service is running" << std::endl;
    rclcpp::spin(service);
    rclcpp::shutdown();
    return 0;
}
