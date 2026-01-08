#include "chat_interfaces/msg/node_info.hpp"
#include "chat_interfaces/srv/register_node.hpp"
#include "chat_interfaces/srv/get_nodes.hpp"

#include <cstdint>
#include <map>
#include <mutex>
#include <rclcpp/timer.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

class DiscoveryServer : public rclcpp::Node {
    public:
        using RegisterNode = chat_interfaces::srv::RegisterNode; 
        using GetNodes = chat_interfaces::srv::GetNodes;

        using NodeInfo = chat_interfaces::msg::NodeInfo; 

        DiscoveryServer(); 

    private:
        rclcpp::Service<RegisterNode>::SharedPtr register_service_;
        rclcpp::Service<GetNodes>::SharedPtr get_nodes_service_;
        rclcpp::TimerBase::SharedPtr clean_timer_;

        std::unordered_map<std::string, NodeInfo> nodes_;
        std::multimap<int64_t, std::string> nodes_time_;
        std::mutex nodes_mutex_;

        void HandleRegistration(const std::shared_ptr<RegisterNode::Request> request, std::shared_ptr<RegisterNode::Response> response);
        void HandleGetNodes(const std::shared_ptr<GetNodes::Request> request, std::shared_ptr<GetNodes::Response> response);
        void CleanTimer();
};
