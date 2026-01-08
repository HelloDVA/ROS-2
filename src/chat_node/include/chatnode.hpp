
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <atomic>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <thread>

#include "chat_interfaces/msg/chat_message.hpp"
#include "chat_interfaces/srv/get_nodes.hpp"
#include "chat_interfaces/srv/register_node.hpp"

class ChatNode : public rclcpp::Node {
    public:
        using ChatMessage = chat_interfaces::msg::ChatMessage;

        using GetNodes = chat_interfaces::srv::GetNodes;
        using RegisterNode = chat_interfaces::srv::RegisterNode;

        ChatNode(const std::string& name);
        ~ChatNode();

        void Run();
    
    private:
        rclcpp::Publisher<ChatMessage>::SharedPtr publisher_;
        rclcpp::Subscription<ChatMessage>::SharedPtr subscription_;
        rclcpp::Client<GetNodes>::SharedPtr get_nodes_client_;
        rclcpp::Client<RegisterNode>::SharedPtr register_node_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string node_name_;
        std::string node_id_;
        std::atomic<bool> running_{false};
        std::thread input_thread_;

        void InputThreadFunc();

        void ChatCallback(const ChatMessage::SharedPtr message);
        std::string TimeToString(const rclcpp::Time& timestamp);
        void HeartBeat();

        bool RegisterNodeService();
        void GetNodesService();

};
