#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DriveNode : public rclcpp::Node {
public:
    DriveNode() : Node("drive_node"), energy_mode_("NORMAL") {
        energy_sub_ = this->create_subscription<std_msgs::msg::String>(
            "energy_status",
            10,
            std::bind(&DriveNode::energy_callback, this, std::placeholders::_1));

        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "drive_command",
            10,
            std::bind(&DriveNode::command_callback, this, std::placeholders::_1));
    }

private:
    void energy_callback(const std_msgs::msg::String::SharedPtr msg) {
        energy_mode_ = msg->data;
        RCLCPP_INFO(this->get_logger(),
                    "Drive: energy mode updated -> %s",
                    energy_mode_.c_str());
    }

    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (energy_mode_ == "CRITICAL") {
            RCLCPP_WARN(this->get_logger(),
                        "Drive: CRITICAL energy! Ignoring command [%s]",
                        msg->data.c_str());
            return;
        }

        if (energy_mode_ == "LOW_POWER") {
            RCLCPP_INFO(this->get_logger(),
                        "Drive: LOW_POWER mode, limiting movement [%s]",
                        msg->data.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Drive: executing command [%s]",
                        msg->data.c_str());
        }
    }

    std::string energy_mode_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr energy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}
