#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class EnergyNode : public rclcpp::Node {
public:
    EnergyNode() : Node("energy_management_node"), battery_level_(100) {
        publisher_ =
            this->create_publisher<std_msgs::msg::String>("energy_status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&EnergyNode::publish_energy_status, this));
    }

private:
    void publish_energy_status() {
        auto msg = std_msgs::msg::String();

        if (battery_level_ > 60)
            msg.data = "NORMAL";
        else if (battery_level_ > 30)
            msg.data = "LOW_POWER";
        else
            msg.data = "CRITICAL";

        RCLCPP_INFO(this->get_logger(),
                    "Energy System: battery %d%% -> mode [%s]",
                    battery_level_,
                    msg.data.c_str());

        publisher_->publish(msg);
        battery_level_ -= 10;
        if (battery_level_ < 0) battery_level_ = 100;
    }

    int battery_level_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnergyNode>());
    rclcpp::shutdown();
    return 0;
}
