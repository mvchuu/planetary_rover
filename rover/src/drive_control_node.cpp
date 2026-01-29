#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include <map>

/**
 * @brief Węzeł sterowania napędem łazika planetarnego
 *
 * Reaguje na tryby energetyczne i wykonuje komendy ruchu
 * z uwzględnieniem ograniczeń energetycznych
 */
class DriveControlNode : public rclcpp::Node {
public:
    DriveControlNode() 
        : Node("drive_control_node"),
          energy_mode_("NORMAL"),
          battery_level_(100.0f),
          current_speed_(0.0f),
          max_speed_(100.0f),
          distance_traveled_(0.0f)
    {
        // Subscribers
        energy_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "energy/status", 10,
            std::bind(&DriveControlNode::energyStatusCallback, this, 
                     std::placeholders::_1));

        battery_level_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "energy/battery_level", 10,
            std::bind(&DriveControlNode::batteryLevelCallback, this, 
                     std::placeholders::_1));

        drive_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "drive/command", 10,
            std::bind(&DriveControlNode::driveCommandCallback, this, 
                     std::placeholders::_1));

        // Publishers
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "drive/current_speed", 10);
        
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "drive/status", 10);
        
        charging_control_pub_ = this->create_publisher<std_msgs::msg::String>(
            "energy/charging_command", 10);

        // Timer do symulacji ruchu
        motion_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DriveControlNode::updateMotion, this));

        // Timer do automatycznego sprawdzania stanu
        check_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&DriveControlNode::checkAutoCharge, this));

        RCLCPP_INFO(this->get_logger(), 
                   "╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), 
                   "║  Drive Control System INITIALIZED             ║");
        RCLCPP_INFO(this->get_logger(), 
                   "║  Max Speed: %.1f m/s | Status: READY          ║", 
                   max_speed_ / 100.0f);
        RCLCPP_INFO(this->get_logger(), 
                   "╚════════════════════════════════════════════════╝");
        
        printAvailableCommands();
    }

private:
    // Stan systemu napędu
    std::string energy_mode_;
    float battery_level_;
    float current_speed_;     // 0-100
    float max_speed_;         // 0-100
    float distance_traveled_; // metry
    bool is_moving_;

    // ROS2 Publishers & Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr energy_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_level_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_command_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr charging_control_pub_;
    
    rclcpp::TimerBase::SharedPtr motion_timer_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    /**
     * @brief Callback dla statusu energii
     */
    void energyStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string old_mode = energy_mode_;
        energy_mode_ = msg->data;
        
        if (old_mode != energy_mode_) {
            RCLCPP_INFO(this->get_logger(),
                       "Energy mode updated: %s to %s",
                       old_mode.c_str(),
                       energy_mode_.c_str());
            updateMaxSpeed();
        }
    }

    /**
     * @brief Callback dla poziomu baterii
     */
    void batteryLevelCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        battery_level_ = msg->data;
    }

    /**
     * @brief Aktualizacja maksymalnej prędkości na podstawie trybu energetycznego
     */
    void updateMaxSpeed() {
        float old_max = max_speed_;

        if (energy_mode_ == "NORMAL") {
            max_speed_ = 100.0f;  // ~1.0 m/s
        } 
        else if (energy_mode_ == "LOW_POWER") {
            max_speed_ = 50.0f;   // ~0.5 m/s
            if (current_speed_ > max_speed_) {
                current_speed_ = max_speed_;
                RCLCPP_WARN(this->get_logger(),
                           "Speed reduced to %.2f m/s due to LOW_POWER mode",
                           current_speed_ / 100.0f);
            }
        } 
        else if (energy_mode_ == "CRITICAL") {
            max_speed_ = 20.0f;   // ~0.2 m/s
            if (current_speed_ > max_speed_) {
                current_speed_ = max_speed_;
                RCLCPP_WARN(this->get_logger(),
                           "Speed reduced to %.2f m/s due to CRITICAL mode",
                           current_speed_ / 100.0f);
            }
        } 
        else if (energy_mode_ == "EMERGENCY") {
            max_speed_ = 0.0f;
            current_speed_ = 0.0f;
            RCLCPP_ERROR(this->get_logger(),
                        "ALL MOVEMENT DISABLED - EMERGENCY MODE!");
        }

        if (old_max != max_speed_) {
            RCLCPP_INFO(this->get_logger(),
                       "Max speed changed: %.2f → %.2f m/s",
                       old_max / 100.0f, max_speed_ / 100.0f);
        }
    }

    /**
     * @brief Callback dla komend jazdy
     */
    void driveCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;

        // Sprawdź czy można wykonać komendę
        if (energy_mode_ == "EMERGENCY" && 
            cmd != "STOP" && cmd != "CHARGE" && cmd != "STATUS") {
            RCLCPP_ERROR(this->get_logger(),
                        "EMERGENCY MODE! Command [%s] BLOCKED. Battery: %.1f%%",
                        cmd.c_str(), battery_level_);
            RCLCPP_ERROR(this->get_logger(),
                        "Only CHARGE command available!");
            return;
        }

        if (energy_mode_ == "CRITICAL" && 
            cmd != "STOP" && cmd != "CHARGE" && cmd != "STATUS" && 
            cmd != "SLOW_FORWARD") {
            RCLCPP_WARN(this->get_logger(),
                       "CRITICAL ENERGY! Command [%s] blocked. Battery: %.1f%%",
                       cmd.c_str(), battery_level_);
            RCLCPP_WARN(this->get_logger(),
                       "Consider charging. Only SLOW_FORWARD, STOP, CHARGE available.");
            return;
        }

        // Wykonaj komendę
        executeCommand(cmd);
    }

    /**
     * @brief Wykonanie komendy ruchu
     */
    void executeCommand(const std::string& cmd) {
        if (cmd == "FORWARD") {
            current_speed_ = max_speed_ * 0.8f;
            is_moving_ = true;
            RCLCPP_INFO(this->get_logger(), 
                       "Moving FORWARD at %.2f m/s", 
                       current_speed_ / 100.0f);
        }
        else if (cmd == "SLOW_FORWARD") {
            current_speed_ = max_speed_ * 0.3f;
            is_moving_ = true;
            RCLCPP_INFO(this->get_logger(), 
                       "Moving SLOW FORWARD at %.2f m/s", 
                       current_speed_ / 100.0f);
        }
        else if (cmd == "BACKWARD") {
            current_speed_ = -max_speed_ * 0.5f;
            is_moving_ = true;
            RCLCPP_INFO(this->get_logger(), 
                       "Moving BACKWARD at %.2f m/s", 
                       std::abs(current_speed_) / 100.0f);
        }
        else if (cmd == "STOP") {
            current_speed_ = 0.0f;
            is_moving_ = false;
            RCLCPP_INFO(this->get_logger(), "STOPPED");
        }
        else if (cmd == "SPEED_UP") {
            if (current_speed_ < max_speed_) {
                current_speed_ = std::min(current_speed_ + 10.0f, max_speed_);
                RCLCPP_INFO(this->get_logger(), 
                           "Speed increased to %.2f m/s", 
                           current_speed_ / 100.0f);
            } else {
                RCLCPP_WARN(this->get_logger(), 
                           "Already at max speed (%.2f m/s)", 
                           max_speed_ / 100.0f);
            }
        }
        else if (cmd == "SLOW_DOWN") {
            if (current_speed_ > 0.0f) {
                current_speed_ = std::max(current_speed_ - 10.0f, 0.0f);
                RCLCPP_INFO(this->get_logger(), 
                           "Speed decreased to %.2f m/s", 
                           current_speed_ / 100.0f);
            }
        }
        else if (cmd == "CHARGE") {
            current_speed_ = 0.0f;
            is_moving_ = false;
            auto charge_msg = std_msgs::msg::String();
            charge_msg.data = "START";
            charging_control_pub_->publish(charge_msg);
            RCLCPP_INFO(this->get_logger(), 
                       "CHARGING initiated. Deploying solar panels...");
        }
        else if (cmd == "STOP_CHARGE") {
            auto charge_msg = std_msgs::msg::String();
            charge_msg.data = "STOP";
            charging_control_pub_->publish(charge_msg);
            RCLCPP_INFO(this->get_logger(), "CHARGING stopped");
        }
        else if (cmd == "STATUS") {
            printStatus();
        }
        else {
            RCLCPP_WARN(this->get_logger(), 
                       "Unknown command: %s", cmd.c_str());
            printAvailableCommands();
        }

        publishStatus();
    }

    /**
     * @brief Aktualizacja ruchu (symulacja)
     */
    void updateMotion() {
        if (is_moving_ && current_speed_ != 0.0f) {
            // Symulacja przejazdu: 100 jednostek speed = 1 m/s
            float dt = 0.1f; // 100ms
            distance_traveled_ += (current_speed_ / 100.0f) * dt;
        }

        // Publikuj prędkość
        auto speed_msg = std_msgs::msg::Float32();
        speed_msg.data = current_speed_ / 100.0f;
        speed_pub_->publish(speed_msg);
    }

    /**
     * @brief Automatyczne sprawdzanie czy należy ładować
     */
    void checkAutoCharge() {
        if (battery_level_ < 15.0f && current_speed_ == 0.0f) {
            RCLCPP_WARN(this->get_logger(),
                       "AUTO-CHARGE recommended! Battery: %.1f%%", 
                       battery_level_);
            RCLCPP_INFO(this->get_logger(),
                       "Send command: CHARGE");
        }
    }

    /**
     * @brief Publikuj status napędu
     */
    void publishStatus() {
        auto status_msg = std_msgs::msg::String();
        if (current_speed_ > 0) {
            status_msg.data = "MOVING_FORWARD";
        } else if (current_speed_ < 0) {
            status_msg.data = "MOVING_BACKWARD";
        } else {
            status_msg.data = "STOPPED";
        }
        status_pub_->publish(status_msg);
    }

    /**
     * @brief Wyświetl szczegółowy status
     */
    void printStatus() {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔═══════════════════ DRIVE STATUS ═══════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║ Energy Mode:      %-30s ║", energy_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "║ Battery Level:    %.1f%%%-26s ║", 
                   battery_level_, "");
        RCLCPP_INFO(this->get_logger(), "║ Current Speed:    %.2f m/s%-23s ║", 
                   current_speed_ / 100.0f, "");
        RCLCPP_INFO(this->get_logger(), "║ Max Speed:        %.2f m/s%-23s ║", 
                   max_speed_ / 100.0f, "");
        RCLCPP_INFO(this->get_logger(), "║ Distance:         %.2f m%-25s ║", 
                   distance_traveled_, "");
        RCLCPP_INFO(this->get_logger(), "║ Status:           %-30s ║", 
                   is_moving_ ? "MOVING" : "STOPPED");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "");
    }
    /**
     * @brief Wyświetl dostępne komendy
     */
    void printAvailableCommands() {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔════════════════ AVAILABLE COMMANDS ═════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║ FORWARD        - Move forward at 80%% max speed     ║");
        RCLCPP_INFO(this->get_logger(), "║ SLOW_FORWARD   - Move forward at 30%% max speed     ║");
        RCLCPP_INFO(this->get_logger(), "║ BACKWARD       - Move backward at 50%% max speed    ║");
        RCLCPP_INFO(this->get_logger(), "║ STOP           - Stop all movement                  ║");
        RCLCPP_INFO(this->get_logger(), "║ SPEED_UP       - Increase speed by 10%%             ║");
        RCLCPP_INFO(this->get_logger(), "║ SLOW_DOWN      - Decrease speed by 10%%             ║");
        RCLCPP_INFO(this->get_logger(), "║ CHARGE         - Start battery charging             ║");
        RCLCPP_INFO(this->get_logger(), "║ STOP_CHARGE    - Stop battery charging              ║");
        RCLCPP_INFO(this->get_logger(), "║ STATUS         - Display current status             ║");
        RCLCPP_INFO(this->get_logger(), "╚═════════════════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveControlNode>());
    rclcpp::shutdown();
    return 0;
}
