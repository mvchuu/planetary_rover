#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <cmath>

/**
 * @brief Węzeł zarządzania energią łazika planetarnego
 * 
 * Implementuje model energetyczny zgodny z raportem:
 * E(t+1) = E(t) - (E_drive + E_sensors + E_CPU) + E_solar
 */
class EnergyManagementNode : public rclcpp::Node {
public:
    EnergyManagementNode() 
        : Node("energy_management_node"),
          battery_level_(100.0f),
          battery_voltage_(28.8f),
          temperature_(25.0f),
          charging_(false),
          solar_generation_(0.0f),
          consumption_rate_(2.0f),
          current_mode_(PowerMode::NORMAL)
    {
        // Publishers - telemetria
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "energy/status", 10);
        
        battery_level_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "energy/battery_level", 10);
        
        battery_voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "energy/battery_voltage", 10);
        
        temperature_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "energy/battery_temperature", 10);
        
        solar_power_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "energy/solar_power", 10);
        
        consumption_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "energy/consumption_rate", 10);

        // Subscribers - sterowanie
        charging_sub_ = this->create_subscription<std_msgs::msg::String>(
            "energy/charging_command", 10,
            std::bind(&EnergyManagementNode::chargingCallback, this, 
                     std::placeholders::_1));
        
        solar_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "energy/solar_input", 10,
            std::bind(&EnergyManagementNode::solarCallback, this, 
                     std::placeholders::_1));

        // Timers
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&EnergyManagementNode::updateBattery, this));
        
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&EnergyManagementNode::publishTelemetry, this));

        RCLCPP_INFO(this->get_logger(), 
                   "╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), 
                   "║  Energy Management System INITIALIZED         ║");
        RCLCPP_INFO(this->get_logger(), 
                   "║  Battery: %.1f%% | Voltage: %.1fV | Temp: %.1f°C ║", 
                   battery_level_, battery_voltage_, temperature_);
        RCLCPP_INFO(this->get_logger(), 
                   "╚════════════════════════════════════════════════╝");
    }

private:
    enum class PowerMode {
        NORMAL,
        LOW_POWER,
        CRITICAL,
        EMERGENCY
    };

    // Stan baterii
    float battery_level_;        // 0-100%
    float battery_voltage_;      // V
    float temperature_;          // °C
    bool charging_;
    float solar_generation_;     // W
    float consumption_rate_;     // W
    PowerMode current_mode_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_level_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr solar_power_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr consumption_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr charging_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr solar_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    /**
     * @brief Callback dla komend ładowania
     */
    void chargingCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "START") {
            charging_ = true;
            solar_generation_ = 80.0f; // Symulacja paneli słonecznych
            RCLCPP_INFO(this->get_logger(), 
                       "CHARGING STARTED - Solar panels deployed");
        } 
        else if (msg->data == "STOP") {
            charging_ = false;
            solar_generation_ = 0.0f;
            RCLCPP_INFO(this->get_logger(), 
                       "CHARGING STOPPED");
        }
        else if (msg->data == "NIGHT") {
            charging_ = false;
            solar_generation_ = 0.0f;
            RCLCPP_WARN(this->get_logger(), 
                       "NIGHT MODE - No solar generation");
        }
    }

    /**
     * @brief Callback dla danych z paneli słonecznych
     */
    void solarCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        solar_generation_ = msg->data;
    }

    /**
     * @brief Aktualizacja stanu baterii
     * Model: E(t+1) = E(t) - (E_drive + E_sensors + E_CPU) + E_solar
     */
    void updateBattery() {
        float dt = 0.5f; // 500ms w sekundach
        
        // Oblicz zużycie bazowe na trybie pracy
        float base_consumption = getBaseConsumption();
        
        if (charging_) {
            // Ładowanie: energia rośnie
            float charge_rate = (solar_generation_ - base_consumption) * dt / 3600.0f;
            battery_level_ += charge_rate * 100.0f / 50.0f; // Zakładamy 50Wh pojemność
            
            if (battery_level_ > 100.0f) {
                battery_level_ = 100.0f;
                charging_ = false;
                solar_generation_ = 0.0f;
                RCLCPP_INFO(this->get_logger(), 
                           "BATTERY FULLY CHARGED - Auto-stop charging");
            }
            
            // Temperatura rośnie podczas ładowania
            temperature_ += 0.05f;
            if (temperature_ > 45.0f) {
                temperature_ = 45.0f;
                RCLCPP_WARN(this->get_logger(), 
                           "HIGH TEMPERATURE WARNING: %.1f°C", temperature_);
            }
        } 
        else {
            // Rozładowanie
            float discharge_rate = base_consumption * dt / 3600.0f;
            battery_level_ -= discharge_rate * 100.0f / 50.0f;
            
            if (battery_level_ < 0.0f) {
                battery_level_ = 0.0f;
                RCLCPP_ERROR(this->get_logger(), 
                            "CRITICAL: BATTERY DEPLETED!");
            }
            
            // Temperatura spada
            temperature_ -= 0.03f;
            if (temperature_ < 20.0f) {
                temperature_ = 20.0f;
            }
        }

        // Aktualizacja napięcia (model liniowy)
        battery_voltage_ = 24.0f + (battery_level_ / 100.0f) * 4.8f;

        // Aktualizacja trybu pracy
        updatePowerMode();

        // Aktualizacja zużycia
        consumption_rate_ = base_consumption;
    }

    /**
     * @brief Oblicz bazowe zużycie energii w zależności od trybu
     */
    float getBaseConsumption() {
        switch (current_mode_) {
            case PowerMode::NORMAL:
                return 40.0f; // W
            case PowerMode::LOW_POWER:
                return 20.0f; // W
            case PowerMode::CRITICAL:
                return 10.0f; // W
            case PowerMode::EMERGENCY:
                return 5.0f;  // W - tylko podstawowe systemy
            default:
                return 40.0f;
        }
    }

    /**
     * @brief Aktualizuj tryb pracy na podstawie poziomu baterii
     */
    void updatePowerMode() {
        PowerMode old_mode = current_mode_;

        if (battery_level_ > 60.0f) {
            current_mode_ = PowerMode::NORMAL;
        } 
        else if (battery_level_ > 30.0f) {
            current_mode_ = PowerMode::LOW_POWER;
        } 
        else if (battery_level_ > 10.0f) {
            current_mode_ = PowerMode::CRITICAL;
        } 
        else {
            current_mode_ = PowerMode::EMERGENCY;
        }

        // Log zmiany trybu
        if (old_mode != current_mode_) {
            RCLCPP_WARN(this->get_logger(), 
                       "POWER MODE CHANGED: %s → %s (Battery: %.1f%%)",
                       getModeString(old_mode).c_str(),
                       getModeString(current_mode_).c_str(),
                       battery_level_);
        }

        // Krytyczne ostrzeżenia
        if (current_mode_ == PowerMode::EMERGENCY && 
            old_mode != PowerMode::EMERGENCY) {
            RCLCPP_ERROR(this->get_logger(), 
                        "EMERGENCY MODE ACTIVATED!");
            RCLCPP_ERROR(this->get_logger(), 
                        "Battery critically low: %.1f%%", battery_level_);
        }
    }

    /**
     * @brief Publikuj telemetrię
     */
    void publishTelemetry() {
        // Status jako string
        auto status_msg = std_msgs::msg::String();
        status_msg.data = getModeString(current_mode_);
        status_pub_->publish(status_msg);

        // Poziom baterii
        auto battery_msg = std_msgs::msg::Float32();
        battery_msg.data = battery_level_;
        battery_level_pub_->publish(battery_msg);

        // Napięcie
        auto voltage_msg = std_msgs::msg::Float32();
        voltage_msg.data = battery_voltage_;
        battery_voltage_pub_->publish(voltage_msg);

        // Temperatura
        auto temp_msg = std_msgs::msg::Float32();
        temp_msg.data = temperature_;
        temperature_pub_->publish(temp_msg);

        // Moc z paneli słonecznych
        auto solar_msg = std_msgs::msg::Float32();
        solar_msg.data = solar_generation_;
        solar_power_pub_->publish(solar_msg);

        // Zużycie
        auto consumption_msg = std_msgs::msg::Float32();
        consumption_msg.data = consumption_rate_;
        consumption_pub_->publish(consumption_msg);

        // Szczegółowy log
        std::string charge_status = charging_ ? "CHARGING" : "DISCHARGING";
        RCLCPP_INFO(this->get_logger(),
                   "Energy: %.1f%% [%s] | %.1fV | %.1f°C | Solar: %.1fW | Consumption: %.1fW | %s",
                   battery_level_,
                   status_msg.data.c_str(),
                   battery_voltage_,
                   temperature_,
                   solar_generation_,
                   consumption_rate_,
                   charge_status.c_str());
    }

    /**
     * @brief Konwersja trybu na string
     */
    std::string getModeString(PowerMode mode) {
        switch (mode) {
            case PowerMode::NORMAL:     return "NORMAL";
            case PowerMode::LOW_POWER:  return "LOW_POWER";
            case PowerMode::CRITICAL:   return "CRITICAL";
            case PowerMode::EMERGENCY:  return "EMERGENCY";
            default:                    return "UNKNOWN";
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnergyManagementNode>());
    rclcpp::shutdown();
    return 0;
}
