#ifndef POWER_MANAGER_HPP
#define POWER_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>
#include <cmath>

namespace rover_energy {
enum class PowerMode {
    NORMAL,         
    LOW_POWER,      
    HIBERNATION,      
    EMERGENCY
};

struct EnergyState {
    float battery_soc;      
    float voltage;              
    float current;      
    float power_consumption;    
    float solar_generation; 
    float temperature;      
    PowerMode mode;
};

enum class ComponentPriority {
    CRITICAL = 0,   
    HIGH = 1,   
    MEDIUM = 2,
    LOW = 3 
};

struct PowerComponent {
    std::string name;
    ComponentPriority priority;
    float nominal_power;
    float current_power;
    bool is_enabled;
    bool is_essential;
};

class PowerManager : public rclcpp::Node {
public:
    PowerManager() : Node("power_manager"), 
                     current_mode_(PowerMode::NORMAL),
                     last_prediction_time_(this->now()) {
        
        energy_state_.battery_soc = 100.0f;
        energy_state_.voltage = 28.0f;
        energy_state_.current = 0.0f;
        energy_state_.power_consumption = 0.0f;
        energy_state_.solar_generation = 0.0f;
        energy_state_.temperature = 20.0f;
        energy_state_.mode = PowerMode::NORMAL;

        initializeComponents();

        power_mode_pub_ = this->create_publisher<std_msgs::msg::String>(
            "power/mode", 10);
        
        battery_status_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "power/battery_soc", 10);
        
        power_budget_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "power/available_power", 10);

        battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "sensors/battery_voltage", 10,
            std::bind(&PowerManager::batteryCallback, this, std::placeholders::_1));
        
        solar_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "sensors/solar_power", 10,
            std::bind(&PowerManager::solarCallback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&PowerManager::velocityCallback, this, std::placeholders::_1));

        management_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PowerManager::managementLoop, this));

        prediction_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PowerManager::predictionLoop, this));

        RCLCPP_INFO(this->get_logger(), "PowerManager initialized");
    }

    PowerMode getCurrentMode() const { return current_mode_; }
    
    EnergyState getEnergyState() const { return energy_state_; }
    
    void setMode(PowerMode mode) {
        if (mode != current_mode_) {
            switchMode(mode);
        }
    }

    float getAvailablePower() const {
        float net_power = energy_state_.solar_generation - 
                         getCriticalPowerConsumption();
        return std::max(0.0f, net_power);
    }

private:
    void initializeComponents() {
        components_.push_back({"communication", ComponentPriority::CRITICAL, 15.0f, 15.0f, true, true});
        components_.push_back({"fdir_watchdog", ComponentPriority::CRITICAL, 5.0f, 5.0f, true, true});
        components_.push_back({"navigation", ComponentPriority::HIGH, 25.0f, 25.0f, true, true});
        components_.push_back({"motors", ComponentPriority::HIGH, 50.0f, 0.0f, true, true});
        components_.push_back({"lidar", ComponentPriority::MEDIUM, 20.0f, 20.0f, true, false});
        components_.push_back({"cameras", ComponentPriority::MEDIUM, 15.0f, 15.0f, true, false});
        components_.push_back({"science_instruments", ComponentPriority::LOW, 30.0f, 0.0f, true, false});
        components_.push_back({"heating", ComponentPriority::MEDIUM, 40.0f, 0.0f, true, false});
    }

    void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        energy_state_.voltage = msg->data;
        
        energy_state_.battery_soc = ((msg->data - 24.0f) / 5.4f) * 100.0f;
        energy_state_.battery_soc = std::clamp(energy_state_.battery_soc, 0.0f, 100.0f);
        
        auto soc_msg = std_msgs::msg::Float32();
        soc_msg.data = energy_state_.battery_soc;
        battery_status_pub_->publish(soc_msg);
    }

    void solarCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        energy_state_.solar_generation = msg->data;
    }

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        float speed = std::abs(msg->linear.x);
        float angular = std::abs(msg->angular.z);
        
        float motor_power = 10.0f + 40.0f * speed + 20.0f * angular;
        
        for (auto& comp : components_) {
            if (comp.name == "motors") {
                comp.current_power = motor_power;
                break;
            }
        }
    }

    void managementLoop() {
        updatePowerConsumption();
        
        float power_balance = energy_state_.solar_generation - 
                             energy_state_.power_consumption;
        
        PowerMode target_mode = determineTargetMode(
            energy_state_.battery_soc, power_balance);
        
        if (target_mode != current_mode_) {
            switchMode(target_mode);
        }
        
        allocatePower();
        
        auto power_msg = std_msgs::msg::Float32();
        power_msg.data = getAvailablePower();
        power_budget_pub_->publish(power_msg);
    }

    void predictionLoop() {
        auto current_time = this->now();
        float dt = (current_time - last_prediction_time_).seconds();
        last_prediction_time_ = current_time;
        
        float predicted_energy = predictEnergyForNextSol();
        
        RCLCPP_INFO(this->get_logger(), 
            "Energy prediction for next sol: %.2f Wh | Current SOC: %.1f%% | Mode: %s",
            predicted_energy, energy_state_.battery_soc, 
            powerModeToString(current_mode_).c_str());
    }

    void updatePowerConsumption() {
        float total = 0.0f;
        for (const auto& comp : components_) {
            if (comp.is_enabled) {
                total += comp.current_power;
            }
        }
        energy_state_.power_consumption = total;
    }

    PowerMode determineTargetMode(float soc, float power_balance) {
        if (soc < 15.0f) {
            return PowerMode::EMERGENCY;
        }
        
        if (energy_state_.solar_generation < 5.0f && soc < 50.0f) {
            return PowerMode::HIBERNATION;
        }
        
        if (soc < 30.0f || power_balance < -10.0f) {
            return PowerMode::LOW_POWER;
        }
        
        if (soc > 40.0f && power_balance > 0.0f) {
            return PowerMode::NORMAL;
        }
        
        return current_mode_;
    }

    void switchMode(PowerMode new_mode) {
        RCLCPP_WARN(this->get_logger(), 
            "Switching power mode: %s -> %s",
            powerModeToString(current_mode_).c_str(),
            powerModeToString(new_mode).c_str());
        
        current_mode_ = new_mode;
        energy_state_.mode = new_mode;
    
        auto mode_msg = std_msgs::msg::String();
        mode_msg.data = powerModeToString(new_mode);
        power_mode_pub_->publish(mode_msg);
        
        adjustComponentsForMode(new_mode);
    }

    void adjustComponentsForMode(PowerMode mode) {
        switch (mode) {
            case PowerMode::NORMAL:
                // Wszystkie komponenty włączone
                for (auto& comp : components_) {
                    comp.is_enabled = true;
                }
                break;
                
            case PowerMode::LOW_POWER:
                for (auto& comp : components_) {
                    if (comp.priority == ComponentPriority::LOW) {
                        comp.is_enabled = false;
                    }
                    if (comp.name == "cameras") {
                        comp.is_enabled = false;
                    }
                }
                break;
                
            case PowerMode::HIBERNATION:
                for (auto& comp : components_) {
                    if (comp.priority != ComponentPriority::CRITICAL && !comp.is_essential) {
                        comp.is_enabled = false;
                    }
                }
                break;
                
            case PowerMode::EMERGENCY:
                for (auto& comp : components_) {
                    if (comp.priority == ComponentPriority::CRITICAL) {
                        comp.is_enabled = true;
                    } else {
                        comp.is_enabled = false;
                    }
                }
                break;
        }
    }

    void allocatePower() {
        float available_power = energy_state_.solar_generation;
    
        std::vector<PowerComponent*> sorted_components;
        for (auto& comp : components_) {
            if (comp.is_enabled) {
                sorted_components.push_back(&comp);
            }
        }
        
        std::sort(sorted_components.begin(), sorted_components.end(),
            [](const PowerComponent* a, const PowerComponent* b) {
                return a->priority < b->priority;
            });
        
        for (auto* comp : sorted_components) {
            if (available_power >= comp->nominal_power) {
                comp->current_power = comp->nominal_power;
                available_power -= comp->nominal_power;
            } else {
                comp->current_power = available_power;
                available_power = 0.0f;
            }
        }
    }

    float getCriticalPowerConsumption() const {
        float critical_power = 0.0f;
        for (const auto& comp : components_) {
            if (comp.is_enabled && comp.priority == ComponentPriority::CRITICAL) {
                critical_power += comp.current_power;
            }
        }
        return critical_power;
    }

    float predictEnergyForNextSol() {
        float avg_solar_generation = 80.0f; // Średnia generacja [W]
        float sol_duration = 24.6f * 3600.0f; // Czas sola [s]
        float daylight_fraction = 0.5f; // 50% czasu to dzień
        
        float predicted_generation = avg_solar_generation * sol_duration * 
                                    daylight_fraction / 3600.0f; // [Wh]
        
        float avg_consumption = 40.0f; // Średnie zużycie [W]
        float predicted_consumption = avg_consumption * sol_duration / 3600.0f;
        
        return predicted_generation - predicted_consumption;
    }

    std::string powerModeToString(PowerMode mode) const {
        switch (mode) {
            case PowerMode::NORMAL: return "NORMAL";
            case PowerMode::LOW_POWER: return "LOW_POWER";
            case PowerMode::HIBERNATION: return "HIBERNATION";
            case PowerMode::EMERGENCY: return "EMERGENCY";
            default: return "UNKNOWN";
        }
    }

    PowerMode current_mode_;
    EnergyState energy_state_;
    std::vector<PowerComponent> components_;
    rclcpp::Time last_prediction_time_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr power_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_budget_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr solar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::TimerBase::SharedPtr management_timer_;
    rclcpp::TimerBase::SharedPtr prediction_timer_;
};

}

#endif // POWER_MANAGER_HPP

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto power_manager = std::make_shared<rover_energy::PowerManager>();
    
    RCLCPP_INFO(power_manager->get_logger(), 
        "Rover Power Management System started");
    
    rclcpp::spin(power_manager);
    
    rclcpp::shutdown();
    return 0;
}
