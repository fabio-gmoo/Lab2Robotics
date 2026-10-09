#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

// Estados posibles del robot
enum class RobotState {
    FORWARD,
    OBSTACLE_AVOIDANCE,
    STOPPED
};

class AutoPilot : public rclcpp::Node
{
public:
    AutoPilot() : Node("autopilot"), current_state_(RobotState::FORWARD)
    {
        // 1. Percepción: Suscriptor al tópico /scan del LiDAR
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&AutoPilot::scan_callback, this, std::placeholders::_1));

        // 2. Actuación: Publicador de velocidad a /cmd_vel
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 3. Control: Bucle periódico a 10 Hz (cada 100 ms)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&AutoPilot::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "AutoPilot init: Feature 2 (10 Hz)");
    }

private:
    // Callback de percepción: Solo analiza datos del LiDAR y actualiza banderas
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty()) return;

        const float angle_window = 30.0f * (M_PI / 180.0f); // +/- 30 grados
        const float safety_distance = 0.40f;                // Umbral de 40 cm
        float min_front_distance = msg->range_max;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;

            while (angle > M_PI) angle -= 2.0f * M_PI;
            while (angle < -M_PI) angle += 2.0f * M_PI;

            if (angle >= -angle_window && angle <= angle_window) {
                float dist = msg->ranges[i];
                if (dist >= msg->range_min && dist <= msg->range_max && !std::isnan(dist)) {
                    if (dist < min_front_distance) {
                        min_front_distance = dist;
                    }
                }
            }
        }

        // Se actualiza la bandera para que el bucle de control la consulte
        obstacle_detected_ = (min_front_distance < safety_distance);
    }

    // Funciones modulares de actuación
    void move_forward()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.12;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    void rotate_in_place()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.3; // Rotación constante básica (en Feature 3 será aleatoria)
        cmd_pub_->publish(cmd);
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    // Bucle de control periódico (FSM)
    void control_loop()
    {
        // Transición de estados según la percepción
        if (obstacle_detected_) {
            current_state_ = RobotState::OBSTACLE_AVOIDANCE;
        } else {
            current_state_ = RobotState::FORWARD;
        }

        // Ejecución de acciones según el estado activo
        switch (current_state_) {
            case RobotState::FORWARD:
                move_forward();
                break;

            case RobotState::OBSTACLE_AVOIDANCE:
                rotate_in_place();
                break;

            case RobotState::STOPPED:
                stop_robot();
                break;
        }
    }

    // Suscriptores, publicadores y temporizador
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables internas de estado
    RobotState current_state_;
    bool obstacle_detected_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoPilot>());
    rclcpp::shutdown();
    return 0;
}
