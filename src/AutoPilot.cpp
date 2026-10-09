#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>

class AutoPilot : public rclcpp::Node
{
public:
    AutoPilot() : Node("autopilot")
    {
        // Suscripción al tópico /scan del LiDAR
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&AutoPilot::scan_callback, this, std::placeholders::_1));

        // Publicador hacia /cmd_vel
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "AutoPilot iniciado: Feature 1 (Filtro LiDAR +/-30 deg).");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty()) {
            return;
        }

        auto cmd_msg = geometry_msgs::msg::Twist();

        // Ventana angular frontal: +/- 30 grados convertidos a radianes (~0.5236 rad)
        const float angle_window = 30.0f * (M_PI / 180.0f);
        const float safety_distance = 0.40f; // Distancia de seguridad (40 cm)
        float min_front_distance = msg->range_max;

        // Recorrer todos los rayos del escaneo
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;

            // Normalizar el ángulo dentro del rango [-PI, PI]
            while (angle > M_PI) angle -= 2.0f * M_PI;
            while (angle < -M_PI) angle += 2.0f * M_PI;

            // Filtrar únicamente los rayos dentro del arco [-30°, +30°]
            if (angle >= -angle_window && angle <= angle_window) {
                float dist = msg->ranges[i];

                // Descartar lecturas fuera de rango válido o valores NaN/Infinitos
                if (dist >= msg->range_min && dist <= msg->range_max && !std::isnan(dist)) {
                    if (dist < min_front_distance) {
                        min_front_distance = dist;
                    }
                }
            }
        }

        // Si se detecta un obstáculo dentro de los 40 cm en cualquier punto del arco frontal
        if (min_front_distance < safety_distance) {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.3; // Gira sobre su eje
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Obstaculo detectado a %.2f m en el sector frontal. Girando...", min_front_distance);
        } else {
            cmd_msg.linear.x = 0.12; // Avanza en línea recta a velocidad prudente
            cmd_msg.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoPilot>());
    rclcpp::shutdown();
    return 0;
}
