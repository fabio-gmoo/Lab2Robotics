#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class AutoPilot : public rclcpp::Node
{
public:
    AutoPilot() : Node("autopilot")
    {
        // Subscriber to /scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&AutoPilot::scan_callback, this, std::placeholders::_1));

        // Publisher to /cmd_vel
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto cmd_msg = geometry_msgs::msg::Twist();

        // Simple obstacle avoidance: if obstacle is closer than 1.0 meter in front, turn
        int center_index = msg->ranges.size() / 2;
        float front_distance = msg->ranges[center_index];

        if (front_distance < 1.0)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.1; // turn
        }
        else
        {
            cmd_msg.linear.x = 0.1; // move forward
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
