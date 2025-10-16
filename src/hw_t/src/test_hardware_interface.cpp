#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("jog_and_speed_publisher");

    // Publisher to start/stop jog
    auto pub_jog = node->create_publisher<std_msgs::msg::Bool>("cmd_jog", 10);

    // Publisher for speed
    auto pub_speed = node->create_publisher<std_msgs::msg::Int32MultiArray>("cmd_speed", 10);

    rclcpp::Rate rate(1); // 1 Hz

    // Start jog
    std_msgs::msg::Bool jog_msg;
    jog_msg.data = true;
    pub_jog->publish(jog_msg);
    RCLCPP_INFO(node->get_logger(), "Jog started!");
    rclcpp::spin_some(node);
    rate.sleep(); // give time for the node to process jog

    // Send speed commands
    std_msgs::msg::Int32MultiArray speed_msg;
    speed_msg.data = {-50, -50}; // left, right wheel speeds
    pub_speed->publish(speed_msg);
    RCLCPP_INFO(node->get_logger(), "Published speeds: [%d, %d]", speed_msg.data[0], speed_msg.data[1]);
    rclcpp::spin_some(node);

    rclcpp::shutdown();
    return 0;
}
