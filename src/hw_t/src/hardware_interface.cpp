#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <memory>
#include <cmath>
#include "hw_t/moonsModbus.hpp"

using namespace std;

class MoonsNode : public rclcpp::Node {
public:
    MoonsNode() : Node("moons_node") {
        // Initialize Moons hardware interface with inversion (left=1, right=-1)
        moons_ = std::make_shared<Moons>(1, -1);

        // Publishers
        pub_enc1_ = this->create_publisher<std_msgs::msg::Int32>("wheel1/encoder", 10);
        pub_enc2_ = this->create_publisher<std_msgs::msg::Int32>("wheel2/encoder", 10);
        pub_vel1_ = this->create_publisher<std_msgs::msg::Float32>("wheel1/speed", 10);
        pub_vel2_ = this->create_publisher<std_msgs::msg::Float32>("wheel2/speed", 10);
        pub_cur1_ = this->create_publisher<std_msgs::msg::Float32>("wheel1/current", 10);
        pub_cur2_ = this->create_publisher<std_msgs::msg::Float32>("wheel2/current", 10);
        pub_alarm1_ = this->create_publisher<std_msgs::msg::Int32>("wheel1/alarm_code", 10);
        pub_alarm2_ = this->create_publisher<std_msgs::msg::Int32>("wheel2/alarm_code", 10);

        // Subscribers
        sub_speed_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "cmd_speed", 10, std::bind(&MoonsNode::speedCb, this, std::placeholders::_1));
        sub_jog_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_jog", 10, std::bind(&MoonsNode::jogCb, this, std::placeholders::_1));
        sub_encoder_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "cmd_encoder", 10, std::bind(&MoonsNode::encoderCb, this, std::placeholders::_1));
        sub_reset_alarm_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_reset_alarm", 10, std::bind(&MoonsNode::resetAlarmCb, this, std::placeholders::_1));

        gear_ratio_ = 40;
        prev_enc1_ = 0;
        prev_enc2_ = 0;

        RCLCPP_INFO(this->get_logger(), "MoonsNode initialized.");
    }

private:
    std::shared_ptr<Moons> moons_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_enc1_, pub_enc2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_vel1_, pub_vel2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cur1_, pub_cur2_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_alarm1_, pub_alarm2_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_speed_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_jog_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_encoder_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset_alarm_;

    int32_t prev_enc1_, prev_enc2_;
    int gear_ratio_;

    void publishData() {
        // Read raw encoder counts
        auto encs = moons_->getEncoder(prev_enc1_, prev_enc2_);
        prev_enc1_ = encs.first;
        prev_enc2_ = encs.second;

        // Read raw speed (counts/sec)
        auto vels = moons_->getSpeed();

        // Read current (Amps)
        auto cur = moons_->getCurrent();

        // Read alarms (list of codes)
        auto alarms1 = moons_->readAlarm(1);
        auto alarms2 = moons_->readAlarm(2);

        // Convert encoder counts to degrees
        double enc1_deg = (encs.first * 360.0) / (10000.0 * gear_ratio_);
        double enc2_deg = (encs.second * 360.0) / (10000.0 * gear_ratio_);

        // Convert speed counts/sec to degrees/sec
        double vel1_deg = (360.0 * vels.first) / (240.0 * gear_ratio_);
        double vel2_deg = (360.0 * vels.second) / (240.0 * gear_ratio_);

        // Publish encoder
        std_msgs::msg::Int32 msg_enc;
        msg_enc.data = static_cast<int>(enc1_deg);
        pub_enc1_->publish(msg_enc);
        msg_enc.data = static_cast<int>(enc2_deg);
        pub_enc2_->publish(msg_enc);

        // Publish speed
        std_msgs::msg::Float32 msg_vel;
        msg_vel.data = static_cast<float>(vel1_deg);
        pub_vel1_->publish(msg_vel);
        msg_vel.data = static_cast<float>(vel2_deg);
        pub_vel2_->publish(msg_vel);

        // Publish currents
        msg_vel.data = static_cast<float>(cur.first);
        pub_cur1_->publish(msg_vel);
        msg_vel.data = static_cast<float>(cur.second);
        pub_cur2_->publish(msg_vel);

        // Publish alarms as bitmask
        std_msgs::msg::Int32 msg_alarm;
        int bitmask1 = 0;
        for (int i : alarms1) bitmask1 |= (1 << i);
        msg_alarm.data = bitmask1;
        pub_alarm1_->publish(msg_alarm);

        int bitmask2 = 0;
        for (int i : alarms2) bitmask2 |= (1 << i);
        msg_alarm.data = bitmask2;
        pub_alarm2_->publish(msg_alarm);
    }

    // Callbacks
    void speedCb(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        double vel_left  = msg->data[0]; // deg/s
        double vel_right = msg->data[1]; // deg/s

        // Convert to motor counts
        long int deg_left  = round((vel_left  / 360.0) * 240.0 * gear_ratio_);
        long int deg_right = round((vel_right / 360.0) * 240.0 * gear_ratio_);

        // Send to both motors in one command
        moons_->setSpeed(0, deg_left, deg_right); // 0 = both motors

        // Publish all updated data
        publishData();
    }

    void jogCb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) moons_->startJog();
        else moons_->stopJog();
        publishData();
    }

    void encoderCb(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        moons_->setEncoder({msg->data[0], msg->data[1]});
        publishData();
    }

    void resetAlarmCb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) moons_->resetAlarm();
        publishData();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoonsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
