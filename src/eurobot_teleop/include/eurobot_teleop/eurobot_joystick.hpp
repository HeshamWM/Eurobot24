#ifndef __DEBICT_EUROBOT_TELEOP__EUROBOT_JOYSTICK_H__
#define __DEBICT_EUROBOT_TELEOP__EUROBOT_JOYSTICK_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/char.hpp> // Pour gérer les entrées du clavier
#include <vector>

namespace debict
{
    namespace eurobot
    {
        namespace teleop
        {
            class EurobotJoystick
                : public rclcpp::Node
            {
            public:
                EurobotJoystick(const std::string & name);
                ~EurobotJoystick();

            private:
                void update();
                void handleJoystickInput(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
                void handleKeyboardInput(const std_msgs::msg::Char::SharedPtr key_msg);

            private:
                rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
                rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_subscription_;
                rclcpp::TimerBase::SharedPtr update_timer_;
                std::vector<int> buttons_;
                std::vector<double> axes_;
                int device_handle_;
                double deadzone_;
                double scale_;
                double unscaled_deadzone_;

            };
        }
    }
}

#endif // __DEBICT_EUROBOT_TELEOP__EUROBOT_JOYSTICK_H__
