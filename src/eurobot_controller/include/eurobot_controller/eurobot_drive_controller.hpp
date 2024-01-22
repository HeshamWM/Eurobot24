
#ifndef __DEBICT_EUROBOT_CONTROLLER__EUROBOT_DRIVE_CONTROLLER_H__
#define __DEBICT_EUROBOT_CONTROLLER__EUROBOT_DRIVE_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>

#include "eurobot_controller/eurobot_wheel.hpp"
#include "eurobot_controller/eurobot_controller_compiler.h"

namespace debict
{
    namespace eurobot
    {
        namespace controller
        {
            using Twist = geometry_msgs::msg::TwistStamped;

            class EurobotDriveController
                : public controller_interface::ControllerInterface
            {
            public:
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                EurobotDriveController();
                
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::InterfaceConfiguration command_interface_configuration() const override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::InterfaceConfiguration state_interface_configuration() const override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_init() override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
            
            protected:
                std::shared_ptr<EurobotWheel> get_wheel(const std::string & wheel_joint_name);

                bool reset();

            protected:
                rclcpp::Subscription<Twist>::SharedPtr velocity_command_subsciption_;
                realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;
                std::shared_ptr<EurobotWheel> roue_AvG_;
                std::shared_ptr<EurobotWheel> roue_AvD_;
                std::shared_ptr<EurobotWheel> roue_ArG_;
                std::shared_ptr<EurobotWheel> roue_ArD_;
                std::string roue_AvG_joint_name_;
                std::string roue_AvD_joint_name_;
                std::string roue_ArG_joint_name_;
                std::string roue_ArD_joint_name_;
                double linear_scale_;
                double radial_scale_;
                double wheel_radius_;
                double wheel_distance_width_;
                double wheel_distance_length_;
                double wheel_separation_width_;
                double wheel_separation_length_;
                bool subscriber_is_active_;

            };
        }
    }
}

#endif // __DEBICT_EUROBOT_CONTROLLER__EUROBOT_DRIVE_CONTROLLER_H__