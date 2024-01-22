
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "eurobot_controller/eurobot_drive_controller.hpp"

PLUGINLIB_EXPORT_CLASS(
    debict::eurobot::controller::EurobotDriveController,
    controller_interface::ControllerInterface
)

using namespace debict::eurobot::controller;

EurobotDriveController::EurobotDriveController()
    : controller_interface::ControllerInterface()
    , velocity_command_subsciption_(nullptr)
    , velocity_command_ptr_(nullptr)
{

}

controller_interface::InterfaceConfiguration EurobotDriveController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    RCLCPP_INFO(get_node()->get_logger(), "Configure EurobotDriveController");

    command_interfaces_config.names.push_back(roue_AvG_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(roue_AvD_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(roue_ArG_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(roue_ArD_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration EurobotDriveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.push_back(roue_AvG_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(roue_AvG_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(roue_AvD_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(roue_AvD_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(roue_ArG_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(roue_ArG_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(roue_ArD_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(roue_ArD_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return state_interfaces_config;
}

controller_interface::CallbackReturn EurobotDriveController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EurobotDriveController::update(
    [[maybe_unused]] const rclcpp::Time & time, 
    [[maybe_unused]] const rclcpp::Duration & period)
{
    // Get the last velocity command
    auto velocity_command = velocity_command_ptr_.readFromRT();
    if (!velocity_command || !(*velocity_command)) {
        return controller_interface::return_type::OK;
    }

    // Calculate the wheel velocity
    // See: http://robotsforroboticists.com/drive-kinematics/
    const auto twist = (*velocity_command)->twist;
    double roue_AvG_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double roue_AvD_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double roue_ArG_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double roue_ArD_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);

    roue_AvG_->set_velocity(roue_AvG_velocity);
    roue_AvD_->set_velocity(roue_AvD_velocity);
    roue_ArG_->set_velocity(roue_ArG_velocity);
    roue_ArD_->set_velocity(roue_ArD_velocity);

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn EurobotDriveController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Configure EurobotDriveController");

    roue_AvG_joint_name_ = get_node()->get_parameter("roue_AvG_joint_name").as_string();
    roue_AvD_joint_name_ = get_node()->get_parameter("roue_AvD_joint_name").as_string();
    roue_ArG_joint_name_ = get_node()->get_parameter("roue_ArG_joint_name").as_string();
    roue_ArD_joint_name_ = get_node()->get_parameter("roue_ArD_joint_name").as_string();
    if (roue_AvG_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'roue_AvG_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (roue_AvD_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'roue_AvD_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (roue_ArG_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'roue_ArG_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (roue_ArD_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'roue_ArD_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    wheel_distance_width_ = get_node()->get_parameter("wheel_distance.width").as_double();
    wheel_distance_length_ = get_node()->get_parameter("wheel_distance.length").as_double();
    if (wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_radius' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_distance_width_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_distance.width' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_distance_length_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_distance.length' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    wheel_separation_width_ = wheel_distance_width_ / 2;
    wheel_separation_length_ = wheel_distance_length_ / 2;

    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    velocity_command_subsciption_ = get_node()->create_subscription<Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS(), [this](const Twist::SharedPtr twist)
    {
        velocity_command_ptr_.writeFromNonRT(twist);
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EurobotDriveController::on_activate(const rclcpp_lifecycle::State &)
{
    // Initialize the wheels
    roue_AvG_ = get_wheel(roue_AvG_joint_name_);
    roue_AvD_ = get_wheel(roue_AvD_joint_name_);
    roue_ArG_ = get_wheel(roue_ArG_joint_name_);
    roue_ArD_ = get_wheel(roue_ArD_joint_name_);
    if (!roue_AvG_ || !roue_AvD_ || !roue_ArG_ || !roue_ArD_) {
        return controller_interface::CallbackReturn::ERROR;
    }
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EurobotDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EurobotDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EurobotDriveController::on_error(const rclcpp_lifecycle::State &)
{
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EurobotDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

std::shared_ptr<EurobotWheel> EurobotDriveController::get_wheel(const std::string & wheel_joint_name)
{
    // Lookup the position state interface
    const auto position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity command interface
    const auto velocity_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity command interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Create the wheel instance
    return std::make_shared<EurobotWheel>(
        std::ref(*position_state),
        std::ref(*velocity_state),
        std::ref(*velocity_command)
        );
}

bool EurobotDriveController::reset()
{
    subscriber_is_active_ = false;
    velocity_command_subsciption_.reset();

    roue_AvG_.reset();
    roue_AvD_.reset();
    roue_ArG_.reset();
    roue_ArD_.reset();

    return true;
}