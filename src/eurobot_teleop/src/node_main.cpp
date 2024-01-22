
#include <rclcpp/rclcpp.hpp>
#include "eurobot_teleop/eurobot_teleop.hpp"
#include "eurobot_teleop/eurobot_joystick.hpp"

int main(int argc, char* argv[])
{
    // Initialize the application
    rclcpp::init(argc, argv);

    // Setup the executor
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Create the joystick node
    auto teleop   = std::make_shared<debict::eurobot::teleop::EurobotTeleop>("teleop");
    auto joystick = std::make_shared<debict::eurobot::teleop::EurobotJoystick>("joystick");

    // Run the node(s)
    executor->add_node(teleop);
    executor->add_node(joystick);
    executor->spin();

    // Exit
    rclcpp::shutdown();
    return 0;
}