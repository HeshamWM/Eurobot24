
#ifndef __DEBICT_EUROBOT_CONTROLLER__EUROBOT_WHEEL_H__
#define __DEBICT_EUROBOT_CONTROLLER__EUROBOT_WHEEL_H__

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace debict
{
    namespace eurobot
    {
        namespace controller
        {
            class EurobotWheel
            {
            public:
                EurobotWheel(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command
                    );

                void set_velocity(double value);

            private:
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command_;

            };
        }
    }
}

#endif // __DEBICT_EUROBOT_CONTROLLER__EUROBOT_WHEEL_H__