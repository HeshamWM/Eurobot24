#ifndef __DEBICT_EUROBOT_CONTROLLER__EUROBOT_WHEEL_H__
#define __DEBICT_EUROBOT_CONTROLLER__EUROBOT_WHEEL_H__

// Les directives de préprocesseur ci-dessus sont des gardiens d'inclusion
// qui évitent l'inclusion multiple de ce fichier dans le même fichier source.

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

// Espace de noms pour la bibliothèque Eurobot Controller
namespace debict
{
    namespace eurobot
    {
        namespace controller
        {
            // Déclaration de la classe EurobotWheel
            class EurobotWheel
            {
            public:
                // Constructeur de la classe prenant des interfaces d'état et de commande en paramètres
                EurobotWheel(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command
                    );

                // Méthode pour définir la vitesse de la roue
                void set_velocity(double value);

            private:
                // Interfaces d'état et de commande utilisées par la roue
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command_;

            };
        }
    }
}

#endif // __DEBICT_EUROBOT_CONTROLLER__EUROBOT_WHEEL_H__
