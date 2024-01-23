
#ifndef __DEBICT_EUROBOT_CONTROLLER__EUROBOT_DRIVE_CONTROLLER_H__
#define __DEBICT_EUROBOT_CONTROLLER__EUROBOT_DRIVE_CONTROLLER_H__

// Les directives de préprocesseur ci-dessus sont des gardiens d'inclusion
// qui évitent l'inclusion multiple de ce fichier dans le même fichier source.

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>

#include "eurobot_controller/eurobot_wheel.hpp"
#include "eurobot_controller/eurobot_controller_compiler.h"

// Espace de noms pour la bibliothèque Eurobot Controller
namespace debict
{
    namespace eurobot
    {
        namespace controller
        {
            // Alias pour le type TwistStamped de geometry_msgs
            using Twist = geometry_msgs::msg::TwistStamped;

            // Déclaration de la classe EurobotDriveController qui hérite de ControllerInterface
            class EurobotDriveController
                : public controller_interface::ControllerInterface
            {
            public:
                // Constructeur de la classe
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                EurobotDriveController();
                
                // Méthode pour configurer l'interface de commande
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::InterfaceConfiguration command_interface_configuration() const override;

                // Méthode pour configurer l'interface d'état
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::InterfaceConfiguration state_interface_configuration() const override;

                // Méthode pour la mise à jour du contrôleur
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

                // Méthode appelée lors de l'initialisation du contrôleur
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_init() override;

                // Méthode appelée lors de la configuration du contrôleur
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

                // Méthode appelée lors de l'activation du contrôleur
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

                // Méthode appelée lors de la désactivation du contrôleur
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

                // Méthode appelée lors du nettoyage du contrôleur
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

                // Méthode appelée en cas d'erreur du contrôleur
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

                // Méthode appelée lors de l'arrêt du contrôleur
                DEBICT_EUROBOT_CONTROLLER_PUBLIC
                controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
            
            protected:
                // Méthode pour obtenir un pointeur partagé vers une roue Eurobot spécifiée par son nom
                std::shared_ptr<EurobotWheel> get_wheel(const std::string & wheel_joint_name);

                // Méthode pour réinitialiser le contrôleur
                bool reset();

            protected:
                // Abonnement au topic de commande de vitesse (Twist)
                rclcpp::Subscription<Twist>::SharedPtr velocity_command_subsciption_;

                // Buffer en temps réel pour stocker la commande de vitesse
                realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;

                // Pointeurs partagés vers les roues du robot
                std::shared_ptr<EurobotWheel> roue_AvG_;
                std::shared_ptr<EurobotWheel> roue_AvD_;
                std::shared_ptr<EurobotWheel> roue_ArG_;
                std::shared_ptr<EurobotWheel> roue_ArD_;

                // Noms des joints des roues
                std::string roue_AvG_joint_name_;
                std::string roue_AvD_joint_name_;
                std::string roue_ArG_joint_name_;
                std::string roue_ArD_joint_name_;

                // Paramètres pour l'échelle linéaire, l'échelle radiale et les dimensions des roues
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