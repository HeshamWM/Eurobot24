// control gère les controleurs 

// Ce code représente le point d'entrée principal d'une application ROS 2 destinée à la gestion des contrôleurs d'un robot. 
// Voici une description générale de ce que fait le code :

// 1. Initialisation de ROS 2 :
//    - Le code commence par initialiser le système ROS 2 à l'aide de `rclcpp::init(argc, argv);`.

// 2. Configuration du Gestionnaire de Contrôleurs :
//    - Un nom de nœud pour le gestionnaire de contrôleurs est défini comme "controller_manager_node_name".
//    - Un exécuteur ROS 2 multithreadé est créé à l'aide de `std::make_shared<rclcpp::executors::MultiThreadedExecutor>();`.
//    - Un nœud de gestionnaire de contrôleurs est créé avec le nom défini et le chemin du fichier de description du robot 
//      (~/"robot_description").

// 3. Affichage de la Fréquence de Mise à Jour :
//    - La fréquence de mise à jour du gestionnaire de contrôleurs est affichée.

// 4. Configuration du Thread de Contrôle :
//    - Un thread (`cm_thread`) est créé pour exécuter la boucle de contrôle du gestionnaire de contrôleurs.
//    - Si un noyau temps réel est détecté, la planification en temps réel est configurée avec une priorité spécifiée (`kSchedPriority`).
//    - La boucle principale du thread effectue les opérations suivantes en boucle :
//       - Calcule la période mesurée de la boucle.
//       - Exécute les étapes de lecture, mise à jour et écriture des contrôleurs.
//       - Calcule le prochain moment d'itération et met le thread en sommeil jusqu'à ce moment.

// 5. Chargement et Configuration des Contrôleurs :
//    - Le code charge les contrôleurs spécifiés (ici, "joint_state_controller" et "eurobot_drive_controller").
//    - Il configure ensuite ces contrôleurs.

// 6. Démarrage des Contrôleurs :
//    - Le code spécifie les contrôleurs à démarrer ("joint_state_controller" et "eurobot_drive_controller").
//    - Il utilise le service `switch_controller` pour démarrer les contrôleurs avec l'effort maximal.

// 7. Exécution de l'Exécuteur :
//    - Le nœud du gestionnaire de contrôleurs est ajouté à l'exécuteur.
//    - L'exécuteur est exécuté, faisant fonctionner le gestionnaire de contrôleurs.

// 8. Arrêt de ROS 2 :
//    - Après l'arrêt de la boucle principale, ROS 2 est arrêté avec `rclcpp::shutdown();`.


// Globalement, ce code configure et lance un gestionnaire de contrôleurs multithreadé dans un environnement ROS 2. 
// Ce gestionnaire prend en charge la gestion des contrôleurs du robot, en effectuant des opérations de lecture, 
// mise à jour et écriture à une fréquence spécifiée. Les contrôleurs spécifiés sont chargés, configurés et démarrés, 
// et l'ensemble du processus est orchestré par un exécuteur ROS 2.

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <realtime_tools/thread_priority.hpp>

// Définition de la priorité pour la planification en temps réel
int const kSchedPriority = 50;

int main(int argc, char * argv[])
{
    // Initialisation de ROS 2
    rclcpp::init(argc, argv);

    // Configuration du nom du nœud du gestionnaire de contrôleurs
    std::string controller_manager_node_name = "controller_manager";

    // Création d'un exécuteur ROS 2 multithreadé
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Création du nœud du gestionnaire de contrôleurs
    auto controller_manager_node = std::make_shared<controller_manager::ControllerManager>(executor, controller_manager_node_name, "~/robot_description");
    
    // Affichage de la fréquence de mise à jour du gestionnaire de contrôleurs
    RCLCPP_INFO(controller_manager_node->get_logger(), "update rate is %d Hz", controller_manager_node->get_update_rate());

    // Création d'un thread dédié pour la boucle de contrôle du gestionnaire de contrôleurs
    std::thread cm_thread([controller_manager_node]() {
        // Configuration de la planification en temps réel si un noyau temps réel est détecté
        if (realtime_tools::has_realtime_kernel()) {
            if (!realtime_tools::configure_sched_fifo(kSchedPriority)) {
                // Avertissement si la configuration de la planification en temps réel échoue
                RCLCPP_WARN(controller_manager_node->get_logger(), "Could not enable FIFO RT scheduling policy");
            }
        } else {
            // Affichage d'un message d'information si un noyau temps réel n'est pas détecté
            RCLCPP_INFO(controller_manager_node->get_logger(), "RT kernel is recommended for better performance");
        }

        // Configuration des variables pour le calcul du temps de sommeil
        auto const period = std::chrono::nanoseconds(1'000'000'000 / controller_manager_node->get_update_rate());
        auto const cm_now = std::chrono::nanoseconds(controller_manager_node->now().nanoseconds());
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{cm_now};

        // Configuration des variables pour le calcul de la période mesurée de la boucle
        rclcpp::Time previous_time = controller_manager_node->now();

        // Boucle principale du thread
        while (rclcpp::ok()) {
            // Calcul de la période mesurée
            auto const current_time = controller_manager_node->now();
            auto const measured_period = current_time - previous_time;
            previous_time = current_time;

            // Exécution des étapes de lecture, mise à jour et écriture des contrôleurs
            controller_manager_node->read(controller_manager_node->now(), measured_period);
            controller_manager_node->update(controller_manager_node->now(), measured_period);
            controller_manager_node->write(controller_manager_node->now(), measured_period);

            // Calcul du prochain moment d'itération
            next_iteration_time += period;
            std::this_thread::sleep_until(next_iteration_time);
        }
    });

    // Chargement et configuration des contrôleurs
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    controller_manager_node->load_controller("joint_state_controller", "joint_state_controller/JointStateController");
    controller_manager_node->load_controller("eurobot_drive_controller", "eurobot_controller/EurobotDriveController");
    controller_manager_node->configure_controller("joint_state_controller");
    controller_manager_node->configure_controller("eurobot_drive_controller");

    // Démarrage des contrôleurs spécifiés
    start_controllers.push_back("joint_state_controller");
    start_controllers.push_back("eurobot_drive_controller");
    controller_manager_node->switch_controller(start_controllers, stop_controllers, 1, controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);

    // Ajout du nœud du gestionnaire de contrôleurs à l'exécuteur
    executor->add_node(controller_manager_node);

    // Exécution de l'exécuteur pour faire fonctionner le nœud du gestionnaire de contrôleurs
    executor->spin();

    // Arrêt de ROS 2
    rclcpp::shutdown();

    // Retour de la fonction main
    return 0;
}
