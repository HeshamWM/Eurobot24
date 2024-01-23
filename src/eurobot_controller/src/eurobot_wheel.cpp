// Ce code définit une classe appelée `EurobotWheel`, qui représente une roue d'un robot. Voici ce que fait le code en gros :

// 1. Inclusion des Bibliothèques :
//   - Le code inclut les bibliothèques nécessaires, notamment celles liées à l'interface matérielle de ROS 2.

// 2. Déclaration de l'Espace de Noms :
//   - Le code utilise l'espace de noms `debict::eurobot::controller` pour simplifier l'utilisation des classes de ce namespace.

// 3. Définition du Constructeur :
//    - Le constructeur de la classe `EurobotWheel` est défini.
//    - Il prend en paramètres des références à des interfaces d'état (`position_state` et `velocity_state`) et une référence à une interface de commande (`velocity_command`).
//    - Ces références sont utilisées pour initialiser des membres de la classe (`position_state_`, `velocity_state_`, et `velocity_command_`).

// 4. Définition de la Méthode `set_velocity` :
//    - La classe `EurobotWheel` a une méthode nommée `set_velocity`.
//    - Cette méthode prend en paramètre une valeur de vitesse (`value`).
//    - Elle utilise l'interface de commande (`velocity_command_`) pour définir la valeur de la vitesse de la roue.

// En résumé, la classe `EurobotWheel` encapsule les informations et les actions associées à une roue du robot. Le constructeur initialise la classe avec des références aux interfaces d'état et de commande, et la méthode `set_velocity` permet de définir la vitesse de la roue en utilisant l'interface de commande.

#include "eurobot_controller/eurobot_wheel.hpp"

// Utilisation de l'espace de noms pour simplifier l'utilisation des classes
using namespace debict::eurobot::controller;

// Définition du constructeur de la classe EurobotWheel
EurobotWheel::EurobotWheel(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command
    )
    : position_state_(position_state)
    , velocity_state_(velocity_state)
    , velocity_command_(velocity_command)
{
    // Initialisation des membres de la classe avec les références fournies en paramètres du constructeur
}

// Définition de la méthode pour définir la vitesse de la roue
void EurobotWheel::set_velocity(double value) // fonction set_velocity de classe EurobotWheel
{
    // Utilisation de l'interface de commande pour définir la valeur de la vitesse de la roue
    velocity_command_.get().set_value(value);
}
