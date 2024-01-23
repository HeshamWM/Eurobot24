#ifndef __DEBICT_EUROBOT_CONTROLLER__EUROBOT_CONTROLLER_COMPILER_H__
#define __DEBICT_EUROBOT_CONTROLLER__EUROBOT_CONTROLLER_COMPILER_H__

// Les directives de préprocesseur ci-dessus sont des gardiens d'inclusion
// qui évitent l'inclusion multiple de ce fichier dans le même fichier source.

// Cette macro est utilisée pour indiquer que le symbole doit être visible à l'extérieur
// de la bibliothèque ou du module lorsqu'elle est utilisée comme bibliothèque partagée.
#define DEBICT_EUROBOT_CONTROLLER_EXPORT __attribute__((visibility("default")))

// Cette macro est utilisée pour indiquer que le symbole doit être visible à l'intérieur
// de la bibliothèque ou du module lorsqu'elle est utilisée comme bibliothèque partagée.
#define DEBICT_EUROBOT_CONTROLLER_IMPORT

// Cette macro est utilisée pour indiquer que le symbole doit être visible à l'extérieur
// de la bibliothèque ou du module lorsqu'elle est utilisée comme bibliothèque partagée.
#define DEBICT_EUROBOT_CONTROLLER_PUBLIC __attribute__((visibility("default")))

// Cette macro est utilisée pour indiquer que le symbole doit être visible uniquement
// à l'intérieur de la bibliothèque ou du module lorsqu'elle est utilisée comme bibliothèque partagée.
#define DEBICT_EUROBOT_CONTROLLER_LOCAL __attribute__((visibility("hidden")))

// Cette macro semble être réservée pour une utilisation future, peut-être pour indiquer
// la visibilité d'un type spécifique. Actuellement, elle est définie sans action.
#define DEBICT_EUROBOT_CONTROLLER_PUBLIC_TYPE

#endif // __DEBICT_EUROBOT_CONTROLLER__EUROBOT_CONTROLLER_COMPILER_H__
