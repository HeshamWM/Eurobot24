
#ifndef __DEBICT_EUROBOT_HARDWARE__EUROBOT_HARDWARE_COMPILER_H__
#define __DEBICT_EUROBOT_HARDWARE__EUROBOT_HARDWARE_COMPILER_H__

#if defined _WIN32 || defined __CYGWIN__
#   ifdef __GNUC__
#       define DEBICT_EUROBOT_HARDWARE_EXPORT __attribute__((dllexport))
#       define DEBICT_EUROBOT_HARDWARE_IMPORT __attribute__((dllimport))
#   else
#       define DEBICT_EUROBOT_HARDWARE_EXPORT __declspec(dllexport)
#       define DEBICT_EUROBOT_HARDWARE_IMPORT __declspec(dllimport)
#   endif
#   ifdef DEBICT_EUROBOT_HARDWARE_BUILDING_DLL
#       define DEBICT_EUROBOT_HARDWARE_PUBLIC DEBICT_EUROBOT_HARDWARE_EXPORT
#   else
#       define DEBICT_EUROBOT_HARDWARE_PUBLIC DEBICT_EUROBOT_HARDWARE_IMPORT
#   endif
#   define DEBICT_EUROBOT_HARDWARE_PUBLIC_TYPE DEBICT_EUROBOT_HARDWARE_PUBLIC
#   define DEBICT_EUROBOT_HARDWARE_LOCAL
#else
#   define DEBICT_EUROBOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#   define DEBICT_EUROBOT_HARDWARE_IMPORT
#   if __GNUC__ >= 4
#       define DEBICT_EUROBOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#       define DEBICT_EUROBOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#   else
#       define DEBICT_EUROBOT_HARDWARE_PUBLIC
#       define DEBICT_EUROBOT_HARDWARE_LOCAL
#   endif
#   define DEBICT_EUROBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif // __DEBICT_EUROBOT_HARDWARE__EUROBOT_HARDWARE_COMPILER_H__