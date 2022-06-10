#ifndef VESC_DRIVER__VISIBILITY_H_
#define VESC_DRIVER__VISIBILITY_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define VESC_DRIVER_EXPORT __attribute__((dllexport))
#define VESC_DRIVER_IMPORT __attribute__((dllimport))
#else
#define VESC_DRIVER_EXPORT __declspec(dllexport)
#define VESC_DRIVER_IMPORT __declspec(dllimport)
#endif

#ifdef VESC_DRIVER_DLL
#define VESC_DRIVER_PUBLIC VESC_DRIVER_EXPORT
#else
#define VESC_DRIVER_PUBLIC VESC_DRIVER_IMPORT
#endif

#define VESC_DRIVER_PUBLIC_TYPE VESC_DRIVER_PUBLIC

#define VESC_DRIVER_LOCAL

#else

#define VESC_DRIVER_EXPORT __attribute__((visibility("default")))
#define VESC_DRIVER_IMPORT

#if __GNUC__ >= 4
#define VESC_DRIVER_PUBLIC __attribute__((visibility("default")))
#define VESC_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define VESC_DRIVER_PUBLIC
#define VESC_DRIVER_LOCAL
#endif

#define VESC_DRIVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif // VESC_DRIVER__VISIBILITY_H_
