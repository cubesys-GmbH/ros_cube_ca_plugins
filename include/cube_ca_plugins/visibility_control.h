#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CUBE_CA_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define CUBE_CA_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define CUBE_CA_PLUGINS_EXPORT __declspec(dllexport)
    #define CUBE_CA_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CUBE_CA_PLUGINS_BUILDING_LIBRARY
    #define CUBE_CA_PLUGINS_PUBLIC CUBE_CA_PLUGINS_EXPORT
  #else
    #define CUBE_CA_PLUGINS_PUBLIC CUBE_CA_PLUGINS_IMPORT
  #endif
  #define CUBE_CA_PLUGINS_PUBLIC_TYPE CUBE_CA_PLUGINS_PUBLIC
  #define CUBE_CA_PLUGINS_LOCAL
#else
  #define CUBE_CA_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define CUBE_CA_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define CUBE_CA_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define CUBE_CA_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CUBE_CA_PLUGINS_PUBLIC
    #define CUBE_CA_PLUGINS_LOCAL
  #endif
  #define CUBE_CA_PLUGINS_PUBLIC_TYPE
#endif
