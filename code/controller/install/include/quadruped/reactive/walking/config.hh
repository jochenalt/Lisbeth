/* 
 * This file has been automatically generated by the jrl-cmakemodules.
 * Please see https://github.com/jrl-umi3218/jrl-cmakemodules/blob/master/config.hh.cmake for details.
*/

#ifndef QUADRUPED_REACTIVE_WALKING_CONFIG_HH
# define QUADRUPED_REACTIVE_WALKING_CONFIG_HH

// Package version (header).
# define QUADRUPED_REACTIVE_WALKING_VERSION_UNKNOWN_TAG 0 // Used to mention that the current version is unknown.
# define QUADRUPED_REACTIVE_WALKING_VERSION "UNKNOWN"
# define QUADRUPED_REACTIVE_WALKING_MAJOR_VERSION QUADRUPED_REACTIVE_WALKING_VERSION_UNKNOWN_TAG
# define QUADRUPED_REACTIVE_WALKING_MINOR_VERSION QUADRUPED_REACTIVE_WALKING_VERSION_UNKNOWN_TAG
# define QUADRUPED_REACTIVE_WALKING_PATCH_VERSION QUADRUPED_REACTIVE_WALKING_VERSION_UNKNOWN_TAG

#define QUADRUPED_REACTIVE_WALKING_VERSION_AT_LEAST(major, minor, patch) (QUADRUPED_REACTIVE_WALKING_MAJOR_VERSION>major || (QUADRUPED_REACTIVE_WALKING_MAJOR_VERSION>=major && \
                                                             (QUADRUPED_REACTIVE_WALKING_MINOR_VERSION>minor || (QUADRUPED_REACTIVE_WALKING_MINOR_VERSION>=minor && \
                                                                                                     QUADRUPED_REACTIVE_WALKING_PATCH_VERSION>=patch))))

#define QUADRUPED_REACTIVE_WALKING_VERSION_AT_MOST(major, minor, patch) (QUADRUPED_REACTIVE_WALKING_MAJOR_VERSION<major || (QUADRUPED_REACTIVE_WALKING_MAJOR_VERSION<=major && \
                                                            (QUADRUPED_REACTIVE_WALKING_MINOR_VERSION<minor || (QUADRUPED_REACTIVE_WALKING_MINOR_VERSION<=minor && \
                                                                                                     QUADRUPED_REACTIVE_WALKING_PATCH_VERSION<=patch))))

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define QUADRUPED_REACTIVE_WALKING_DLLIMPORT __declspec(dllimport)
#  define QUADRUPED_REACTIVE_WALKING_DLLEXPORT __declspec(dllexport)
#  define QUADRUPED_REACTIVE_WALKING_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define QUADRUPED_REACTIVE_WALKING_DLLIMPORT __attribute__ ((visibility("default")))
#   define QUADRUPED_REACTIVE_WALKING_DLLEXPORT __attribute__ ((visibility("default")))
#   define QUADRUPED_REACTIVE_WALKING_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define QUADRUPED_REACTIVE_WALKING_DLLIMPORT
#   define QUADRUPED_REACTIVE_WALKING_DLLEXPORT
#   define QUADRUPED_REACTIVE_WALKING_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef QUADRUPED_REACTIVE_WALKING_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define QUADRUPED_REACTIVE_WALKING_DLLAPI
#  define QUADRUPED_REACTIVE_WALKING_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef quadruped_reactive_walking_EXPORTS
#   define QUADRUPED_REACTIVE_WALKING_DLLAPI QUADRUPED_REACTIVE_WALKING_DLLEXPORT
#  else
#   define QUADRUPED_REACTIVE_WALKING_DLLAPI QUADRUPED_REACTIVE_WALKING_DLLIMPORT
#  endif // QUADRUPED_REACTIVE_WALKING_EXPORTS
#  define QUADRUPED_REACTIVE_WALKING_LOCAL QUADRUPED_REACTIVE_WALKING_DLLLOCAL
# endif // QUADRUPED_REACTIVE_WALKING_STATIC
#endif //! QUADRUPED_REACTIVE_WALKING_CONFIG_HH
