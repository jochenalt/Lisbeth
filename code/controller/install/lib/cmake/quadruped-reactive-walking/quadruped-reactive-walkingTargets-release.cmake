#----------------------------------------------------------------
# Generated CMake target import file for configuration "RELEASE".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "quadruped-reactive-walking::quadruped-reactive-walking" for configuration "RELEASE"
set_property(TARGET quadruped-reactive-walking::quadruped-reactive-walking APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(quadruped-reactive-walking::quadruped-reactive-walking PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libquadruped-reactive-walking.so.UNKNOWN"
  IMPORTED_SONAME_RELEASE "libquadruped-reactive-walking.so.UNKNOWN"
  )

list(APPEND _IMPORT_CHECK_TARGETS quadruped-reactive-walking::quadruped-reactive-walking )
list(APPEND _IMPORT_CHECK_FILES_FOR_quadruped-reactive-walking::quadruped-reactive-walking "${_IMPORT_PREFIX}/lib/libquadruped-reactive-walking.so.UNKNOWN" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
