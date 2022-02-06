#----------------------------------------------------------------
# Generated CMake target import file for configuration "RELEASE".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "quadruped-walkgen::quadruped-walkgen" for configuration "RELEASE"
set_property(TARGET quadruped-walkgen::quadruped-walkgen APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(quadruped-walkgen::quadruped-walkgen PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libquadruped-walkgen.so.1.0.0"
  IMPORTED_SONAME_RELEASE "libquadruped-walkgen.so.1.0.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS quadruped-walkgen::quadruped-walkgen )
list(APPEND _IMPORT_CHECK_FILES_FOR_quadruped-walkgen::quadruped-walkgen "${_IMPORT_PREFIX}/lib/libquadruped-walkgen.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
