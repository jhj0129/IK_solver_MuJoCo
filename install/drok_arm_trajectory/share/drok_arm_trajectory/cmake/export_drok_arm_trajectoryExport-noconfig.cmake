#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "drok_arm_trajectory::drok_arm_trajectory" for configuration ""
set_property(TARGET drok_arm_trajectory::drok_arm_trajectory APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(drok_arm_trajectory::drok_arm_trajectory PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdrok_arm_trajectory.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS drok_arm_trajectory::drok_arm_trajectory )
list(APPEND _IMPORT_CHECK_FILES_FOR_drok_arm_trajectory::drok_arm_trajectory "${_IMPORT_PREFIX}/lib/libdrok_arm_trajectory.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
