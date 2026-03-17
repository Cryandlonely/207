#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "MAVFTP_Pro::mavftp_pro" for configuration "Release"
set_property(TARGET MAVFTP_Pro::mavftp_pro APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(MAVFTP_Pro::mavftp_pro PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmavftp_pro.a"
  )

list(APPEND _cmake_import_check_targets MAVFTP_Pro::mavftp_pro )
list(APPEND _cmake_import_check_files_for_MAVFTP_Pro::mavftp_pro "${_IMPORT_PREFIX}/lib/libmavftp_pro.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
