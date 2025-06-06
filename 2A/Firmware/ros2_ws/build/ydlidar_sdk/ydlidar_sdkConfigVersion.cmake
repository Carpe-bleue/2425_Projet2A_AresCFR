set(PACKAGE_VERSION "1.2.15")

# Check build type is valid
if( "System:${CMAKE_SYSTEM_NAME},Android:${ANDROID},iOS:${IOS}" STREQUAL
    "System:Linux,Android:,iOS:" )
    # Check whether the requested PACKAGE_FIND_VERSION is compatible
    if("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}")
      set(PACKAGE_VERSION_COMPATIBLE FALSE)
    else()
      set(PACKAGE_VERSION_COMPATIBLE TRUE)
      if ("${PACKAGE_VERSION}" VERSION_EQUAL "${PACKAGE_FIND_VERSION}")
        set(PACKAGE_VERSION_EXACT TRUE)
      endif()
    endif()
else()
    set(PACKAGE_VERSION_COMPATIBLE FALSE)
endif()
