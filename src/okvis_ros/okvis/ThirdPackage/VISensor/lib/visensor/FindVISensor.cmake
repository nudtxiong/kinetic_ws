# - Config file for the OKVIS package
# It defines the following variables
#  OKVIS_INCLUDE_DIRS - include directories for FooBar
#  OKVIS_LIBRARIES    - libraries to link against
#  OKVIS_EXECUTABLE   - the okvis_app_synchronous executable
#  OKVIS_CERES_CONFIG - path to CeresConfig.cmake, to use find_package(ceres)

include(LibFindMacros)

set(VisensorPcDirectory "/home/nubot/kinetic_ws/src/okvis_ros/okvis/ThirdPackage/VISensor/share/pkgconfig")

SET(ENV{PKG_CONFIG_PATH} "${VisensorPcDirectory}")

libfind_pkg_check_modules(VISensorDriver_PKGCONF visensor) 

# Use pkg-config to get hints about paths
# Include dir
find_path(VISensorDriver_INCLUDE_DIR
  NAMES aslam_sensor_driver.hpp
  PATHS ${VISensorDriver_PKGCONF_INCLUDE_DIRS}
)


# Finally the library itself
find_library(VISensorDriver_LIBRARY
  NAMES libvisensor.so
  PATHS ${VISensorDriver_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(VISensorDriver_PROCESS_INCLUDES VISensorDriver_INCLUDE_DIR)
set(VISensorDriver_PROCESS_LIBS VISensorDriver_LIBRARY)
libfind_process(VISensorDriver)



