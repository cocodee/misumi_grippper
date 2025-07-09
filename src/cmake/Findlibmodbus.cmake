# Findlibmodbus.cmake
# - Try to find libmodbus
#
# This module defines:
#  LIBMODBUS_FOUND - True if libmodbus was found
#  LIBMODBUS_INCLUDE_DIRS - The include directories for libmodbus
#  LIBMODBUS_LIBRARIES - The libraries to link against for libmodbus

find_path(LIBMODBUS_INCLUDE_DIR modbus.h
    HINTS
    /usr/include
    /usr/local/include
    /opt/local/include
    /sw/include
    PATH_SUFFIXES modbus
)

find_library(LIBMODBUS_LIBRARY NAMES modbus
    HINTS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
)

set(LIBMODBUS_INCLUDE_DIRS ${LIBMODBUS_INCLUDE_DIR})
set(LIBMODBUS_LIBRARIES ${LIBMODBUS_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libmodbus DEFAULT_MSG
    LIBMODBUS_LIBRARY LIBMODBUS_INCLUDE_DIR
)

mark_as_advanced(LIBMODBUS_INCLUDE_DIR LIBMODBUS_LIBRARY)

if(LIBMODBUS_FOUND AND NOT TARGET libmodbus::modbus)
    add_library(libmodbus::modbus UNKNOWN IMPORTED)
    set_target_properties(libmodbus::modbus PROPERTIES
        IMPORTED_LOCATION "${LIBMODBUS_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBMODBUS_INCLUDE_DIRS}"
    )
endif()