cmake_minimum_required( VERSION 2.8 )
PROJECT( eposcontrol )

SET( CMAKE_CXX_FLAGS "-fPIC" )

# Setup installation settings for library RPATHs
# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

#-------------------------------------------------------------------------------

FIND_PACKAGE( PythonInterp )
FIND_PACKAGE( PythonLibs )

EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} --version
              ERROR_VARIABLE PYTHON_VERSION_FULL
              OUTPUT_STRIP_TRAILING_WHITESPACE)

STRING(REGEX MATCH "[0-9].[0-9]" PYTHON_VERSION_MAJOR_MINOR "${PYTHON_VERSION_FULL}")
IF(UNIX)
    SET(PYTHON_PLUGIN_INSTALL_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages/EPOSControl)
    SET(PYTHON_PACKAGES_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages)
ENDIF()
IF(WIN32)
    GET_FILENAME_COMPONENT(PYTHON_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION_MAJOR_MINOR}\\InstallPath]" ABSOLUTE CACHE)
    SET(PYTHON_PLUGIN_INSTALL_PATH "${PYTHON_PATH}/Lib/site-packages/EPOSControl")
    SET(PYTHON_PACKAGES_PATH "${PYTHON_PATH}/Lib/site-packages")
ENDIF()

INCLUDE_DIRECTORIES(
    /usr/local/include/canfestival
    ${CAN_OPEN_MASTER_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PYTHON_INCLUDE_DIR} )

LINK_DIRECTORIES(
    ${CAN_OPEN_MASTER_LIB_DIR} )

#-------------------------------------------------------------------------------
# EPOSControl library
#-------------------------------------------------------------------------------
SET( EPOSControlFiles 
    src/CANOpenInterface.cpp
    src/CANChannel.cpp
    src/EPOSControl.cpp
    src/CANMotorControllerAction.cpp
    src/CANMotorController.cpp
    src/SDOField.cpp
    ) 

ADD_LIBRARY( EPOSControl ${EPOSControlFiles} )
#TARGET_LINK_LIBRARIES( EPOSControl CanOpenMaster )

ADD_LIBRARY( EPOSControlShared SHARED ${EPOSControlFiles} )
#TARGET_LINK_LIBRARIES( EPOSControlShared CanOpenMaster )
SET_TARGET_PROPERTIES( EPOSControlShared 
    PROPERTIES OUTPUT_NAME EPOSControl )

INSTALL( TARGETS EPOSControl
        ARCHIVE DESTINATION lib )
INSTALL( TARGETS EPOSControlShared
        LIBRARY DESTINATION lib )

INSTALL( DIRECTORY ${PROJECT_SOURCE_DIR}/include/EPOSControl DESTINATION include
          FILES_MATCHING PATTERN "*.h" )

#-------------------------------------------------------------------------------
# Python interface
#-------------------------------------------------------------------------------
ADD_LIBRARY( EPOSControlPython SHARED interfaces/python/PyEPOSControl.cpp )
SET_TARGET_PROPERTIES( EPOSControlPython 
    PROPERTIES OUTPUT_NAME PyEPOSControl )
TARGET_LINK_LIBRARIES( EPOSControlPython
    EPOSControl
    CanOpenMaster
    boost_thread
    )
SET_TARGET_PROPERTIES( EPOSControlPython PROPERTIES PREFIX "" )
SET_TARGET_PROPERTIES( EPOSControlPython PROPERTIES LINK_FLAGS ${CAN_OPEN_MASTER_LINK_FLAGS} )
INSTALL( TARGETS EPOSControlPython
        LIBRARY DESTINATION ${PYTHON_PLUGIN_INSTALL_PATH} )

#-------------------------------------------------------------------------------
# Example application
#-------------------------------------------------------------------------------
ADD_EXECUTABLE( simple 
    examples/simple/simple.cpp )

TARGET_LINK_LIBRARIES( simple 
    EPOSControl
    CanOpenMaster
    boost_thread
    )

INSTALL( TARGETS simple
        RUNTIME DESTINATION bin )
