cmake_minimum_required( VERSION 2.8 )
PROJECT( eposcontrol )

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
    ${PROJECT_SOURCE_DIR}/include
    ${PYTHON_INCLUDE_DIR} )

#-------------------------------------------------------------------------------
# EPOSControl library
#-------------------------------------------------------------------------------
SET( EPOSControlFiles 
    src/CANFestivalInterface.cpp
    src/CANChannel.cpp
    src/EPOSControl.cpp
    src/CANMotorControllerAction.cpp
    src/CANMotorController.cpp
    src/SDOField.cpp
    src/ObjDict/EPOSObjectDict_1.c
    src/ObjDict/EPOSObjectDict_2.c
    ) 

ADD_LIBRARY( EPOSControl ${EPOSControlFiles} )
ADD_LIBRARY( EPOSControlShared SHARED ${EPOSControlFiles} )
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
    pcan
    canfestival_unix 
    canfestival
    pthread rt dl )
SET_TARGET_PROPERTIES( EPOSControlPython PROPERTIES PREFIX "" )
INSTALL( TARGETS EPOSControlPython
        LIBRARY DESTINATION ${PYTHON_PLUGIN_INSTALL_PATH} )

#-------------------------------------------------------------------------------
# Example application
#-------------------------------------------------------------------------------
ADD_EXECUTABLE( simple 
    examples/simple/simple.cpp )

TARGET_LINK_LIBRARIES( simple 
    EPOSControl
    pcan
    canfestival_unix 
    canfestival
    pthread rt dl )

INSTALL( TARGETS simple
        RUNTIME DESTINATION bin )