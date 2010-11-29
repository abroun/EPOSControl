cmake_minimum_required( VERSION 2.8 )
PROJECT( eposcontrol )

INCLUDE_DIRECTORIES(
    /usr/local/include/canfestival
    ${PROJECT_SOURCE_DIR}/include )

#-------------------------------------------------------------------------------
# EPOSControl library
#-------------------------------------------------------------------------------
ADD_LIBRARY( EPOSControl 
    src/CANFestivalInterface.cpp
    src/CANChannel.cpp
    src/EPOSControl.cpp
    src/CANMotorControllerAction.cpp
    src/CANMotorController.cpp
    src/SDOField.cpp
    src/ObjDict/EPOSObjectDict_1.c
    src/ObjDict/EPOSObjectDict_2.c )

INSTALL( TARGETS EPOSControl
        ARCHIVE DESTINATION lib )

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