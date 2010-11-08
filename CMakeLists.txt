cmake_minimum_required( VERSION 2.8 )
PROJECT( eposcontrol )

INCLUDE_DIRECTORIES(
    /usr/local/include/canfestival
    ${PROJECT_SOURCE_DIR}/include )

#ADD_EXECUTABLE( eposcontrol 
#    src/main.cpp
#    src/EPOSMaster.c )

ADD_LIBRARY( EPOSControl 
    src/CANFestivalInterface.cpp
    src/CANChannel.cpp
    src/EPOSControl.cpp
    src/ObjDict/EPOSObjectDict_1.c
    src/ObjDict/EPOSObjectDict_2.c )

ADD_EXECUTABLE( simple 
    examples/simple/simple.cpp )

TARGET_LINK_LIBRARIES( simple 
    EPOSControl
    pcan
    canfestival_unix 
    canfestival
    pthread rt dl )

