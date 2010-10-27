cmake_minimum_required( VERSION 2.8 )
PROJECT( eposcontrol )

INCLUDE_DIRECTORIES( /usr/local/include/canfestival )

ADD_EXECUTABLE( eposcontrol 
    src/main.cpp
    src/EPOSMaster.c )

TARGET_LINK_LIBRARIES( eposcontrol pcan
    canfestival_unix 
    canfestival
    pthread rt dl )

