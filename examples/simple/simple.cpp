//------------------------------------------------------------------------------
// File: simple.cpp
// Desc: A bare bones example of using the EPOSControl library
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "EPOSControl/EPOSControl.h"

//------------------------------------------------------------------------------
#if !defined(WIN32) || defined(__CYGWIN__)
void catchSignal( int sig )
{
    signal( SIGTERM, catchSignal );
    signal( SIGINT, catchSignal );
    printf( "Got Signal %d\n",sig );
}
#endif

//------------------------------------------------------------------------------
int main()
{
#if !defined(WIN32) || defined(__CYGWIN__)
    // install signal handler for manual break
    signal( SIGTERM, catchSignal );
    signal( SIGINT, catchSignal );
#endif
    
    // Start up the EPOSControl library
    if ( !EPOS_InitLibrary() )
    {
        fprintf( stderr, "Error: Unable to open EPOSControl library\n" );
        return -1;
    }
    
    CANChannel* pChannel = EPOS_OpenCANChannel( "libCanUSBDriver.so", "32", eBR_1M );
    if ( NULL == pChannel )
    {
        fprintf( stderr, "Error: Unable to open CAN bus channel\n" );
        return -1;
    }

    // Wait for Ctrl-C
    pause();
    
    // Shut down the EPOSControl library
    EPOS_CloseCANChannel( pChannel );
    EPOS_DeinitLibrary();
    
    return 0;
}
