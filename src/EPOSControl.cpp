//------------------------------------------------------------------------------
// File: EPOSControl.cpp
// Desc: A library for communicating with and configuring EPOS motor controllers
//       made by Maxon motors using CAN bus.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#include "EPOSControl/EPOSControl.h"
#include "CANOpenInterface.h"

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Library globals
//------------------------------------------------------------------------------
static bool gbInitialised = false;
static CANChannel gCANChannels[ MAX_NUM_CAN_CHANNELS ];
static bool gbChannelInUse[ MAX_NUM_CAN_CHANNELS ] = { false };

//------------------------------------------------------------------------------
bool EPOS_InitLibrary()
{
    if ( !gbInitialised )
    {
        if ( !COI_InitCANOpenInterface() )
        {
            goto Finished;
        }
        gbInitialised = true;
    }
    
Finished:
    return gbInitialised;
}

//------------------------------------------------------------------------------
void EPOS_DeinitLibrary()
{
    for ( S32 channelIdx = 0; channelIdx < MAX_NUM_CAN_CHANNELS; channelIdx++ )
    {
        gCANChannels[ channelIdx ].Deinit();
        gbChannelInUse[ channelIdx ] = false;
    }
}

//------------------------------------------------------------------------------
CANChannel* EPOS_OpenCANChannel( const char* driverLibraryName, const char* canDevice,
                                 eBaudRate baudRate, S32 channelIdx )
{
    CANChannel* pResult = NULL;
    
    if ( -1 == channelIdx )
    {
        for ( S32 i = 0; i < MAX_NUM_CAN_CHANNELS; i++ )
        {
            if ( !gbChannelInUse[ i ] )
            {
                channelIdx = i;
                break;          // Found a free channel
            }
        }
    }
    
    if ( -1 == channelIdx )
    {
        fprintf( stderr, "Error: No free slot for channel\n" );
    }
    else if ( gbChannelInUse[ channelIdx ] )
    {
        fprintf( stderr, "Error: Slot %i already in use\n", channelIdx );
    }

    if ( gCANChannels[ channelIdx ].Init( driverLibraryName, canDevice, baudRate, channelIdx + 1 ) )
    {
        // A channel has been found and initialised
        pResult = &gCANChannels[ channelIdx ];
        gbChannelInUse[ channelIdx ] = true;
    }

    return pResult;
}

//------------------------------------------------------------------------------
void EPOS_CloseCANChannel( CANChannel* pChannel )
{
    for ( S32 channelIdx = 0; channelIdx < MAX_NUM_CAN_CHANNELS; channelIdx++ )
    {
        if ( &gCANChannels[ channelIdx ] == pChannel )
        {
            gCANChannels[ channelIdx ].Deinit();
            gbChannelInUse[ channelIdx ] = false;
            break;
        }
    }
}
