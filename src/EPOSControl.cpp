//------------------------------------------------------------------------------
// File: EPOSControl.cpp
// Desc: A library for communicating with and configuring EPOS motor controllers
//       made by Maxon motors using CAN bus.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <stdlib.h>

#include "EPOSControl/EPOSControl.h"
#include "CANFestivalInterface.h"

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
bool EPOS_InitLibaray()
{
    if ( !gbInitialised )
    {
        if ( !CFI_InitCANFestivalInterface() )
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
CANChannel* EPOS_OpenCANChannel( const char* canDevice, eBaudRate baudRate )
{
    CANChannel* pResult = NULL;
    
    // Look for a free channel
    for ( S32 channelIdx = 0; channelIdx < MAX_NUM_CAN_CHANNELS; channelIdx++ )
    {
        if ( !gbChannelInUse[ channelIdx ] )
        {
            if ( gCANChannels[ channelIdx ].Init( canDevice, baudRate ) )
            {
                // A free channel has been found and initialised
                pResult = &gCANChannels[ channelIdx ];
                gbChannelInUse[ channelIdx ] = true;
                break;
            }
        }
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