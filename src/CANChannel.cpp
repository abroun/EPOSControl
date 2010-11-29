//------------------------------------------------------------------------------
// File: CANChannel.cpp
// Desc: An object for communicating with EPOS motor controllers on a CAN Bus.
//       The CAN Open protocol is used for communicating with the EPOS 
//       controllers but we try to conceal the library we actually use as much
//       as possible in the hope that EPOSControl can be more easily ported
//       to a different CAN Open library.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <stdio.h>
#include "EPOSControl/CANChannel.h"
#include "CANFestivalInterface.h"

//------------------------------------------------------------------------------
CANChannel::CANChannel()
    : mbInitialised( false )
{
}

//------------------------------------------------------------------------------
CANChannel::~CANChannel()
{
    Deinit();
}
    
//------------------------------------------------------------------------------
void CANChannel::OnCANOpenHeartbeatError( U8 error )
{
    printf( "Heartbeat error called\n" );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenInitialisation()
{
    printf( "Initialisation called\n" );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPreOperational()
{
    printf( "PreOperational called\n" );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenOperational()
{
    printf( "Operational called\n" );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenStopped()
{
    printf( "Stopped called\n" );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostSync()
{
    printf( "PostSync called\n" );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostTPDO()
{
    printf( "PostTPDO called\n" );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostEmergency( U8 nodeId, U16 errCode, U8 errReg )
{
    printf( "PostEmergency called\n" );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostSlaveBootup( U8 nodeId )
{
    printf( "PostSlaveBootup for node %i called\n", nodeId );
    
    mMotorControllers[ nodeId ].TellAboutNMTState( eNMTS_PreOperational );
}
    
//------------------------------------------------------------------------------
bool CANChannel::Init( const char* canDevice, eBaudRate baudRate )
{
    if ( !mbInitialised )
    {
        if ( !CFI_InitCANChannel( this, canDevice, baudRate ) )
        {
            fprintf( stderr, "Error: Unable set up CAN bus\n" );
            goto Finished;
        }
        
        for ( S32 nodeId = 0; nodeId < MAX_NUM_MOTOR_CONTROLLERS; nodeId++ )
        {
            mMotorControllers[ nodeId ].Init( this, nodeId );
        }
        
        mbInitialised = true;
    }
    
Finished:
    return mbInitialised;
}

//------------------------------------------------------------------------------
void CANChannel::Deinit()
{
    for ( S32 nodeId = 0; nodeId < MAX_NUM_MOTOR_CONTROLLERS; nodeId++ )
    {
        mMotorControllers[ nodeId ].Deinit();
    }
    
    CFI_DeinitCANChannel( this );
    
    mbInitialised = false;
}
