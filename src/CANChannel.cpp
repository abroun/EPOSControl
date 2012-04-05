//------------------------------------------------------------------------------
// File: CANChannel.cpp
// Desc: An object for communicating with EPOS motor controllers on a CAN Bus.
//       The CAN Open protocol is used for communicating with the EPOS 
//       controllers but we try to conceal the library we actually use as much
//       as possible in the hope that EPOSControl can be more easily ported
//       to a different CAN Open library.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <stdio.h>
#include "EPOSControl/CANChannel.h"
#include "CANOpenInterface.h"

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
    printf( "Channel %i: Heartbeat error called\n", mChannelIdx );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenInitialisation()
{
    printf( "Channel %i: Initialisation called\n", mChannelIdx );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPreOperational()
{
    printf( "Channel %i: PreOperational called\n", mChannelIdx );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenOperational()
{
    printf( "Channel %i: Operational called\n", mChannelIdx );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenStopped()
{
    printf( "Channel %i: Stopped called\n", mChannelIdx );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostSync()
{
    printf( "Channel %i: PostSync called\n", mChannelIdx );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostTPDO()
{
    printf( "Channel %i: PostTPDO called\n", mChannelIdx );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostEmergency( U8 nodeId, U16 errCode, U8 errReg )
{
    printf( "Channel %i: PostEmergency called for node %i - Error: %s\n",
        mChannelIdx, nodeId, GetEposErrorMessage( errCode, errReg ) );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostSlaveBootup( U8 nodeId )
{
    printf( "Channel %i: PostSlaveBootup for node %i called at frame %i\n",
        mChannelIdx, nodeId, mFrameIdx );
    
    mMotorControllers[ nodeId ].TellAboutNMTState( eNMTS_PreOperational );
}

//------------------------------------------------------------------------------
void CANChannel::OnSDOFieldWriteComplete( U8 nodeId )
{
    mMotorControllers[ nodeId ].OnSDOFieldWriteComplete( mFrameIdx );
}

//------------------------------------------------------------------------------
void CANChannel::OnSDOFieldReadComplete( U8 nodeId, U8* pData, U32 numBytes )
{
    mMotorControllers[ nodeId ].OnSDOFieldReadComplete( pData, numBytes );
}
   
//------------------------------------------------------------------------------
void CANChannel::Update()
{
    S32 nodeId = mStartingNodeId;
    S32 numNodesUpdated = 0;
    bool bNewStartingNodeChosen = false;
    
    //printf( "Update called\n" );
    mFrameIdx++;
    
    while ( numNodesUpdated < MAX_NUM_MOTOR_CONTROLLERS )
    {    
        mMotorControllers[ nodeId ].Update( mFrameIdx );
        
        // There are only a limited number of slots available for sending
        // SDO messages. By constantly changing the starting order for updates
        // we ensure that all nodes get a fair chance of sending an SDO message
        if ( mMotorControllers[ nodeId ].GetLastKnownNMTState() != eNMTS_Unknown
            && nodeId != mStartingNodeId
            && false == bNewStartingNodeChosen )
        {
            mStartingNodeId = nodeId;
            bNewStartingNodeChosen = true;
        }
        
        nodeId = (nodeId + 1)%MAX_NUM_MOTOR_CONTROLLERS;
        numNodesUpdated++;
    }
}
   
//------------------------------------------------------------------------------
void CANChannel::ConfigureAllMotorControllersForPositionControl()
{
    for ( S32 nodeId = 0; nodeId < MAX_NUM_MOTOR_CONTROLLERS; nodeId++ )
    {
        mMotorControllers[ nodeId ].SetConfiguration( CANMotorController::eC_PositionControl ); 
    }
}

//------------------------------------------------------------------------------
void CANChannel::GetMotorControllerData( MotorControllerData* pDataBuffer, S32* pBufferSizeOut )
{
    S32 bufferSize = 0;
    
    for ( S32 nodeId = 0; nodeId < MAX_NUM_MOTOR_CONTROLLERS; nodeId++ )
    {
        if ( mMotorControllers[ nodeId ].IsPresent() )
        {
            pDataBuffer[ bufferSize ].mNodeId = nodeId;
            pDataBuffer[ bufferSize ].mState = mMotorControllers[ nodeId ].GetState();
            pDataBuffer[ bufferSize ].mAngle = mMotorControllers[ nodeId ].GetAngle();
            pDataBuffer[ bufferSize ].mbAngleValid = mMotorControllers[ nodeId ].IsAngleValid();
            bufferSize++;
        }
    }
    
    *pBufferSizeOut = bufferSize;
}

//------------------------------------------------------------------------------
void CANChannel::SetMotorAngle( U8 nodeId, S32 angle )
{
    if ( nodeId < MAX_NUM_MOTOR_CONTROLLERS )
    {
         mMotorControllers[ nodeId ].SetDesiredAngle( angle, mFrameIdx );
    }
}

//------------------------------------------------------------------------------
void CANChannel::SetMotorProfileVelocity( U8 nodeId, U32 velocity )
{
    if ( nodeId < MAX_NUM_MOTOR_CONTROLLERS )
    {
        mMotorControllers[ nodeId ].SetProfileVelocity( velocity );
    }
}

//------------------------------------------------------------------------------
void CANChannel::SetMaximumFollowingError( U8 nodeId, U32 maximumFollowingError )
{
    if ( nodeId < MAX_NUM_MOTOR_CONTROLLERS )
    {
        mMotorControllers[ nodeId ].SetMaximumFollowingError( maximumFollowingError );
    }
}

//------------------------------------------------------------------------------
void CANChannel::SendFaultReset( U8 nodeId )
{
    if ( nodeId < MAX_NUM_MOTOR_CONTROLLERS )
    {
         mMotorControllers[ nodeId ].SendFaultReset();
    }
}

//------------------------------------------------------------------------------
const char* CANChannel::GetEposErrorMessage( U16 errCode, U8 errReg )
{
    if ( 0x0000 == errCode && 0x00 == errReg )
    {
        return "No Error";
    }
    else if ( 0x1000 == errCode && 0x01 == errReg )
    {
        return "Generic Error";
    }
    else if ( 0x2310 == errCode && 0x02 == errReg )
    {
        return "Over Current Error";
    }
    else if ( 0x3210 == errCode && 0x04 == errReg )
    {
        return "Over Voltage Error";
    }
    else if ( 0x3220 == errCode && 0x04 == errReg )
    {
        return "Under Voltage";
    }
    else if ( 0x4210 == errCode && 0x08 == errReg )
    {
        return "Over Temperature";
    }
    else if ( 0x5113 == errCode && 0x04 == errReg )
    {
        return "Supply Voltage (+5V) too low";
    }
    else if ( 0x6100 == errCode && 0x20 == errReg )
    {
        return "Internal Software Error";
    }
    else if ( 0x6320 == errCode && 0x20 == errReg )
    {
        return "Software Parameter Error";
    }
    else if ( 0x7320 == errCode && 0x20 == errReg )
    {
        return "Sensor Position Error";
    }
    else if ( 0x8110 == errCode && 0x10 == errReg )
    {
        return "CAN Overrun Error (Objects Lost)";
    }
    else if ( 0x8111 == errCode && 0x10 == errReg )
    {
        return "CAN Overrun Error";
    }
    else if ( 0x8120 == errCode && 0x10 == errReg )
    {
        return "CAN Passive Mode Error";
    }
    else if ( 0x8130 == errCode && 0x10 == errReg )
    {
        return "CAN Life Guard Error";
    }
    else if ( 0x8150 == errCode && 0x10 == errReg )
    {
        return "CAN Tansmit COB-ID collision";
    }
    else if ( 0x81FD == errCode && 0x10 == errReg )
    {
        return "CAN Bus Off";
    }
    else if ( 0x81FE == errCode && 0x10 == errReg )
    {
        return "CAN Rx Queue Overrun";
    }
    else if ( 0x81FF == errCode && 0x10 == errReg )
    {
        return "CAN Tx Queue Overrun";
    }
    else if ( 0x8210 == errCode && 0x10 == errReg )
    {
        return "CAN PDO Length Error";
    }
    else if ( 0x8611 == errCode && 0x20 == errReg )
    {
        return "Following Error";
    }
    else if ( 0xFF01 == errCode && 0x80 == errReg )
    {
        return "Hall Sensor Error";
    }
    else if ( 0xFF02 == errCode && 0x80 == errReg )
    {
        return "Index Processing Error";
    }
    else if ( 0xFF03 == errCode && 0x80 == errReg )
    {
        return "Encoder Resolution Error";
    }
    else if ( 0xFF04 == errCode && 0x80 == errReg )
    {
        return "Hallsensor not found Error";
    }
    else if ( 0xFF06 == errCode && 0x80 == errReg )
    {
        return "Negative Limit Error";
    }
    else if ( 0xFF07 == errCode && 0x80 == errReg )
    {
        return "Positive Limit Error";
    }
    else if ( 0xFF08 == errCode && 0x80 == errReg )
    {
        return "Hall Angle detection Error";
    }
    else if ( 0xFF09 == errCode && 0x80 == errReg )
    {
        return "Software Position Limit Error";
    }
    else if ( 0xFF0A == errCode && 0x80 == errReg )
    {
        return "Position Sensor Breach";
    }
    else if ( 0xFF0B == errCode && 0x20 == errReg )
    {
        return "System Overloaded";
    }
    else
    {
        static char buffer[ 128 ];
        
        snprintf( buffer, sizeof( buffer ), 
                  "Unrecognised error message 0x%X - 0x%X", errCode, errReg );
        buffer[ sizeof( buffer ) - 1 ] = '\0';
        
        return buffer;
    }
}
   
//------------------------------------------------------------------------------
bool CANChannel::Init( const char* driverLibraryName, const char* canDevice, eBaudRate baudRate, S32 channelIdx )
{
    if ( !mbInitialised )
    {
        if ( !COI_InitCANChannel( this, driverLibraryName, canDevice, baudRate ) )
        {
            fprintf( stderr, "Error: Unable set up CAN bus\n" );
            goto Finished;
        }
        
        for ( S32 nodeId = 0; nodeId < MAX_NUM_MOTOR_CONTROLLERS; nodeId++ )
        {
            mMotorControllers[ nodeId ].Init( this, nodeId );
        }
        
        mStartingNodeId = 0;
        mFrameIdx = 0;
        mChannelIdx = channelIdx;
        
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
    
    COI_DeinitCANChannel( this );
    
    mbInitialised = false;
}
