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
    printf( "PostEmergency called for node %i - Error: %s\n", 
            nodeId, GetEposErrorMessage( errCode, errReg ) );
}

//------------------------------------------------------------------------------
void CANChannel::OnCANOpenPostSlaveBootup( U8 nodeId )
{
    printf( "PostSlaveBootup for node %i called at frame %i\n", nodeId, mFrameIdx );
    
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
    CANMotorControllerAction actionList[ 6 ];
    S32 actionIdx = 0;
    
    actionList[ actionIdx++ ] = CANMotorControllerAction::CreateEnsureNMTStateAction(
        EnsureNMTState( EnsureNMTState::eT_Passive, eNMTS_PreOperational ) );
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Mode of Operation", 0x6060, 0 ) );
    actionList[ actionIdx++ ].mSDOField.SetU8( 1 );     // Use profile position mode
        
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Profile Velocity", 0x6081, 0 ) );
    actionList[ actionIdx++ ].mSDOField.SetU32( 500 );
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Motion profile type", 0x6086, 0 ) );
    actionList[ actionIdx++ ].mSDOField.SetU16( 1 );    // Use a sinusoidal profile
    
    /*actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Transmit PDO 1 Parameter", 0x1800, 1 ) );
    actionList[ actionIdx++ ].mSDOField.SetU32( 0x180 + MASTER_NODE_ID );
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Transmit PDO 1 Transmission Type", 0x1800, 2 ) );
    actionList[ actionIdx++ ].mSDOField.SetU8( 255 );    // Asynchronous transfer
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Transmit PDO 1 Inhibition time", 0x1800, 3 ) );
    actionList[ actionIdx++ ].mSDOField.SetU16( 100 );    // Should limit transfer to once every 10ms
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Transmit PDO 1 Map - Num Items", 0x1A00, 0 ) );
    actionList[ actionIdx++ ].mSDOField.SetU8( 0 );    // Disable PDO first
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Transmit PDO 1 Map - Item 1", 0x1A00, 1 ) );
    actionList[ actionIdx++ ].mSDOField.SetU32( ( 0x6064 << 16 ) | ( 0x00 << 8 ) | 32 );    // Actual position
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Transmit PDO 1 Map - Num Items", 0x1A00, 0 ) );
    actionList[ actionIdx++ ].mSDOField.SetU8( 1 );    // Reenable PDO
    */
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Controlword", 0x6040, 0 ) );
    actionList[ actionIdx++ ].mSDOField.SetU16( 0x0006 );    // Shutdown
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Controlword", 0x6040, 0 ) );
    actionList[ actionIdx++ ].mSDOField.SetU16( 0x000F );    // Switch On
    
    S32 numActions = actionIdx;
    assert( ARRAY_LENGTH( actionList ) == numActions );
    
    for ( S32 nodeId = 0; nodeId < MAX_NUM_MOTOR_CONTROLLERS; nodeId++ )
    {
        for ( S32 i = 0; i < numActions; i++ )
        {
            mMotorControllers[ nodeId ].AddConfigurationAction( 
                actionList[ i ] );
        }
    }
}

//------------------------------------------------------------------------------
void CANChannel::GetMotorAngles( AngleData* pAngleBuffer, S32* pBufferSizeOut )
{
    S32 numAngles = 0;
    
    for ( S32 nodeId = 0; nodeId < MAX_NUM_MOTOR_CONTROLLERS; nodeId++ )
    {
        if ( mMotorControllers[ nodeId ].IsAngleValid() )
        {
            pAngleBuffer[ numAngles ].mNodeId = nodeId;
            pAngleBuffer[ numAngles ].mAngle = mMotorControllers[ nodeId ].GetAngle();
            numAngles++;
        }
    }
    
    *pBufferSizeOut = numAngles;
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
void CANChannel::SetMotorProfileVelocity( U32 velocity )
{
    for ( S32 nodeId = 0; nodeId < MAX_NUM_MOTOR_CONTROLLERS; nodeId++ )
    {
        mMotorControllers[ nodeId ].SetProfileVelocity( velocity );
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
bool CANChannel::Init( const char* canDevice, eBaudRate baudRate )
{
    if ( !mbInitialised )
    {
        if ( !COI_InitCANChannel( this, canDevice, baudRate ) )
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
