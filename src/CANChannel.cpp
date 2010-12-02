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
void CANChannel::OnSDOFieldWriteComplete( U8 nodeId )
{
    mMotorControllers[ nodeId ].OnSDOFieldWriteComplete();
}

//------------------------------------------------------------------------------
void CANChannel::OnSDOFieldReadComplete( U8 nodeId, U8* pData, U32 numBytes )
{
    mMotorControllers[ nodeId ].OnSDOFieldReadComplete( pData, numBytes );
}
   
//------------------------------------------------------------------------------
void CANChannel::OnCANUpdate()
{
    S32 nodeId = mStartingNodeId;
    S32 numNodesUpdated = 0;
    bool bNewStartingNodeChosen = false;
    
    while ( numNodesUpdated < MAX_NUM_MOTOR_CONTROLLERS )
    {    
        mMotorControllers[ nodeId ].Update();
        
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
    CANMotorControllerAction actionList[ 8 ];
    S32 actionIdx = 0;
    
    actionList[ actionIdx++ ] = CANMotorControllerAction::CreateEnsureNMTStateAction(
        EnsureNMTState( EnsureNMTState::eT_Passive, eNMTS_PreOperational ) );
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Profile Velocity", 0x6081, 0 ) );
    actionList[ actionIdx++ ].mSDOField.SetU32( 500 );
    
    actionList[ actionIdx ] = CANMotorControllerAction::CreateSDOFieldAction(
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
        
        mStartingNodeId = 0;
        
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
