//------------------------------------------------------------------------------
// File: CANFestivalInterface.cpp
// Desc: Provides access to the CanFestival library which we're using for the
//       CAN Open protocol.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <stdio.h>
#include "CANFestivalInterface.h"
#include "canfestival/canfestival.h"
#include "ObjDict/EPOSObjectDict_1.h"
#include "ObjDict/EPOSObjectDict_2.h"

//------------------------------------------------------------------------------
static const char* CAN_LIBRARY_PATH = "libcanfestival_can_peak_linux.so";

//------------------------------------------------------------------------------
struct ChannelMapping
{
    CANChannel* mpChannel;
    CO_Data* mpCANOpenData;
};

static bool gbCANFestivalStarted = false;
static ChannelMapping gChannelMappings[] = 
{ 
    { NULL, &EPOSObjectDict_1_Data }, 
    { NULL, &EPOSObjectDict_2_Data }
};
COMPILE_TIME_ASSERT( ARRAY_LENGTH( gChannelMappings ) == MAX_NUM_CAN_CHANNELS );

static const char* BAUD_RATES[] = 
{
    "1M",
    "500K",
    "250K",
    "125K",
    "100K",
    "50K",
    "20K",
    "10K",
    "5K"
};
COMPILE_TIME_ASSERT( ARRAY_LENGTH( BAUD_RATES ) == eBR_NumBaudRates );

//------------------------------------------------------------------------------
ChannelMapping* FindMapping( CANChannel* pChannel )
{
    ChannelMapping* pResult = NULL;
    
    for ( S32 mappingIdx = 0; mappingIdx < MAX_NUM_CAN_CHANNELS; mappingIdx++ )
    {
        if ( gChannelMappings[ mappingIdx ].mpChannel == pChannel )
        {
            pResult = &gChannelMappings[ mappingIdx ];
            break;
        }
    }
    
    return pResult;
}

//------------------------------------------------------------------------------
ChannelMapping* FindMapping( CO_Data* pCANOpenData )
{
    ChannelMapping* pResult = NULL;
    
    for ( S32 mappingIdx = 0; mappingIdx < MAX_NUM_CAN_CHANNELS; mappingIdx++ )
    {
        if ( gChannelMappings[ mappingIdx ].mpCANOpenData == pCANOpenData )
        {
            pResult = &gChannelMappings[ mappingIdx ];
            break;
        }
    }
    
    return pResult;
}

//------------------------------------------------------------------------------
// CAN Open event callbacks
//------------------------------------------------------------------------------
void MasterHeartbeatError( CO_Data* pCANOpenData, U8 error )
{
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenHeartbeatError( error );
    }
}

//------------------------------------------------------------------------------
void MasterInitialisation( CO_Data* pCANOpenData )
{
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenInitialisation();
    }
}

//------------------------------------------------------------------------------
void MasterPreOperational( CO_Data* pCANOpenData )
{   
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPreOperational();
    }
}

//------------------------------------------------------------------------------
void MasterOperational( CO_Data* pCANOpenData )
{
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenOperational();
    }
}

//------------------------------------------------------------------------------
void MasterStopped( CO_Data* pCANOpenData )
{
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenStopped();
    }
}

//------------------------------------------------------------------------------
void MasterPostSync( CO_Data* pCANOpenData )
{
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPostSync();
    }
}

//------------------------------------------------------------------------------
void MasterPostTPDO( CO_Data* pCANOpenData )
{
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPostTPDO();
    }
}

//------------------------------------------------------------------------------
void MasterPostEmergency( CO_Data* pCANOpenData, U8 nodeId, U16 errCode, U8 errReg )
{
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPostEmergency( nodeId, errCode, errReg );
    }
}

//------------------------------------------------------------------------------
void MasterPostSlaveBootup( CO_Data* pCANOpenData, U8 nodeId )
{
    ChannelMapping* pMapping = FindMapping( pCANOpenData );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPostSlaveBootup( nodeId );
    }
}

//------------------------------------------------------------------------------
void TimerStartCallback( CO_Data* pCANOpenData, U32 id )
{
    // Nothing to do
}

//------------------------------------------------------------------------------
void UpdateCallback( CO_Data* pCANOpenData, U32 id )
{
    for ( S32 mappingIdx = 0; mappingIdx < MAX_NUM_CAN_CHANNELS; mappingIdx++ )
    {
        if ( NULL != gChannelMappings[ mappingIdx ].mpChannel )
        {
            gChannelMappings[ mappingIdx ].mpChannel->OnCANUpdate();
        }
    }
    printf( "Update called\n" );
}

//------------------------------------------------------------------------------
void ReadSDOFieldCallback( CO_Data* pCANOpenData, U8 nodeId )
{
    U32 abortCode;
    U8 data[ sizeof( SDOField::mData ) ];
    U32 numBytes = sizeof( data );    
    
    U8 readResult = getReadResultNetworkDict( pCANOpenData, 
        nodeId, data, &numBytes, &abortCode );
    
    // Finalize last SDO transfer with this node
    closeSDOtransfer( pCANOpenData, nodeId, SDO_CLIENT );
    
    if ( readResult != SDO_FINISHED )
    {
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeId, abortCode );
    }
    else
    {        
        ChannelMapping* pMapping = FindMapping( pCANOpenData );
        if ( NULL != pMapping )
        {
            pMapping->mpChannel->OnSDOFieldReadComplete( nodeId, data, numBytes );
        }
    }
}

//------------------------------------------------------------------------------
void WriteSDOFieldCallback( CO_Data* pCANOpenData, U8 nodeId )
{
    printf( "Write callback called\n" );
    
    U32 abortCode;
    U8 writeResult = getWriteResultNetworkDict( pCANOpenData, nodeId, &abortCode );
    
    // Finalize last SDO transfer with this node
    closeSDOtransfer( pCANOpenData, nodeId, SDO_CLIENT );
    
    if ( writeResult != SDO_FINISHED )
    {
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeId, abortCode );
    }
    else
    {
        ChannelMapping* pMapping = FindMapping( pCANOpenData );
        if ( NULL != pMapping )
        {
            pMapping->mpChannel->OnSDOFieldWriteComplete( nodeId );
        }
    }
}

//------------------------------------------------------------------------------
void TimerStopCallback( CO_Data* pCANOpenData, U32 id )
{
    for ( S32 mappingIdx = 0; mappingIdx < MAX_NUM_CAN_CHANNELS; mappingIdx++ )
    {
        // All channels should have been shut down by now but we check here
        // just to be safe
        if ( NULL != gChannelMappings[ mappingIdx ].mpChannel )
        {
            assert( false && "CAN Channel was not properly shut down" );
            
            // Reset all other nodes on the network
            masterSendNMTstateChange( gChannelMappings[ mappingIdx ].mpCANOpenData, 0x00, NMT_Reset_Node );    
    
            // Stop master
            setState( gChannelMappings[ mappingIdx ].mpCANOpenData, Stopped );
            
            gChannelMappings[ mappingIdx ].mpChannel = NULL;
        }
    }
}
        
//------------------------------------------------------------------------------
bool CFI_InitCANFestivalInterface()
{
    if ( !gbCANFestivalStarted )
    {
        if ( LoadCanDriver( CAN_LIBRARY_PATH ) == NULL )
        {
            fprintf( stderr, "Error: Unable to load library: %s\n", CAN_LIBRARY_PATH );
            goto Finished;
        }
        
        // Start timer thread
        TimerInit();
        StartTimerLoop( TimerStartCallback );

        // Start an update loop
        EnterMutex();
        SetAlarm( NULL, 0, UpdateCallback, 0, 500000 );  // Trigger update to fire once every 0.5 seconds
        LeaveMutex();
        
        gbCANFestivalStarted = true;
    }
    
Finished:
    return gbCANFestivalStarted;
}

//------------------------------------------------------------------------------
void CFI_DeinitCANFestivalInterface()
{
    if ( gbCANFestivalStarted )
    {
        // Stop timer thread
        StopTimerLoop( TimerStopCallback );   
        TimerCleanup();
    }
    
    gbCANFestivalStarted = false;
}

//------------------------------------------------------------------------------
bool CFI_InitCANChannel( CANChannel* pChannel, const char* canDevice, eBaudRate baudRate )
{   
    assert( baudRate >= 0 && baudRate < eBR_NumBaudRates );
    assert( NULL != pChannel );
    
    bool bResult = false;
    
    if ( gbCANFestivalStarted )
    {
        ChannelMapping* pMapping = NULL;
        for ( S32 mappingIdx = 0; mappingIdx < MAX_NUM_CAN_CHANNELS; mappingIdx++ )
        {
            if ( NULL == gChannelMappings[ mappingIdx ].mpChannel )
            {
                pMapping = &gChannelMappings[ mappingIdx ];
                break;
            }
        }
        
        if ( NULL == pMapping )
        {
            fprintf( stderr, "Error: No more CAN mappings available\n" );
            goto Finished;
        }
        
        // Configure the CanOpen data object
        pMapping->mpCANOpenData->heartbeatError = MasterHeartbeatError;
        pMapping->mpCANOpenData->initialisation = MasterInitialisation;
        pMapping->mpCANOpenData->preOperational = MasterPreOperational;
        pMapping->mpCANOpenData->operational = MasterOperational;
        pMapping->mpCANOpenData->stopped = MasterStopped;
        pMapping->mpCANOpenData->post_sync = MasterPostSync;
        pMapping->mpCANOpenData->post_TPDO = MasterPostTPDO;
        pMapping->mpCANOpenData->post_emcy = MasterPostEmergency;
        pMapping->mpCANOpenData->post_SlaveBootup = MasterPostSlaveBootup;
        
        s_BOARD CAN_CONFIG =
        {
            (char*)canDevice,
            (char*)BAUD_RATES[ baudRate ]
        };
        if( !canOpen( &CAN_CONFIG, pMapping->mpCANOpenData ) )
        {
            fprintf( stderr, "Error: Unable to start CAN comunications\n" );
            goto Finished;
        }
        
        EnterMutex();
        setNodeId( pMapping->mpCANOpenData, MASTER_NODE_ID );
        setState( pMapping->mpCANOpenData, Initialisation );
        LeaveMutex();
        
        pMapping->mpChannel = pChannel;
        bResult = true;
    }
    
Finished:
    return bResult;
}

//------------------------------------------------------------------------------
void CFI_DeinitCANChannel( CANChannel* pChannel )
{
    ChannelMapping* pMapping = FindMapping( pChannel );
    if ( NULL != pMapping )
    {
        EnterMutex();
        
        // Reset all other nodes on the network
        masterSendNMTstateChange( pMapping->mpCANOpenData, 0x00, NMT_Reset_Node );    
    
        // Stop master
        setState( pMapping->mpCANOpenData, Stopped );
        
        LeaveMutex();
        
        
        canClose( pMapping->mpCANOpenData ); 
        pMapping->mpChannel = NULL;
    }
}

//------------------------------------------------------------------------------
void CFI_ProcessSDOField( CANChannel* pChannel, U8 nodeId, SDOField& field )
{
    if ( 5 != nodeId )
    {
        return;
    }
    
    ChannelMapping* pMapping = FindMapping( pChannel );
    if ( NULL != pMapping )
    {
        switch ( field.mType )
        {
            case SDOField::eT_Write:
            {
                printf( "Processing write for node %i\n", nodeId );
                printf( "%i %i b: %i a: %x\n", field.mIndex, field.mSubIndex, 
                    field.mNumBytes, field.mData );
                
                writeNetworkDictCallBack( pMapping->mpCANOpenData,
                    nodeId, field.mIndex, field.mSubIndex, 
                    field.mNumBytes, 0, field.mData,
                    WriteSDOFieldCallback );
                    
                printf( "Write sent\n" );
                break;
            }
            case SDOField::eT_Read:
            {
                printf( "Processing read\n" );
                
                readNetworkDictCallback( pMapping->mpCANOpenData,
                    nodeId, field.mIndex, field.mSubIndex, 
                    0, ReadSDOFieldCallback );
                break;
            }
            default:
            {
                assert( false && "Unhandled field type" );
            }
        }
    }
}