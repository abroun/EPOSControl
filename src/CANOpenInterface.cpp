//------------------------------------------------------------------------------
// File: CANOpenInterface.cpp
// Desc: Provides access to the whichever library we're using for the
//       CAN Open protocol.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <stdio.h>
#include "CANOpenInterface.h"
#include "CanOpenMaster/CanOpenMaster.h"

//------------------------------------------------------------------------------
struct ChannelMapping
{
    CANChannel* mpChannel;
    COM_CanChannelHandle mChannelHandle;
};

static bool gbCANOpenStarted = false;
static ChannelMapping gChannelMappings[] = 
{ 
    { NULL, NULL }, 
    { NULL, NULL }
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
static ChannelMapping* FindMapping( CANChannel* pChannel )
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
static ChannelMapping* FindMapping( COM_CanChannelHandle channelHandle )
{
    ChannelMapping* pResult = NULL;
    
    for ( S32 mappingIdx = 0; mappingIdx < MAX_NUM_CAN_CHANNELS; mappingIdx++ )
    {
        if ( gChannelMappings[ mappingIdx ].mChannelHandle == channelHandle )
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
void MasterHeartbeatError( COM_CanChannelHandle handle, U8 error )
{
    ChannelMapping* pMapping = FindMapping( handle );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenHeartbeatError( error );
    }
}

//------------------------------------------------------------------------------
void MasterPostSync( COM_CanChannelHandle handle )
{
    ChannelMapping* pMapping = FindMapping( handle );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPostSync();
    }
}

//------------------------------------------------------------------------------
void MasterPostTPDO( COM_CanChannelHandle handle )
{
    ChannelMapping* pMapping = FindMapping( handle );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPostTPDO();
    }
}

//------------------------------------------------------------------------------
void MasterPostEmergency( COM_CanChannelHandle handle, U8 nodeId, U16 errCode, U8 errReg )
{
    ChannelMapping* pMapping = FindMapping( handle );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPostEmergency( nodeId, errCode, errReg );
    }
}

//------------------------------------------------------------------------------
void MasterPostSlaveBootup( COM_CanChannelHandle handle, U8 nodeId )
{
    ChannelMapping* pMapping = FindMapping( handle );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnCANOpenPostSlaveBootup( nodeId );
    }
}

//------------------------------------------------------------------------------
void ReadSDOFieldCallback( COM_CanChannelHandle handle, U8 nodeId,
                           U8* pData, U8 numBytes )
{ 
    ChannelMapping* pMapping = FindMapping( handle );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnSDOFieldReadComplete( nodeId, pData, numBytes );
    }
}

//------------------------------------------------------------------------------
void WriteSDOFieldCallback( COM_CanChannelHandle handle, U8 nodeId )
{   
    ChannelMapping* pMapping = FindMapping( handle );
    if ( NULL != pMapping )
    {
        pMapping->mpChannel->OnSDOFieldWriteComplete( nodeId );
    }
}
        
//------------------------------------------------------------------------------
bool COI_InitCANOpenInterface()
{
    if ( !gbCANOpenStarted )
    {
        if ( !COM_Init() )
        {
            fprintf( stderr, "Error: Unable to load start CanOpenMaster library\n" );
            goto Finished;
        }
        
        gbCANOpenStarted = true;
    }
    
Finished:
    return gbCANOpenStarted;
}

//------------------------------------------------------------------------------
void COI_DeinitCANOpenInterface()
{
    COM_Deinit();
    gbCANOpenStarted = false;
}

//------------------------------------------------------------------------------
bool COI_InitCANChannel( CANChannel* pChannel, const char* driverLibraryName, const char* canDevice, eBaudRate baudRate )
{   
    assert( baudRate >= 0 && baudRate < eBR_NumBaudRates );
    assert( NULL != pChannel );
    
    bool bResult = false;
    
    if ( gbCANOpenStarted )
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
        
        // Specify the callbacks for the channel
        COM_CanChannelCallbacks callbacks;
        callbacks.mHeartbeatErrorCB = MasterHeartbeatError;
        callbacks.mPostSyncCB = MasterPostSync;
        callbacks.mPostTpdoCB = MasterPostTPDO;
        callbacks.mPostEmergencyCB = MasterPostEmergency;
        callbacks.mPostSlaveBootupCB = MasterPostSlaveBootup;
        
        COM_CanChannelHandle channelHandle = COM_OpenChannel(
            driverLibraryName, canDevice, BAUD_RATES[ baudRate ], callbacks );
        if ( NULL == channelHandle )
        {
            fprintf( stderr, "Error: Unable to start CAN comunications\n" );
            goto Finished;
        }
        
        // Reset the nodes on the channel
        // TODO: Move out of here once we have an interface for sending NMT messages
        COM_QueueNmtResetNode( channelHandle, 0 );
        
        pMapping->mpChannel = pChannel;
        pMapping->mChannelHandle = channelHandle;
        bResult = true;
    }
    
Finished:
    return bResult;
}

//------------------------------------------------------------------------------
void COI_DeinitCANChannel( CANChannel* pChannel )
{
    ChannelMapping* pMapping = FindMapping( pChannel );
    if ( NULL != pMapping )
    {
        COM_CloseChannel( &(pMapping->mChannelHandle) );
        pMapping->mpChannel = NULL;
    }
}

//------------------------------------------------------------------------------
bool COI_ProcessSDOField( CANChannel* pChannel, U8 nodeId, const SDOField& field )
{   
    bool bFieldProcessed = false;
    
    ChannelMapping* pMapping = FindMapping( pChannel );
    if ( NULL != pMapping )
    {
        switch ( field.mType )
        {
            case SDOField::eT_Write:
            {
                bFieldProcessed = COM_QueueSdoWriteMsg( 
                    pMapping->mChannelHandle, nodeId, 
                    field.mIndex, field.mSubIndex, 
                    WriteSDOFieldCallback,
                    field.mData, field.mNumBytes );

                break;
            }
            case SDOField::eT_Read:
            {   
                bFieldProcessed = COM_QueueSdoReadMsg( 
                    pMapping->mChannelHandle, nodeId, 
                    field.mIndex, field.mSubIndex, 
                    ReadSDOFieldCallback );
                    
                break;
            }
            default:
            {
                assert( false && "Unhandled field type" );
            }
        }
    }
    
    return bFieldProcessed;
}
