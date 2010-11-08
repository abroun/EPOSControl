
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include "canfestival/canfestival.h"
#include "libpcan.h"
#include "EPOSMaster.h"

typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned int U32;
typedef char S8;
typedef short S16;
typedef int S32;

static const char* CAN_LIBRARY_PATH = "libcanfestival_can_peak_linux.so";
//static const char* CAN_BUS_NAME = ;
//static const char* CAN_BAUD_RATE = "1M";

s_BOARD CAN_CONFIG =
{
    (char*)"32",    // Bus name (/dev/pcan{BUS_NAME})
    (char*)"1M"     // Baud rate
};

#define NODE_ID 5
#define MASTER_NODE_ID 25

bool gbActiveSlaves[ 128 ] = { false };

struct SDOField
{
    SDOField() 
        : mpName( NULL ), mIndex( 0 ), mSubIndex( 0 ), mNumBytes( 0 ) {}
    SDOField( const char* pName, U16 mIndex, U8 mSubIndex )
        : mpName( pName ), mIndex( mIndex ), mSubIndex( mSubIndex ), mNumBytes( 0 ) {}
    
    void SetU8( U8 data ) 
    { 
        mData[ 0 ] = data;
        mNumBytes = 1;
    }
    
    void SetU16( U16 data ) 
    { 
        ((U16*)mData)[ 0 ] = data;
        mNumBytes = 2;
    }
    
    void SetU32( U32 data ) 
    { 
        ((U32*)mData)[ 0 ] = data;
        mNumBytes = 4;
    }
    
    void SetS32( S32 data ) 
    { 
        SetU32( (U32)data );
    }
    
    const char* mpName;
    U16 mIndex;
    U8 mSubIndex;
    U8 mData[ 8 ];
    U32 mNumBytes;
};

SDOField gSDOFieldBuffer[ 64 ];
U32 gCurNumSDOFields = 0;
U32 gCurSDOFieldIdx = 0;
bool gbSDOFieldsInUse = false;
bool gbSDOWriteAcknowledged = true;

enum eProgramState
{
    ePS_StartingUp = 0,
    ePS_ReadingInitialValues,
    ePS_ConfiguringNode,
    ePS_ReadingConfiguredValues,
    ePS_ControllingNode
};
eProgramState gProgramState = ePS_StartingUp;

enum eControlState
{
    eCS_WaitingToStart = 0,
    eCS_ShutdownMessageSent,
    eCS_SwitchOnMessageSent,
    eCS_NewTargetPosSet,
    eCS_WaitingForTargetToBeReached,
};
eControlState gControlState = eCS_WaitingToStart;

S32 TARGET_POSITIONS[] =
{
    0,
    10000 //50000
};
S32 NUM_TARGET_POSITONS = sizeof( TARGET_POSITIONS )/sizeof( TARGET_POSITIONS[ 0 ] );
S32 gCurTargetPosition = 0;

extern "C" {
UNS8 canReceive(CAN_PORT port, Message *m);
}

#if !defined(WIN32) || defined(__CYGWIN__)
void catch_signal(int sig)
{
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  printf("Got Signal %d\n",sig);
}
#endif

// Callbacks
void Master_heartbeatError(CO_Data* d, UNS8)
{
    printf( "HeartbeatError called\n" );
}

void Master_initialisation(CO_Data* d)
{
    printf( "Initialisation called - d == &EPOSMaster_Data = %i\n", &EPOSMaster_Data == d );
    
    for ( int i = 1; i <= 18; i++ )
    {
        UNS32 transmitCOBID = 0x600 + i;
        UNS32 transmitCOBIDSize = sizeof(transmitCOBID);
        UNS32 receiveCOBID = 0x580 + i;
        UNS32 receiveCOBIDSize = sizeof(receiveCOBID);
        UNS8 nodeId = (UNS8)i;
        UNS32 nodeIdSize = sizeof(nodeId);
        
        UNS16 index = 0x1280 - 1 + i;
        writeLocalDict( &EPOSMaster_Data, index, (UNS8)1, &transmitCOBID, &transmitCOBIDSize, 1 );
        writeLocalDict( &EPOSMaster_Data, index, (UNS8)2, &receiveCOBID, &receiveCOBIDSize, 1 );
        writeLocalDict( &EPOSMaster_Data, index, (UNS8)3, &nodeId, &nodeIdSize, 1 );
    }
}

void Master_preOperational(CO_Data* d)
{
    printf( "PreOperational called\n" );
    
    // Setup Local Dictionary to handle PDO data from the other nodes on the CANBus
    UNS32 recieveCOBID = 0x180 + MASTER_NODE_ID;
    UNS32 size = sizeof( recieveCOBID );
    writeLocalDict( &EPOSMaster_Data, 0x1400, 1, &recieveCOBID, &size, 1 );
    
    U8 numMappedFields = 0;         // Disable PDO
    size = sizeof( numMappedFields );
    writeLocalDict( &EPOSMaster_Data, 0x1600, 0, &numMappedFields, &size, 1 );
    
    printf( "Trying to set mappedField\n" );
    
    U32 mappedField = ( 0x2000 << 16 ) | ( 0x02 << 8 ) | 32;   // Statusword // Actual position
    size = sizeof( mappedField );
    U32 result = writeLocalDict( &EPOSMaster_Data, 0x1600, 1, &mappedField, &size, 1 );
    
    printf( "Trying Done... Result was %i\n", result );
    
    U32 mappingParam = 0;
    U32 expectedSize = sizeof( mappingParam );
    U8 dataType;
    
    readLocalDict( &EPOSMaster_Data, 0x1600, 1, &mappingParam, &expectedSize, &dataType, 1 );
    printf( "Mapping param should be: 0x%X but is 0x%X\n", mappedField, mappingParam );
    
    numMappedFields = 1;    // Re-enable PDO
    size = sizeof( numMappedFields );
    writeLocalDict( &EPOSMaster_Data, 0x1600, 0, &numMappedFields, &size, 1 );
}

void Master_operational(CO_Data* d)
{
    printf( "Operational called\n" );
}

void Master_stopped(CO_Data* d)
{
    printf( "Stopped called\n" );
}

void Master_post_sync(CO_Data* d)
{
    printf( "PostSync called\n" );
}

void Master_post_TPDO(CO_Data* d)
{
    printf( "PostTPDO called\n" );
}

void Master_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg)
{
    printf( "PostEmcy called\n" );
}

void Master_post_SlaveBootup(CO_Data* d, UNS8 nodeid)
{
    printf( "SlaveBootup called - node is %i\n", nodeid );
    gbActiveSlaves[ nodeid ] = true;
}

/***************************  INIT  *****************************************/
void InitNodes(CO_Data* d, UNS32 id)
{
    /****************************** INITIALISATION MASTER *******************************/    
    setNodeId( &EPOSMaster_Data, MASTER_NODE_ID );
    
    /* init */
    setState( &EPOSMaster_Data, Initialisation );
}

void PresentSDOFields()
{
    for ( S32 i = 0; i < gCurNumSDOFields; i++ )
    {
        SDOField& curField = gSDOFieldBuffer[ i ];
        U32 data = ((U32*)curField.mData)[ 0 ];
        printf( "%s: %i, 0x%X\n", curField.mpName, data, data );
    }
}

void ReadSDOFieldCallback(CO_Data* d, UNS8 nodeid)
{
    UNS32 abortCode;
    SDOField& curField = gSDOFieldBuffer[ gCurSDOFieldIdx ];
    curField.mNumBytes = sizeof(SDOField::mData);
    
    
    U8 readResult = getReadResultNetworkDict( &EPOSMaster_Data, 
        nodeid, curField.mData, &curField.mNumBytes, &abortCode);
    
    /* Finalize last SDO transfer with this node */
    closeSDOtransfer(&EPOSMaster_Data, nodeid, SDO_CLIENT);
    
    if ( readResult != SDO_FINISHED )
    {
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
    }
    else
    {        
        gCurSDOFieldIdx++;
        if ( gCurSDOFieldIdx >= gCurNumSDOFields )
        {
            PresentSDOFields();
            gbSDOFieldsInUse = false;
        }
        else
        {
            SDOField& nextField = gSDOFieldBuffer[ gCurSDOFieldIdx ];
            readNetworkDictCallback( &EPOSMaster_Data, 
                NODE_ID, nextField.mIndex, nextField.mSubIndex, 
                0, ReadSDOFieldCallback );
        }
    }
}

void WriteSDOFieldCallback(CO_Data* d, UNS8 nodeid)
{
    UNS32 abortCode;
    SDOField& curField = gSDOFieldBuffer[ gCurSDOFieldIdx ];
    curField.mNumBytes = sizeof(SDOField::mData);
    
    
    U8 writeResult = getWriteResultNetworkDict( &EPOSMaster_Data, 
        nodeid, &abortCode);
    
    /* Finalize last SDO transfer with this node */
    closeSDOtransfer(&EPOSMaster_Data, nodeid, SDO_CLIENT);
    
    if ( writeResult != SDO_FINISHED )
    {
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
    }
    else
    {        
        gCurSDOFieldIdx++;
        if ( gCurSDOFieldIdx >= gCurNumSDOFields )
        {
            gbSDOFieldsInUse = false;
        }
        else
        {
            SDOField& nextField = gSDOFieldBuffer[ gCurSDOFieldIdx ];
            writeNetworkDictCallBack( &EPOSMaster_Data, 
                NODE_ID, nextField.mIndex, nextField.mSubIndex, 
                nextField.mNumBytes, 0, nextField.mData,
                WriteSDOFieldCallback );
        }
    }
}

void StatuswordSDOReadCallback(CO_Data* d, UNS8 nodeid)
{
    UNS32 abortCode;
    UNS32 data=0;
    UNS32 size=64;

    if(getReadResultNetworkDict(&EPOSMaster_Data, nodeid, &data, &size, &abortCode) != SDO_FINISHED)
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
    else
        printf("\nStatusword : %x\n", data);

    /* Finalize last SDO transfer with this node */
    closeSDOtransfer(&EPOSMaster_Data, nodeid, SDO_CLIENT);
}

void CheckWriteSDO(CO_Data* d, UNS8 nodeid)
{
    UNS32 abortCode;

    if(getWriteResultNetworkDict(&EPOSMaster_Data, nodeid, &abortCode) != SDO_FINISHED)
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
    else
    {
        //printf("\nSend data OK\n");
        gbSDOWriteAcknowledged = true;
    }

    /* Finalize last SDO transfer with this node */
    closeSDOtransfer(&EPOSMaster_Data, nodeid, SDO_CLIENT);
}

void CheckTargetReachedCallback(CO_Data* d, UNS8 nodeid)
{
    UNS32 abortCode;
    UNS32 data=0;
    UNS32 size=64;

    U8 readResult = getReadResultNetworkDict(&EPOSMaster_Data, 
        nodeid, &data, &size, &abortCode);
        
    /* Finalize last SDO transfer with this node */
    closeSDOtransfer(&EPOSMaster_Data, nodeid, SDO_CLIENT);
    
    if ( SDO_FINISHED != readResult )
    {
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
    }
    else
    {
        gbSDOWriteAcknowledged = true;
        if ( data & ( 1 << 10 ) )
        {
            printf( "Reached Target Pos %i\n", TARGET_POSITIONS[ gCurTargetPosition ] );
            gCurTargetPosition = (gCurTargetPosition + 1)%NUM_TARGET_POSITONS;
            gControlState = eCS_SwitchOnMessageSent;
        }
    }
}

/***************************  EXIT  *****************************************/
void DeinitNodes(CO_Data* d, UNS32 id)
{       
    // Reset all other nodes on the network
    masterSendNMTstateChange( &EPOSMaster_Data, 0x00, NMT_Reset_Node);    
    
    // Stop master
    setState( &EPOSMaster_Data, Stopped );
}



void Update(CO_Data* d, UNS32 id)
{
    static bool bDataValuesRead = false;
    
    printf( "Update called - d == &EPOSMaster_Data = %i %x\n", &EPOSMaster_Data == d, d );
    //printf( "Updating\n" );
    
    if ( !gbActiveSlaves[ NODE_ID ] )
    {
        return;
    }
    
    S32 curPosition = 0;
    U32 expectedSize = sizeof( curPosition );
    U8 dataType;
    readLocalDict( &EPOSMaster_Data, 0x2000, 2, &curPosition, &expectedSize, &dataType, 1 );
    
    
    
    printf( "Cur Position: %i\n", curPosition );
    
    //sendPDOrequest( &EPOSMaster_Data, 0x1400 );
    
    switch ( gProgramState )
    {
        case ePS_StartingUp:
        {
            if ( !gbSDOFieldsInUse )
            {
                masterSendNMTstateChange( &EPOSMaster_Data, NODE_ID, NMT_Enter_PreOperational );
                
                
                gbSDOFieldsInUse = true;
                gSDOFieldBuffer[ 0 ] = SDOField( "Motor Type", 0x6402, 0 );
                gSDOFieldBuffer[ 1 ] = SDOField( "Current Limit", 0x6410, 1 );
                gSDOFieldBuffer[ 2 ] = SDOField( "Pole Pair Number", 0x6410, 3 );
                gSDOFieldBuffer[ 3 ] = SDOField( "Thermal Time Constant Winding", 0x6410, 5 );
                gSDOFieldBuffer[ 4 ] = SDOField( "Encoder Pulse Number", 0x2210, 1 );
                gSDOFieldBuffer[ 5 ] = SDOField( "Position Sensor Type", 0x2210, 2 );
                gSDOFieldBuffer[ 6 ] = SDOField( "Current P-Gain", 0x60F6, 1 );
                gSDOFieldBuffer[ 7 ] = SDOField( "Current I-Gain", 0x60F6, 2 );
                gSDOFieldBuffer[ 8 ] = SDOField( "Speed P-Gain", 0x60F9, 1 );
                gSDOFieldBuffer[ 9 ] = SDOField( "Speed I-Gain", 0x60F9, 2 );
                gSDOFieldBuffer[ 10 ] = SDOField( "Position P-Gain", 0x60FB, 1 );
                gSDOFieldBuffer[ 11 ] = SDOField( "Position I-Gain", 0x60FB, 2 );
                gSDOFieldBuffer[ 12 ] = SDOField( "Position D-Gain", 0x60FB, 3 );
                gSDOFieldBuffer[ 13 ] = SDOField( "Mode of Operation", 0x6060, 0 );
                gSDOFieldBuffer[ 14 ] = SDOField( "Max Following Error", 0x6065, 0 );
                gSDOFieldBuffer[ 15 ] = SDOField( "Min Position Limit", 0x607D, 1 );
                gSDOFieldBuffer[ 16 ] = SDOField( "Max Position Limit", 0x607D, 2 );
                gSDOFieldBuffer[ 17 ] = SDOField( "Max Profile Velocity", 0x607F, 0 );
                gSDOFieldBuffer[ 18 ] = SDOField( "Profile Velocity", 0x6081, 0 );
                gSDOFieldBuffer[ 19 ] = SDOField( "Profile Acceleration", 0x6083, 0 );
                gSDOFieldBuffer[ 20 ] = SDOField( "Profile Deceleration", 0x6084, 0 );
                gSDOFieldBuffer[ 21 ] = SDOField( "Quick Stop Deceleration", 0x6085, 0 );
                gSDOFieldBuffer[ 22 ] = SDOField( "Motion Profile Type", 0x6086, 0 );
                gSDOFieldBuffer[ 23 ] = SDOField( "Target Position", 0x607A, 0 );
                gSDOFieldBuffer[ 24 ] = SDOField( "Position Demand", 0x6062, 0 );
                gSDOFieldBuffer[ 25 ] = SDOField( "Position Actual", 0x6064, 0 );
                gCurNumSDOFields = 26;
                
                gCurSDOFieldIdx = 0;
                SDOField& curField = gSDOFieldBuffer[ gCurSDOFieldIdx ];
                readNetworkDictCallback( &EPOSMaster_Data, 
                    NODE_ID, curField.mIndex, curField.mSubIndex, 0, ReadSDOFieldCallback );
                    
                gProgramState = ePS_ReadingInitialValues;
            }
            break;
        }
        case ePS_ReadingInitialValues:
        {
            if ( !gbSDOFieldsInUse )
            {
                gSDOFieldBuffer[ 0 ] = SDOField( "Profile Velocity", 0x6081, 0 );
                gSDOFieldBuffer[ 0 ].SetU32( 500 );
                gSDOFieldBuffer[ 1 ] = SDOField( "Transmit PDO 1 Parameter", 0x1800, 1 );
                gSDOFieldBuffer[ 1 ].SetU32( 0x180 + MASTER_NODE_ID );
                gSDOFieldBuffer[ 2 ] = SDOField( "Transmit PDO 1 Transmission Type", 0x1800, 2 );
                gSDOFieldBuffer[ 2 ].SetU8( 255 );    // Asynchronous transfer
                gSDOFieldBuffer[ 3 ] = SDOField( "Transmit PDO 1 Inhibition time", 0x1800, 3 );
                gSDOFieldBuffer[ 3 ].SetU16( 100 );    // Should limit transfer to once every 10ms
                gSDOFieldBuffer[ 4 ] = SDOField( "Transmit PDO 1 Map - Num Items", 0x1A00, 0 );
                gSDOFieldBuffer[ 4 ].SetU8( 0 );    // Disable PDO first
                gSDOFieldBuffer[ 5 ] = SDOField( "Transmit PDO 1 Map - Item 1", 0x1A00, 1 );
                gSDOFieldBuffer[ 5 ].SetU32( ( 0x6064 << 16 ) | ( 0x00 << 8 ) | 32 );    // Actual position
                gSDOFieldBuffer[ 6 ] = SDOField( "Transmit PDO 1 Map - Num Items", 0x1A00, 0 );
                gSDOFieldBuffer[ 6 ].SetU8( 1 );    // Reenable PDO
                
                gCurNumSDOFields = 7;
                
                gCurSDOFieldIdx = 0;
                SDOField& curField = gSDOFieldBuffer[ gCurSDOFieldIdx ];
                writeNetworkDictCallBack( &EPOSMaster_Data, 
                    NODE_ID, curField.mIndex, curField.mSubIndex, 
                    curField.mNumBytes, 0, curField.mData,
                    WriteSDOFieldCallback );
                    
                gProgramState = ePS_ConfiguringNode;
            }
            break;
        }
        case ePS_ConfiguringNode:
        {
            if ( !gbSDOFieldsInUse )
            {
                printf( "Reading back set values\n" );
                
                gbSDOFieldsInUse = true;
                //masterSendNMTstateChange( &EPOSMaster_Data, NODE_ID, NMT_Start_Node );
                //masterSendNMTstateChange( &EPOSMaster_Data, MASTER_NODE_ID, NMT_Start_Node );
                
                gSDOFieldBuffer[ 0 ] = SDOField( "Profile Velocity", 0x6081, 0 );
                gSDOFieldBuffer[ 1 ] = SDOField( "Transmit PDO 1 Parameter", 0x1800, 1 );
                gSDOFieldBuffer[ 2 ] = SDOField( "Transmit PDO 1 Transmission Type", 0x1800, 2 );
                gSDOFieldBuffer[ 3 ] = SDOField( "Transmit PDO 1 Inhibition time", 0x1800, 3 );
                gSDOFieldBuffer[ 4 ] = SDOField( "Transmit PDO 1 Map - Num Items", 0x1A00, 0 );
                gSDOFieldBuffer[ 5 ] = SDOField( "Transmit PDO 1 Map - Item 1", 0x1A00, 1 );
                gSDOFieldBuffer[ 6 ] = SDOField( "Status word", 0x6041, 0 );
                gCurNumSDOFields = 7;
                
                gCurSDOFieldIdx = 0;
                SDOField& curField = gSDOFieldBuffer[ gCurSDOFieldIdx ];
                readNetworkDictCallback( &EPOSMaster_Data, 
                    NODE_ID, curField.mIndex, curField.mSubIndex, 0, ReadSDOFieldCallback );
                    
                gProgramState = ePS_ReadingConfiguredValues;
            }
            break;
        }
        case ePS_ReadingConfiguredValues:
        {
            if ( !gbSDOFieldsInUse )
            {
                // Move the node into the operational state
                setState( &EPOSMaster_Data, Operational );
                masterSendNMTstateChange( &EPOSMaster_Data, NODE_ID, NMT_Start_Node );
                masterSendNMTstateChange( &EPOSMaster_Data, MASTER_NODE_ID, NMT_Start_Node );
                gProgramState = ePS_ControllingNode;
            }
            break;
        }   
        case ePS_ControllingNode:
        {
            if ( !gbSDOFieldsInUse
                && gbSDOWriteAcknowledged )
            {   
                // We have read all the values so we can now setup and control the motor
                switch ( gControlState )
                {
                    case eCS_WaitingToStart:
                    {
                        U16 data = 0x0006;
                        if ( writeNetworkDictCallBack( &EPOSMaster_Data, NODE_ID, 
                            0x6040, 0, sizeof( data ), 
                            0, &data, CheckWriteSDO ) == 0 )
                        {
                            gbSDOWriteAcknowledged = false;
                            gControlState = eCS_ShutdownMessageSent;
                        }
                        break;
                    }
                    case eCS_ShutdownMessageSent:
                    {
                        U16 data = 0x000F;
                        if ( writeNetworkDictCallBack( &EPOSMaster_Data, NODE_ID, 
                            0x6040, 0, sizeof( data ), 
                            0, &data, CheckWriteSDO ) == 0 )
                        {
                            gbSDOWriteAcknowledged = false;
                            gControlState = eCS_SwitchOnMessageSent;
                        }
                        break;
                    }
                    case eCS_SwitchOnMessageSent:
                    {
                        S32 data = TARGET_POSITIONS[ gCurTargetPosition ];
                        if ( writeNetworkDictCallBack( &EPOSMaster_Data, NODE_ID, 
                            0x607A, 0, sizeof( data ), 
                            0, &data, CheckWriteSDO ) == 0 )
                        {
                            gbSDOWriteAcknowledged = false;
                            gControlState = eCS_NewTargetPosSet;
                        }
                        break;
                    }
                    case eCS_NewTargetPosSet:
                    {
                        U16 data = 0x001F;
                        if ( writeNetworkDictCallBack( &EPOSMaster_Data, NODE_ID, 
                            0x6040, 0, sizeof( data ), 
                            0, &data, CheckWriteSDO ) == 0 )
                        {
                            gbSDOWriteAcknowledged = false;
                            gControlState = eCS_WaitingForTargetToBeReached;
                        }
                        break;
                    }
                    case eCS_WaitingForTargetToBeReached:
                    {
                        if ( readNetworkDictCallback( &EPOSMaster_Data, 
                            NODE_ID, 0x6041, 0, 0, CheckTargetReachedCallback ) == 0 )
                        {
                            gbSDOWriteAcknowledged = false;
                        }
                    }
                }
            }
            break;
        }
    
    }
}

//------------------------------------------------------------------------------
int main( int argc, char** argv )
{
    #if !defined(WIN32) || defined(__CYGWIN__)
  /* install signal handler for manual break */
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    TimerInit();
#endif
    
    // Load the CAN library
    if ( LoadCanDriver( CAN_LIBRARY_PATH ) == NULL )
    {
        fprintf( stderr, "Error: Unable to load library: %s\n", CAN_LIBRARY_PATH );
        return -1;
    }
    
    // Setup a master CANBus node
    EPOSMaster_Data.heartbeatError = Master_heartbeatError;
    EPOSMaster_Data.initialisation = Master_initialisation;
    EPOSMaster_Data.preOperational = Master_preOperational;
    EPOSMaster_Data.operational = Master_operational;
    EPOSMaster_Data.stopped = Master_stopped;
    EPOSMaster_Data.post_sync = Master_post_sync;
    EPOSMaster_Data.post_TPDO = Master_post_TPDO;
    EPOSMaster_Data.post_emcy = Master_post_emcy;
    EPOSMaster_Data.post_SlaveBootup=Master_post_SlaveBootup;
        
    if( !canOpen( &CAN_CONFIG, &EPOSMaster_Data ) )
    {
        fprintf( stderr, "Error: Unable to start CAN comunications\n" );
        return -1;
    }
    
    // Start timer thread
    StartTimerLoop( InitNodes );

    // Start an update loop
    EnterMutex();
    SetAlarm(NULL, 0, Update, 0, 500000 );  // Trigger update to fire once every 0.5 seconds
    LeaveMutex();
    
    // wait Ctrl-C
    pause();

    // Stop timer thread
    StopTimerLoop( DeinitNodes );
    
    // Close CAN devices (and can threads)
    canClose( &EPOSMaster_Data );    
    TimerCleanup();
    return 0;
    
    /*// Load the CAN library
    if ( LoadCanDriver( CAN_LIBRARY_PATH ) == NULL )
    {
        fprintf( stderr, "Error: Unable to load library: %s\n", CAN_LIBRARY_PATH );
        return -1;
    }
    
    // Open up the CAN port
    CO_Data canData;
    memset( &canData, 0, sizeof( canData ) );
    
    CAN_PORT pCANPort = canOpen( &CAN_CONFIG, &canData );
    if ( NULL == pCANPort )
    {
        fprintf( stderr, "Error: Unable to open CAN communication\n" );
        return -1;
    }
    
    // Read the status of an EPOS node
    Message msg = Message_Initializer;
    msg.cob_id = 0x600 + NODE_ID;
    msg.data[ 0 ] = 0x40;   // Read SDO
    msg.data[ 1 ] = 0x41;   // Statusword low byte
    msg.data[ 2 ] = 0x60;   // Statusword high byte
    canSend( pCANPort, &msg );
    
    printf( "Waiting for response\n" );
    canReceive( pCANPort, &msg );
    
    printf( "Got response: COB_ID = %x, Bytes = %x,%x,%x,%x\n",
        msg.cob_id, msg.data[ 4 ], msg.data[ 5 ], msg.data[ 6 ], msg.data[ 7 ] );
    
    
    printf( "All done\n" );
    
    
    canClose( &canData );*/
    
    /*int result;
    
    //HANDLE canHandle = CAN_Open( HW_USB, 1 );
    HANDLE canHandle = LINUX_CAN_Open( "/dev/pcan32", O_RDWR );
    if ( NULL == canHandle )
    {
        fprintf( stderr, "Error: Unable to open CAN communication\n" );
        return -1;
    }
    
    result = CAN_Init( canHandle, CAN_BAUD_1M, CAN_INIT_TYPE_ST );
    printf( "Init result was %i\n", result );
    
    TPCANMsg msg;
    memset( &msg, 0, sizeof( msg ) );
    msg.ID = 0x600 + NODE_ID;
    msg.MSGTYPE = MSGTYPE_STANDARD;
    msg.LEN = 4;
    msg.DATA[ 0 ] = 0x40;   // Read SDO
    msg.DATA[ 1 ] = 0x41;   // Statusword low byte
    msg.DATA[ 2 ] = 0x60;   // Statusword high byte
    result = CAN_Write( canHandle, &msg );
    
    printf( "Write result was %i\n", result );
    
    //for ( int i = 0; i < 100000; i++ );
    
    TPCANRdMsg timeStampMsg;
    result = LINUX_CAN_Read_Timeout( canHandle, &timeStampMsg, 5000000 );
    
    TPCANMsg& msgRef = timeStampMsg.Msg;
    printf( "Message read result was %i\n", result );
    printf( "Message contents are COB_ID = %x, Bytes = %x,%x,%x,%x\n",
        msgRef.ID, msgRef.DATA[ 4 ], msgRef.DATA[ 5 ], msgRef.DATA[ 6 ], msgRef.DATA[ 7 ] );
    
    for ( int i = 0; i < 100000; i++ );
    
    int numPendingReads;
    int numPendingWrites;
    result = LINUX_CAN_Extended_Status( canHandle, &numPendingReads, &numPendingWrites );
    printf( "Status result was %i\n", result );
    printf( "%i pending reads and %i pending writes\n", numPendingReads, numPendingWrites );
    
    CAN_Close( canHandle );
    
    return 0;*/
}
