//------------------------------------------------------------------------------
// File: Common.h
// Desc: Common typedefs, structs and enums for the CAN Control library.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef EPOS_CONTROL_COMMON_H
#define EPOS_CONTROL_COMMON_H

//------------------------------------------------------------------------------
#define COMPILE_TIME_ASSERT( exp ) extern int gTestArray##__LINE__[ (exp) ? 1 : -1 ];
#define ARRAY_LENGTH( a ) ( sizeof( a )/sizeof( a[ 0 ] ) )

//------------------------------------------------------------------------------
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned int U32;
typedef char S8;
typedef short S16;
typedef int S32;

//------------------------------------------------------------------------------
enum eBaudRate
{
    eBR_1M = 0,
    eBR_500K,
    eBR_250K,
    eBR_125K,
    eBR_100K,
    eBR_50K,
    eBR_20K,
    eBR_10K,
    eBR_5K,
    eBR_NumBaudRates
};

//------------------------------------------------------------------------------
#define MAX_NUM_CAN_CHANNELS 2
#define MASTER_NODE_ID 25

//------------------------------------------------------------------------------
enum eNMT_State
{
    eNMTS_Unknown,
    eNMTS_Initialisation,
    eNMTS_PreOperational,
    eNMTS_Operational,
    eNMTS_Stopped
};

#endif // EPOS_CONTROL_COMMON_H