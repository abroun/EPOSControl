//------------------------------------------------------------------------------
// File: CANChannel.h
// Desc: An object for communicating with EPOS motor controllers on a CAN Bus.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_CHANNEL_H
#define CAN_CHANNEL_H

//------------------------------------------------------------------------------
#include "Common.h"

//------------------------------------------------------------------------------
struct CANChannelImpl;

//------------------------------------------------------------------------------
class CANChannel
{
    public: CANChannel();
    public: ~CANChannel();
    
    public: bool Init( const char* canDevice, eBaudRate baudRate );
    public: void Deinit();
    
    public: void OnCANOpenHeartbeatError( U8 error );
    public: void OnCANOpenInitialisation();
    public: void OnCANOpenPreOperational();
    public: void OnCANOpenOperational();
    public: void OnCANOpenStopped();
    public: void OnCANOpenPostSync();
    public: void OnCANOpenPostTPDO();
    public: void OnCANOpenPostEmergency( U8 nodeId, U16 errCode, U8 errReg );
    public: void OnCANOpenPostSlaveBootup( U8 nodeId );
    
    private: CANChannelImpl* mpImpl;
    private: bool mbInitialised;
};

#endif // CAN_CHANNEL_H