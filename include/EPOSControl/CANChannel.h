//------------------------------------------------------------------------------
// File: CANChannel.h
// Desc: An object for communicating with EPOS motor controllers on a CAN Bus.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_CHANNEL_H
#define CAN_CHANNEL_H

//------------------------------------------------------------------------------
#include "Common.h"
#include "EPOSControl/CANMotorController.h"

//------------------------------------------------------------------------------
struct MotorControllerData
{
    U8 mNodeId;
    S32 mState;
    S32 mAngle;     // Angle in encoder tick
    bool mbAngleValid;
};

//------------------------------------------------------------------------------
class CANChannel
{
    //--------------------------------------------------------------------------
    public: CANChannel();
    public: ~CANChannel();
    
    //--------------------------------------------------------------------------
    public: bool Init( const char* driverLibraryName, const char* canDevice, eBaudRate baudRate, S32 channelIdx=0 );
    public: void Deinit();
    
    //--------------------------------------------------------------------------
    // Callbacks used by the CANOpen library
    public: void OnCANOpenHeartbeatError( U8 error );
    public: void OnCANOpenInitialisation();
    public: void OnCANOpenPreOperational();
    public: void OnCANOpenOperational();
    public: void OnCANOpenStopped();
    public: void OnCANOpenPostSync();
    public: void OnCANOpenPostTPDO();
    public: void OnCANOpenPostEmergency( U8 nodeId, U16 errCode, U8 errReg );
    public: void OnCANOpenPostSlaveBootup( U8 nodeId );
    public: void OnSDOFieldWriteComplete( U8 nodeId );
    public: void OnSDOFieldReadComplete( U8 nodeId, U8* pData, U32 numBytes );
    
    //--------------------------------------------------------------------------
    public: void Update();
    
    //--------------------------------------------------------------------------
    public: void ConfigureAllMotorControllersForPositionControl();
    
    //--------------------------------------------------------------------------
    // Gets information about all of the EPOS motor controllers
    public: void GetMotorControllerData( MotorControllerData* pDataBuffer, S32* pBufferSizeOut );
    
    public: void SetMotorAngle( U8 nodeId, S32 angle );
    public: void SetMotorProfileVelocity( U8 nodeId, U32 velocity );
    public: void SetMaximumFollowingError( U8 nodeId, U32 maximumFollowingError );
    public: void SendFaultReset( U8 nodeId );
    
    //--------------------------------------------------------------------------
    public: static const char* GetEposErrorMessage( U16 errCode, U8 errReg );
    
    //--------------------------------------------------------------------------
    public: S32 GetFrameIdx() const { return mFrameIdx; }
    public: S32 GetChannelIdx() const { return mChannelIdx; }

    //--------------------------------------------------------------------------
    public: static const U8 ALL_MOTOR_CONTROLLERS = 0;
    public: static const U8 MAX_NUM_MOTOR_CONTROLLERS = 128;
    private: CANMotorController mMotorControllers[ MAX_NUM_MOTOR_CONTROLLERS ];
    private: bool mbInitialised;
    private: U8 mStartingNodeId;    // See OnCANUpdate for explanation
    
    private: S32 mFrameIdx;
    private: S32 mChannelIdx;       // Lets client code distinguish between channels
};

#endif // CAN_CHANNEL_H
