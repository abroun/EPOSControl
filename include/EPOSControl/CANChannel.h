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
struct AngleData
{
    U8 mNodeId;
    S32 mAngle;     // Angle in encoder tick
};

//------------------------------------------------------------------------------
class CANChannel
{
    //--------------------------------------------------------------------------
    public: CANChannel();
    public: ~CANChannel();
    
    //--------------------------------------------------------------------------
    public: bool Init( const char* canDevice, eBaudRate baudRate );
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
    // Returns the list of angles for all nodes which have recieved an angle
    // from their EPOS motor controller
    public: void GetMotorAngles( AngleData* pAngleBuffer, S32* pBufferSizeOut );
    
    public: void SetMotorAngle( U8 nodeId, S32 angle );
    public: void SendFaultReset( U8 nodeId );
    
    //--------------------------------------------------------------------------
    public: static const char* GetEposErrorMessage( U16 errCode, U8 errReg );
    
    //--------------------------------------------------------------------------
    public: static const U8 ALL_MOTOR_CONTROLLERS = 0;
    public: static const U8 MAX_NUM_MOTOR_CONTROLLERS = 128;
    private: CANMotorController mMotorControllers[ MAX_NUM_MOTOR_CONTROLLERS ];
    private: bool mbInitialised;
    private: U8 mStartingNodeId;   // See OnCANUpdate for explanation
    
    private: S32 mFrameIdx;
    public: S32 GetFrameIdx() const { return mFrameIdx; }
};

#endif // CAN_CHANNEL_H