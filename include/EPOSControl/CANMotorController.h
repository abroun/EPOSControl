//------------------------------------------------------------------------------
// File: CANMotorController.h
// Desc: The object that represents an EPOS motor controller on a CAN bus.
//
//       A CANMotorController is configured and controlled using sequences of
//       CANMotorControllerActions. 'Configuration' actions are assumed to be a
//       persistant list of tasks that should be carried out to configure
//       a motor controller. They are kept around in case a motor controller
//       needs to be restarted for whatever reason and reconfigured.
//
//       'Extra' actions are temporary one-shot actions that are intended to
//       be used to modify the motor controller on the fly. If there are
//       configuration options that change back and forth for example.
//
//       This design is in flux however, it may be that 'extra' actions are
//       an unnecessary distinction and that all 'on the fly' configuration
//       will be done by PDO.
//
//       Whilst we keep them though, the motor controller will keep executing
//       a particular type of action until no more remain. Then if it is not 
//       executing any actions and has a choice between configuration and
//       extra actions, configuration actions have priority.
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_MOTOR_CONTROLLER_H
#define CAN_MOTOR_CONTROLLER_H

//------------------------------------------------------------------------------
#include "Common.h"
#include "CANMotorControllerAction.h"

//------------------------------------------------------------------------------
class CANChannel;

//------------------------------------------------------------------------------
class CANMotorController
{
    //--------------------------------------------------------------------------
    public: enum eState
    {
        eS_Inactive,
        eS_SettingUp,
        eS_Running,
        eS_Homing,
        
        eS_ProcessingConfigurationActions,
        eS_ProcessingExtraActions,
    };
    
    //--------------------------------------------------------------------------
    public: enum eConfiguration
    {
        eC_None,
        eC_PositionControl
    };
    
    //--------------------------------------------------------------------------
    public: enum eRunningTask
    {
        eRT_None,
        eRT_SetDesiredAngle,
        eRT_SendFaultReset,
        eRT_SetProfileVelocity,
        eRT_SetMaximumFollowingError
    };
    
    //--------------------------------------------------------------------------
    public: CANMotorController();
    public: ~CANMotorController();
    
    //--------------------------------------------------------------------------
    public: bool Init( CANChannel* pOwner, U8 nodeId );
    public: void Deinit();
    public: bool IsInitialised() const { return mbInitialised; }
    public: U8 GetNodeId() const { return mNodeId; }
    public: eState GetState() const { return mState; }
    public: eConfiguration GetConfiguration() const { return mConfiguration; }
    
    //--------------------------------------------------------------------------
    // This will start returning true when evidence is received that the
    // physical motor controller is present. At the moment this evidence is
    // being told about the NMT PreOperational state when a node starts up.
    public: bool IsPresent() const { return mbPresent; }
    
    //--------------------------------------------------------------------------
    public: void Update( S32 frameIdx );
    
    //--------------------------------------------------------------------------
    // Lets the CANMotorController object know that the real world motor 
    // controller is in a known NMT state
    public: void TellAboutNMTState( eNMT_State state );
    public: eNMT_State GetLastKnownNMTState() const { return mLastKnownNMTState; }
  
    public: void OnSDOFieldWriteComplete( S32 frameIdx );
    public: void OnSDOFieldReadComplete( U8* pData, U32 numBytes );
  
    public: bool IsAngleValid() const { return mbInitialised && mbAngleValid; }
    public: S32 GetAngle() const { return mAngle; }
    
    // Commands for controlling the motor controller in the eS_Running state.
    // NOTE: These routines are _not_ thread safe with the update routine
    public: void SetDesiredAngle( S32 desiredAngle, S32 frameIdx );
    public: void SetProfileVelocity( U32 profileVelocity );
    public: void SetMaximumFollowingError( U32 maximumFollowingError );
    public: void SendFaultReset();
    
    //--------------------------------------------------------------------------
    // The SDO communication state machine is used to keep track of an SDO read
    // or write
    public: enum eSdoCommunicationState
    {
        eSCS_Inactive,
        eSCS_Active,
        eSCS_NumSdoCommunicationStates
    };
  
    //--------------------------------------------------------------------------
    public: void SetConfiguration( eConfiguration configuration );
    
    //--------------------------------------------------------------------------
    private: bool ProcessSDOWrite( const SDOField& sdoField, bool bDebug=false );
    
    //--------------------------------------------------------------------------
    private: static void HandleSDOReadComplete( SDOField& field );
    
    //--------------------------------------------------------------------------
    public: static const S32 CONFIGURATION_ACTION_LIST_LENGTH = 64;
    public: static const S32 EXTRA_ACTION_LIST_LENGTH = 16;
    
    private: bool mbInitialised;
    private: CANChannel* mpOwner;
    private: U8 mNodeId;
    private: bool mbPresent;
    
    private: eNMT_State mLastKnownNMTState;
    private: eSdoCommunicationState mSdoReadState;
    private: eSdoCommunicationState mSdoWriteState;
    private: SDOField* mpActiveSdoReadField;
    private: SDOField mReadAction;
    private: SDOField mReadStatusAction;
    private: eState mState;
    private: eConfiguration mConfiguration;
    private: eRunningTask mRunningTask;
    private: bool mbAngleValid;
    private: S32 mAngle;
    
    private: bool mbFaultResetRequested;
    private: bool mbNewDesiredAngleRequested;
    private: bool mbNewProfileVelocityRequested;
    private: bool mbNewMaximumFollowingErrorRequested;
    
    private: bool mbStatusValid;
    private: U16 mEposStatusword;
    private: S32 mLastStatusPollFrameIdx;
    
    private: S32 mNewDesiredAngle;
    private: S32 mNewProfileVelocity;
    private: U32 mNewMaximumFollowingError;
    private: S32 mSDOWriteFrameIdx;
    
    private: const SDOField* mpConfigurationSetupCommands;
    private: S32 mCurConfigurationSetupCommandIdx;
    
    private: const SDOField* mpRunningTaskCommands;
    private: S32 mCurRunningTaskCommandIdx;
    
    private: SDOField mSetDesiredAngleCommands[ 2 + 1 ];
    private: SDOField mSetProfileVelocityCommands[ 1 + 1 ];
    private: SDOField mSetMaxFollowingErrorCommands[ 1 + 1 ];
    
    private: static const SDOField POSITION_CONTROL_SETUP_COMMANDS[];
    private: static const SDOField FAULT_RESET_COMMANDS[];
};

#endif // CAN_MOTOR_CONTROLLER_H
