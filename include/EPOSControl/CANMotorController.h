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
        eS_ProcessingConfigurationActions,
        eS_ProcessingExtraActions,
    };
    
    //--------------------------------------------------------------------------
    public: CANMotorController();
    public: ~CANMotorController();
    
    //--------------------------------------------------------------------------
    public: bool Init( CANChannel* pOwner, U8 nodeId );
    public: void Deinit();
    public: bool IsInitialised() const { return mbInitialised; }
    public: U8 GetNodeId() const { return mNodeId; }
    
    //--------------------------------------------------------------------------
    // This will start returning true when evidence is received that the
    // physical motor controller is present. At the moment this evidence is
    // being told about the NMT PreOperational state when a node starts up.
    public: bool IsPresent() const { return mbPresent; }
    
    //--------------------------------------------------------------------------
    public: void Update();
    
    //--------------------------------------------------------------------------
    // Lets the CANMotorController object know that the real world motor 
    // controller is in a known NMT state
    public: void TellAboutNMTState( eNMT_State state );
    public: eNMT_State GetLastKnownNMTState() const { return mLastKnownNMTState; }
  
    //--------------------------------------------------------------------------
    // The communication state machine is used to keep track of communications
    // with the motor controller, trying to ensure that that the motor 
    // controller is in
    public: enum eCommunicationState
    {
    };
  
    //--------------------------------------------------------------------------
    // Adds an action to the configuration actions for the motor controller
    public: void AddConfigurationAction( const CANMotorControllerAction& action );
    
    //--------------------------------------------------------------------------
    public: void ClearConfiguration();
    
    //--------------------------------------------------------------------------
    // Adds an action to the list of extra actions for the motor controller
    public: void AddExtraAction( const CANMotorControllerAction& action );
    
//     //--------------------------------------------------------------------------
//     // This routine is called by the owning CANChannel when an SDO read from
//     // the physical motor controller is complete in order to give the motor 
//     // controller object the read data.
//     public: void OnSDOReadComplete( const SDOField& readField );
//     
//     //--------------------------------------------------------------------------
//     // Lets the motor controller object know that an SDO write has been
//     // completed.
//     public: void OnSDOWriteComplete();
//     
//     //--------------------------------------------------------------------------
//     // Gets the field currently waiting to be read or written from the motor
//     // controller. Returns NULL if nothing is waiting to be read or written.
//     public: const SDOField* GetSDOQueueHead() const;
//     
//     //--------------------------------------------------------------------------
//     // Moves the SDO queue forward one
//     private: void AdvanceSDOQueue();
    
    //--------------------------------------------------------------------------
    // Helper routine for internal state
    private: void ProcessConfigurationAction();
    
    //--------------------------------------------------------------------------
    // Helper routine for internal state
    private: void ProcessExtraAction();
    
    //--------------------------------------------------------------------------
    public: static const S32 CONFIGURATION_ACTION_LIST_LENGTH = 64;
    public: static const S32 EXTRA_ACTION_LIST_LENGTH = 16;
    
    private: bool mbInitialised;
    private: CANChannel* mpOwner;
    private: U8 mNodeId;
    private: bool mbPresent;
    
    private: CANMotorControllerAction mConfigurationActionList[ CONFIGURATION_ACTION_LIST_LENGTH ];
    private: CANMotorControllerAction mExtraActionList[ EXTRA_ACTION_LIST_LENGTH ];
    private: S32 mNumConfigurationActions;
    private: S32 mNumExtraActions;
    private: S32 mNumConfigurationActionsDone;
    
    private: eNMT_State mLastKnownNMTState;
    private: eState mState;
};

#endif // CAN_MOTOR_CONTROLLER_H