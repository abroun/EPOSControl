//------------------------------------------------------------------------------
// File: CANMotorControllerAction.h
// Desc: Represents a very basic action that can be done with a motor
//       controller. A motor controller configuration is built up from a number
//       of actions.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_MOTOR_CONTROLLER_ACTION_H
#define CAN_MOTOR_CONTROLLER_ACTION_H

//------------------------------------------------------------------------------
#include "Common.h"
#include "SDOField.h"

//------------------------------------------------------------------------------
struct EnsureNMTState
{
    enum eType
    {
        eT_Passive, // Waits for the state to be set
        eT_Active,  // Tries to set the state if it's not set
    };
    
    EnsureNMTState( eType type, eNMT_State desiredState )
        : mType( type ), mDesiredState( desiredState ) {}
    
    eType mType;
    eNMT_State mDesiredState;
};

//------------------------------------------------------------------------------
struct CANMotorControllerAction
{
    enum eType
    {
        eT_Invalid = -1,
        eT_EnsureNMTState,
        eT_SDOField,
        eT_NumTypes
    };
    
    CANMotorControllerAction();
    
    static CANMotorControllerAction CreateEnsureNMTStateAction( EnsureNMTState& ensureState );
    static CANMotorControllerAction CreateSDOFieldAction( SDOField& field );
    
    eType mType;
    union
    {
        EnsureNMTState mEnsureNMTState;
        SDOField mSDOField;
    } mData;
};

#endif // CAN_MOTOR_CONTROLLER_ACTION_H