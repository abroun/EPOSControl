//------------------------------------------------------------------------------
// File: CANMotorControllerAction.cpp
// Desc: Represents a very basic action that can be done with a motor
//       controller. A motor controller configuration is built up from a number
//       of actions.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include "EPOSControl/CANMotorControllerAction.h"

//------------------------------------------------------------------------------
CANMotorControllerAction::CANMotorControllerAction()
    : mType( eT_Invalid )
{
}

//------------------------------------------------------------------------------
CANMotorControllerAction CANMotorControllerAction::CreateEnsureNMTStateAction( const EnsureNMTState& ensureState )
{
    CANMotorControllerAction action;
    
    action.mType = eT_EnsureNMTState;
    action.mEnsureNMTState = ensureState;
    
    return action;
}

//------------------------------------------------------------------------------
CANMotorControllerAction CANMotorControllerAction::CreateSDOFieldAction( const SDOField& field )
{
    CANMotorControllerAction action;
    
    action.mType = eT_SDOField;
    action.mSDOField = field;
    
    return action;
}

