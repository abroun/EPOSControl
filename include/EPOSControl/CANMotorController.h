//------------------------------------------------------------------------------
// File: CANMotorController.h
// Desc: The object that represents an EPOS motor controller on a CAN bus.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_MOTOR_CONTROLLER_H
#define CAN_MOTOR_CONTROLLER_H

//------------------------------------------------------------------------------
#include "Common.h"
#include "SDOField.h"

//------------------------------------------------------------------------------
class CANMotorController
{
    //--------------------------------------------------------------------------
    public: CANMotorController();
    public: ~CANMotorController();
    
    //--------------------------------------------------------------------------
    public: bool Init( U8 nodeId );
    public: void Deinit();
    public: bool IsInitialised() const { return mbInitialised; }
    public: U8 GetNodeId() const { return mNodeId; }
    
    //--------------------------------------------------------------------------
    // Adds an SDO field to the queue of fields to be read or written to or
    // from the physical motor controller. If the field is to be read then a 
    // callback should be set on the field before it is passed in
    public: void AddSDOField( const SDOField& field );
    
    //--------------------------------------------------------------------------
    // This routine is called by the owning CANChannel when an SDO read from
    // the physical motor controller is complete in order to give the motor 
    // controller object the read data.
    public: void OnSDOReadComplete( const SDOField& readField );
    
    //--------------------------------------------------------------------------
    // Lets the motor controller object know that an SDO write has been
    // completed.
    public: void OnSDOWriteComplete();
    
    //--------------------------------------------------------------------------
    // Gets the field currently waiting to be read or written from the motor
    // controller. Returns NULL if nothing is waiting to be read or written.
    public: const SDOField* GetSDOQueueHead() const;
    
    //--------------------------------------------------------------------------
    // Moves the SDO queue forward one
    private: void AdvanceSDOQueue();
    
    //--------------------------------------------------------------------------
    public: static const S32 MAX_SDO_QUEUE_LENGTH = 64;
    
    private: bool mbInitialised;
    private: U8 mNodeId;
    private: SDOField mSDOQueue[ MAX_SDO_QUEUE_LENGTH ];
    private: S32 mSDOQueueLength;
};

#endif // CAN_MOTOR_CONTROLLER_H