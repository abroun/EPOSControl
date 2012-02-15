//------------------------------------------------------------------------------
// File: CANOpenInterface.h
// Desc: Provides access to the whichever library we're using for the
//       CAN Open protocol.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_OPEN_INTERFACE_h
#define CAN_OPEN_INTERFACE_h

//------------------------------------------------------------------------------
#include "EPOSControl/Common.h"
#include "EPOSControl/CANChannel.h"
#include "EPOSControl/SDOField.h"

//------------------------------------------------------------------------------
bool COI_InitCANOpenInterface();
void COI_DeinitCANOpenInterface();

//------------------------------------------------------------------------------
bool COI_InitCANChannel( CANChannel* pChannel,
    const char* driverLibraryName, const char* canDevice, eBaudRate baudRate );
void COI_DeinitCANChannel( CANChannel* pChannel );

//------------------------------------------------------------------------------
bool COI_ProcessSDOField( CANChannel* pChannel, U8 nodeId, const SDOField& field );

#endif // CAN_OPEN_INTERFACE_h
