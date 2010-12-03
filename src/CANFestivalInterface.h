//------------------------------------------------------------------------------
// File: CANFestivalInterface.h
// Desc: Provides access to the CanFestival library which we're using for the
//       CAN Open protocol.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef CAN_FESTIVAL_INTERFACE_h
#define CAN_FESTIVAL_INTERFACE_h

//------------------------------------------------------------------------------
#include "EPOSControl/Common.h"
#include "EPOSControl/CANChannel.h"
#include "EPOSControl/SDOField.h"

//------------------------------------------------------------------------------
bool CFI_InitCANFestivalInterface();
void CFI_DeinitCANFestivalInterface();

//------------------------------------------------------------------------------
bool CFI_InitCANChannel( CANChannel* pChannel, const char* canDevice, eBaudRate baudRate );
void CFI_DeinitCANChannel( CANChannel* pChannel );

//------------------------------------------------------------------------------
bool CFI_ProcessSDOField( CANChannel* pChannel, U8 nodeId, SDOField& field );

//------------------------------------------------------------------------------
void CFI_EnterMutex();
void CFI_LeaveMutex();

#endif // CAN_FESTIVAL_INTERFACE_h