//------------------------------------------------------------------------------
// File: EPOSControl.h
// Desc: A library for communicating with and configuring EPOS motor controllers
//       made by Maxon motors using CAN bus.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef EPOS_CONTROL_H
#define EPOS_CONTROL_H

//------------------------------------------------------------------------------
#include "Common.h"
#include "CANChannel.h"

//------------------------------------------------------------------------------
bool EPOS_InitLibaray();
void EPOS_DeinitLibrary();

CANChannel* EPOS_OpenCANChannel( const char* canDevice, eBaudRate baudRate );
void EPOS_CloseCANChannel( CANChannel* pChannel );

//------------------------------------------------------------------------------
void EPOS_EnterCANMutex();
void EPOS_LeaveCANMutex();

#endif // EPOS_CONTROL_H



