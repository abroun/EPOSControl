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
bool EPOS_InitLibrary();
void EPOS_DeinitLibrary();

// Opens a channel. If channelIdx is greater than or equal to 0 then the library attempts
// to open at channel at that slot
CANChannel* EPOS_OpenCANChannel( const char* driverLibraryName, const char* canDevice,
                                 eBaudRate baudRate, S32 channelIdx=-1 );
void EPOS_CloseCANChannel( CANChannel* pChannel );

#endif // EPOS_CONTROL_H



