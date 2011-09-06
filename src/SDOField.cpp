//------------------------------------------------------------------------------
// File: SDOField.cpp
// Desc: An object that configures either a read or a write of an SDO field
//       from a CAN Open node
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <string.h>
#include "EPOSControl/SDOField.h"

//------------------------------------------------------------------------------
SDOField::SDOField()
    : mType( eT_Write ),
    mIndex( 0 ),
    mSubIndex( 0 ),
    mReadCallback( NULL ),
    mpUserData( NULL )
{
    mDescription[ 0 ] = '\0';
}        

//------------------------------------------------------------------------------
SDOField::SDOField( eType type, const char* pDescription, U16 index, U8 subIndex,
              SDOReadCallback readCallback, void* pUserData )
    : mType( type ),
    mIndex( index ),
    mSubIndex( subIndex ),
    mReadCallback( readCallback ),
    mpUserData( pUserData )
{
    // Copy description into buffer and make sure that it's NUL terminated.
    strncpy( mDescription, pDescription, sizeof( mDescription ) );
    mDescription[ sizeof( mDescription ) - 1 ] = '\0';
}
