//------------------------------------------------------------------------------
// File: CANMotorController.cpp
// Desc: The object that represents an EPOS motor controller on a CAN bus.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <string.h>
#include "EPOSControl/CANMotorController.h"

//------------------------------------------------------------------------------
CANMotorController::CANMotorController()
    : mbInitialised( false )
{
}

//------------------------------------------------------------------------------
CANMotorController::~CANMotorController()
{
    Deinit();
}
    
//------------------------------------------------------------------------------
bool CANMotorController::Init( U8 nodeId )
{
    assert( !mbInitialised || mNodeId == nodeId );  // Init should not be called multiple times with different node Ids
    
    if ( !mbInitialised )
    {
        mNodeId = nodeId;
        mSDOQueueLength = 0;
        mbInitialised = true;
    }
    
    return mbInitialised;
}

//------------------------------------------------------------------------------
void CANMotorController::Deinit()
{
    mbInitialised = false;
}

//------------------------------------------------------------------------------
void CANMotorController::AddSDOField( const SDOField& field )
{
    if ( mbInitialised )
    {
        assert( mSDOQueueLength < MAX_SDO_QUEUE_LENGTH );
        if ( mSDOQueueLength < MAX_SDO_QUEUE_LENGTH )
        {
            mSDOQueue[ mSDOQueueLength ] = field;
            mSDOQueueLength++;
        }
    }
}

//------------------------------------------------------------------------------
void CANMotorController::OnSDOReadComplete( const SDOField& readField )
{
    if ( mbInitialised )
    {
        // Check that we're actually expecting an SDO read
        assert( mSDOQueueLength > 0 );
        assert( SDOField::eT_Read == mSDOQueue[ 0 ].mType );
        
        if ( mSDOQueueLength > 0
            && SDOField::eT_Read == mSDOQueue[ 0 ].mType )
        {
            memcpy( mSDOQueue[ 0 ].mData, readField.mData, readField.mNumBytes );
            mSDOQueue[ 0 ].mNumBytes = readField.mNumBytes;
            
            // Call the user callback if available
            if ( NULL != mSDOQueue[ 0 ].mReadCallback )
            {
                mSDOQueue[ 0 ].mReadCallback( mSDOQueue[ 0 ] );
            }
            
            AdvanceSDOQueue();
        }
    }
}
    
//------------------------------------------------------------------------------
void CANMotorController::OnSDOWriteComplete()
{
    if ( mbInitialised )
    {
        // Check that we're actually expecting an SDO write
        assert( mSDOQueueLength > 0 );
        assert( SDOField::eT_Write == mSDOQueue[ 0 ].mType );
        
        if ( mSDOQueueLength > 0
            && SDOField::eT_Write == mSDOQueue[ 0 ].mType )
        {
            AdvanceSDOQueue();
        }
    }
}
    
//------------------------------------------------------------------------------
const SDOField* CANMotorController::GetSDOQueueHead() const
{
    const SDOField* pResult = NULL;
    
    if ( mbInitialised )
    {
        if ( mSDOQueueLength > 0 )
        {
            pResult = &mSDOQueue[ 0 ];
        }
    }
    
    return pResult;
}

//------------------------------------------------------------------------------
void CANMotorController::AdvanceSDOQueue()
{
    if ( mbInitialised 
        && mSDOQueueLength > 0 )
    {
        // Very inefficient update of the queue by copying all fields forward
        // one space.
        memcpy( &mSDOQueue[ 0 ], &mSDOQueue[ 1 ], 
            sizeof( SDOField )*(mSDOQueueLength-1) );
        mSDOQueueLength--;
    }
}