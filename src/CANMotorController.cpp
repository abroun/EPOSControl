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
bool CANMotorController::Init( CANChannel* pOwner, U8 nodeId )
{
    assert( !mbInitialised || mNodeId == nodeId );  // Init should not be called multiple times with different node Ids
    assert( NULL != pOwner );
    
    if ( !mbInitialised )
    {
        mpOwner = pOwner;
        mNodeId = nodeId;
        
        mNumConfigurationActions = 0;
        mNumExtraActions = 0;
        mNumConfigurationActionsDone = 0;
    
        mLastKnownNMTState = eNMTS_Unknown;
        mState = eS_Inactive;
        mbPresent = false;
        
        mbInitialised = true;
    }
    
    return mbInitialised;
}

//------------------------------------------------------------------------------
void CANMotorController::Deinit()
{
    mLastKnownNMTState = eNMTS_Unknown;
    mbInitialised = false;
}

//------------------------------------------------------------------------------
void CANMotorController::Update()
{
    if ( mbPresent )
    {
        switch ( mState )
        {
            case eS_Inactive:
            {
                // Check to see if we have any actions to execute
                if ( mNumConfigurationActions > mNumConfigurationActionsDone )
                {
                    mState = eS_ProcessingConfigurationActions;
                }
                else if ( mNumExtraActions > 0 )
                {
                    mState = eS_ProcessingExtraActions;
                }
                break;
            }
            case eS_ProcessingConfigurationActions:
            {
                ProcessConfigurationAction();
                break;
            }
            case eS_ProcessingExtraActions:
            {
                ProcessExtraAction();
                break;
            }
            default:
            {
                assert( false && "Unhandled state encountered" );
            }
        }
    }
}

//------------------------------------------------------------------------------
void CANMotorController::TellAboutNMTState( eNMT_State state )
{
    if ( mbInitialised )
    {
        mLastKnownNMTState = state;
        if ( eNMTS_PreOperational == state )
        {
            mbPresent = true;
        }
    }
}

//------------------------------------------------------------------------------
void CANMotorController::AddConfigurationAction( const CANMotorControllerAction& action )
{
    if ( mbInitialised )
    {
        assert( mNumConfigurationActions < CONFIGURATION_ACTION_LIST_LENGTH );
        if ( mNumConfigurationActions < CONFIGURATION_ACTION_LIST_LENGTH )
        {
            mConfigurationActionList[ mNumConfigurationActions ] = action;
            mNumConfigurationActions++;
        }
    }
}
    
//------------------------------------------------------------------------------
void CANMotorController::ClearConfiguration()
{
    mNumConfigurationActions = 0;
    mNumConfigurationActionsDone = 0;
    
    if ( eS_ProcessingConfigurationActions == mState )
    {
        mState = eS_Inactive;
    }
}
    
//------------------------------------------------------------------------------
void CANMotorController::AddExtraAction( const CANMotorControllerAction& action )
{
    if ( mbInitialised )
    {
        assert( mNumExtraActions < EXTRA_ACTION_LIST_LENGTH );
        if ( mNumExtraActions < EXTRA_ACTION_LIST_LENGTH )
        {
            mExtraActionList[ mNumExtraActions ] = action;
            mNumExtraActions++;
        }
    }
}

//------------------------------------------------------------------------------
void CANMotorController::ProcessConfigurationAction()
{
    // TODO: write a common routine for processing an action
}
    
//------------------------------------------------------------------------------
void CANMotorController::ProcessExtraAction()
{
}

//------------------------------------------------------------------------------
// void CANMotorController::AddSDOField( const SDOField& field )
// {
//     if ( mbInitialised )
//     {
//         assert( mSDOQueueLength < MAX_SDO_QUEUE_LENGTH );
//         if ( mSDOQueueLength < MAX_SDO_QUEUE_LENGTH )
//         {
//             mSDOQueue[ mSDOQueueLength ] = field;
//             mSDOQueueLength++;
//         }
//     }
// }

//------------------------------------------------------------------------------
//void CANMotorController::OnSDOReadComplete( const SDOField& readField )
// {
//     if ( mbInitialised )
//     {
//         // Check that we're actually expecting an SDO read
//         assert( mSDOQueueLength > 0 );
//         assert( SDOField::eT_Read == mSDOQueue[ 0 ].mType );
//         
//         if ( mSDOQueueLength > 0
//             && SDOField::eT_Read == mSDOQueue[ 0 ].mType )
//         {
//             memcpy( mSDOQueue[ 0 ].mData, readField.mData, readField.mNumBytes );
//             mSDOQueue[ 0 ].mNumBytes = readField.mNumBytes;
//             
//             // Call the user callback if available
//             if ( NULL != mSDOQueue[ 0 ].mReadCallback )
//             {
//                 mSDOQueue[ 0 ].mReadCallback( mSDOQueue[ 0 ] );
//             }
//             
//             AdvanceSDOQueue();
//         }
//     }
// }
//     
// //------------------------------------------------------------------------------
// void CANMotorController::OnSDOWriteComplete()
// {
//     if ( mbInitialised )
//     {
//         // Check that we're actually expecting an SDO write
//         assert( mSDOQueueLength > 0 );
//         assert( SDOField::eT_Write == mSDOQueue[ 0 ].mType );
//         
//         if ( mSDOQueueLength > 0
//             && SDOField::eT_Write == mSDOQueue[ 0 ].mType )
//         {
//             AdvanceSDOQueue();
//         }
//     }
// }
//     
// //------------------------------------------------------------------------------
// const SDOField* CANMotorController::GetSDOQueueHead() const
// {
//     const SDOField* pResult = NULL;
//     
//     if ( mbInitialised )
//     {
//         if ( mSDOQueueLength > 0 )
//         {
//             pResult = &mSDOQueue[ 0 ];
//         }
//     }
//     
//     return pResult;
// }
// 
// //------------------------------------------------------------------------------
// void CANMotorController::AdvanceSDOQueue()
// {
//     if ( mbInitialised 
//         && mSDOQueueLength > 0 )
//     {
//         // Very inefficient update of the queue by copying all fields forward
//         // one space.
//         memcpy( &mSDOQueue[ 0 ], &mSDOQueue[ 1 ], 
//             sizeof( SDOField )*(mSDOQueueLength-1) );
//         mSDOQueueLength--;
//     }
// }