//------------------------------------------------------------------------------
// File: CANMotorController.cpp
// Desc: The object that represents an EPOS motor controller on a CAN bus.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "EPOSControl/CANMotorController.h"
#include "CANOpenInterface.h"

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
        mCommunicationState = eCS_Inactive;
        mpActiveSDOField = NULL;
        mState = eS_Inactive;
        mbPresent = false;
        mbAngleValid = false;
        mLastAnglePollFrameIdx = 0;
        mbDesiredAngleValid = false;
        mbStatusValid = false;
        mLastStatusPollFrameIdx = 0;
        
        mReadAction = SDOField( SDOField::eT_Read, 
            "Position Actual", 0x6064, 0, HandleSDOReadComplete, this );
        mReadStatusAction = SDOField( SDOField::eT_Read, 
            "Statusword", 0x6041, 0, HandleSDOReadComplete, this );
        
        mbInitialised = true;
    }
    
    return mbInitialised;
}

//------------------------------------------------------------------------------
void CANMotorController::Deinit()
{
    mpActiveSDOField = NULL;
    mCommunicationState = eCS_Inactive;
    mLastKnownNMTState = eNMTS_Unknown;
    mbInitialised = false;
}

//------------------------------------------------------------------------------
void CANMotorController::Update( S32 frameIdx )
{
    if ( mbStatusValid 
        && ( ( mEposStatusword & 0x0008 ) ) ) //|| 3 == mNodeId ) )
    {
        printf( "Node %i status is 0x%X\n", mNodeId, mEposStatusword );
    }
    
    if ( mbPresent )
    {
        /*if ( GetNodeId() == 15 )
        {
            switch ( mState )
            {
                case eS_Inactive:
                {
                    printf( "Node %i in eS_Inactive\n", GetNodeId() );
                    break;
                }
                case eS_ProcessingConfigurationActions:
                {
                    printf( "Node %i in eS_ProcessingConfigurationActions\n", GetNodeId() );
                    break;
                }
                case eS_ProcessingExtraActions:
                {
                    printf( "Node %i in eS_ProcessingExtraActions\n", GetNodeId() );
                    break;
                }
                case eS_PollingForPosition:
                {
                    printf( "Node %i in eS_PollingForPosition\n", GetNodeId() );
                    break;
                }
                default:
                {
                    assert( false && "Unhandled state encountered" );
                }
            }
        }*/ 
        
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
                else
                {
                    mState = eS_PollingForPosition;
                }
                break;
            }
            case eS_ProcessingConfigurationActions:
            {
                ProcessConfigurationAction();
                if ( mNumConfigurationActionsDone >= mNumConfigurationActions )
                {
                    mState = eS_Inactive;
                }
                break;
            }
            case eS_ProcessingExtraActions:
            {
                ProcessExtraAction();
                if ( mNumExtraActions <= 0 )
                {
                    //printf( "Node %i finished with extra actions after %i frames\n", 
                    //        GetNodeId(), frameIdx - mExtraFrameIdx );
                    mState = eS_Inactive;
                }
                break;
            }
            case eS_PollingForPosition:
            {
                switch ( mCommunicationState )
                {
                    case eCS_Inactive:
                    {
                        assert( NULL == mpActiveSDOField );
                        
                        if ( mNumConfigurationActions > mNumConfigurationActionsDone )
                        {
                            mState = eS_ProcessingConfigurationActions;
                        }
                        else if ( mNumExtraActions > 0 )
                        {
                            mState = eS_ProcessingExtraActions;
                        }
                        else 
                        {
                            // Nothing else to do so poll
                            if ( !mbAngleValid
                                || ( frameIdx - mLastAnglePollFrameIdx > 200 ) )    // Poll at 5Hz TODO: Make this nicer
                            {
                                if ( COI_ProcessSDOField( mpOwner, mNodeId, mReadAction ) )
                                {
                                    mpActiveSDOField = &mReadAction;
                                    mCommunicationState =  eCS_WaitingForSDORead;
                                    mLastAnglePollFrameIdx = frameIdx;
                                }
                            }
                            else if ( !mbStatusValid
                                || ( frameIdx - mLastStatusPollFrameIdx > 5000 ) )    // Poll at 1Hz TODO: Make this nicer
                            {
                                if ( COI_ProcessSDOField( mpOwner, mNodeId, mReadStatusAction ) )
                                {
                                    mpActiveSDOField = &mReadStatusAction;
                                    mCommunicationState =  eCS_WaitingForSDORead;
                                    mLastStatusPollFrameIdx = frameIdx;
                                }
                            }
                        }
                        
                        break;
                    }
                    case eCS_WaitingForSDORead:
                    {
                        // Nothing to do
                        break;
                    }
                    default:
                    {
                        assert( false && "Unhandled communication state" );
                    }
                }
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
void CANMotorController::OnSDOFieldWriteComplete( S32 frameIdx )
{
    assert( eCS_WaitingForSDOWrite == mCommunicationState );
    
    if ( eS_ProcessingConfigurationActions == mState )
    {
        assert( mNumConfigurationActionsDone < mNumConfigurationActions );
        mNumConfigurationActionsDone++;
    }
    else if ( eS_ProcessingExtraActions == mState )
    {
        //printf( "SDO Write took %i frames\n", frameIdx - mSDOWriteFrameIdx );
        RemoveFirstExtraAction();
    }
    else
    {
        assert( false && "Motor controller in unexpected state" );
    }
    
    mCommunicationState = eCS_Inactive;
    mpActiveSDOField = NULL;
}

//------------------------------------------------------------------------------
void CANMotorController::OnSDOFieldReadComplete( U8* pData, U32 numBytes )
{
    assert( eCS_WaitingForSDORead == mCommunicationState );
    assert( mpActiveSDOField != NULL );
    
    memcpy( mpActiveSDOField->mData, pData, numBytes );
    mpActiveSDOField->mReadCallback( *mpActiveSDOField );
    
    mCommunicationState = eCS_Inactive;
    mpActiveSDOField = NULL;
}

//------------------------------------------------------------------------------
void CANMotorController::SetDesiredAngle( S32 desiredAngle, S32 frameIdx )
{
    if ( mbDesiredAngleValid && mDesiredAngle == desiredAngle )
    {
       return;  // Already set
    }
    
    if ( mNumExtraActions > 0 )
    {
        // TODO: Fix this hack
        return;
    }
    
    CANMotorControllerAction action =
        CANMotorControllerAction::CreateSDOFieldAction(
            SDOField( SDOField::eT_Write, "Target Position", 0x607A, 0 ) );
    action.mSDOField.SetS32( desiredAngle );
    
    AddExtraAction( action );
    
    action = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Controlword", 0x6040, 0 ) );
    action.mSDOField.SetU16( 0x001F );    // Start positioning
    
    AddExtraAction( action );
    
    mbDesiredAngleValid = true;
    mDesiredAngle = desiredAngle;
    mExtraFrameIdx = frameIdx;
    
    //printf( "Added extra action for node %i to go to %i\n", GetNodeId(), desiredAngle );
}

//------------------------------------------------------------------------------
void CANMotorController::SendFaultReset()
{
    mbDesiredAngleValid = false;
    mbAngleValid = false;
    mbStatusValid = false;
    
    CANMotorControllerAction action =
        CANMotorControllerAction::CreateSDOFieldAction(
            SDOField( SDOField::eT_Write, "Controlword", 0x6040, 0 ) );
    action.mSDOField.SetU16( 0x0080 );
    
    AddExtraAction( action );
    
    action = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Controlword", 0x6040, 0 ) );
    action.mSDOField.SetU16( 0x0006 );    // Shutdown
    
    AddExtraAction( action );
    
    action = CANMotorControllerAction::CreateSDOFieldAction(
        SDOField( SDOField::eT_Write, "Controlword", 0x6040, 0 ) );
    action.mSDOField.SetU16( 0x000F );    // Switch On
    
    AddExtraAction( action );
    
    printf( "Added fault reset action\n" );
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
        //assert( mNumExtraActions < EXTRA_ACTION_LIST_LENGTH );
        if ( mNumExtraActions < EXTRA_ACTION_LIST_LENGTH )
        {
            mExtraActionList[ mNumExtraActions ] = action;
            mNumExtraActions++;
        }
        else
        {
            printf( "Warning: Action lost for node %i\n", GetNodeId() );
        }
    }
}

//------------------------------------------------------------------------------
void CANMotorController::RemoveFirstExtraAction()
{
    if ( mbInitialised 
        && mNumExtraActions > 0 )
    {
        // Very inefficient update of the queue by copying all fields forward
        // one space.
        memcpy( &mExtraActionList[ 0 ], &mExtraActionList[ 1 ], 
            sizeof( CANMotorControllerAction )*(mNumExtraActions-1) );
        mNumExtraActions--;
    }
}

//------------------------------------------------------------------------------
void CANMotorController::ProcessConfigurationAction()
{ 
    if ( mNumConfigurationActionsDone < mNumConfigurationActions )
    {
        CANMotorControllerAction& curAction = mConfigurationActionList[ mNumConfigurationActionsDone ];
        bool bActionComplete = ProcessAction( curAction );
        if ( bActionComplete )
        {
            mNumConfigurationActionsDone++;
        }
    }
}
    
//------------------------------------------------------------------------------
void CANMotorController::ProcessExtraAction()
{
    if ( mNumExtraActions > 0 )
    {
        CANMotorControllerAction& curAction = mExtraActionList[ 0 ];
        bool bActionComplete = ProcessAction( curAction, true );
        if ( bActionComplete )
        {
            //printf( "Sent extra action for node %i\n", GetNodeId() );
            RemoveFirstExtraAction();
        }
    }
}

//------------------------------------------------------------------------------
bool CANMotorController::ProcessAction( CANMotorControllerAction& action, bool bDebug )
{
    bool bActionComplete = false;
    
    switch ( action.mType )
    {
        case CANMotorControllerAction::eT_EnsureNMTState:
        {
            if ( mLastKnownNMTState == action.mEnsureNMTState.mDesiredState )
            {
                // We've got to the desired state
                bActionComplete = true;
            }
            else
            { 
                if ( EnsureNMTState::eT_Active == action.mEnsureNMTState.mType )
                {
                    assert( false && "Active state change not implemented yet" );
                }
            }
            break;
        }
        case CANMotorControllerAction::eT_SDOField:
        {
            switch ( mCommunicationState )
            {
                case eCS_Inactive:
                {
                    assert( NULL == mpActiveSDOField );
                    
                    if ( COI_ProcessSDOField( mpOwner, mNodeId, action.mSDOField ) )
                    {
                        mpActiveSDOField = &action.mSDOField;
                        mCommunicationState = 
                            ( SDOField::eT_Read == action.mSDOField.mType 
                            ? eCS_WaitingForSDORead : eCS_WaitingForSDOWrite );
                            
                        if ( eCS_WaitingForSDOWrite == mCommunicationState )
                        {
                            mSDOWriteFrameIdx = mpOwner->GetFrameIdx();
                        }
                    }
                    else if ( bDebug )
                    {
                        printf( "Couldn't send action\n" );
                    }
                    
                    break;
                }
                case eCS_WaitingForSDORead:
                case eCS_WaitingForSDOWrite:
                {
                    // Nothing to do
                    break;
                }
                default:
                {
                    assert( false && "Unhandled communication state" );
                }
            }
            break;
        }
        default:
        {
            assert( false && "Unhandled CAN motor controller action type" );
        }
    }
    
    return bActionComplete;
}

//------------------------------------------------------------------------------
void CANMotorController::HandleSDOReadComplete( SDOField& field )
{
    CANMotorController* pThis = (CANMotorController*)field.mpUserData;
    
    if ( &(pThis->mReadAction) == pThis->mpActiveSDOField )
    {
        pThis->mAngle = *((S32*)field.mData);
        pThis->mbAngleValid = true;
    }
    else if ( &(pThis->mReadStatusAction) == pThis->mpActiveSDOField )
    {
        pThis->mEposStatusword = *((U16*)field.mData);
        pThis->mbStatusValid = true;
    }
    
    /*if ( pThis->GetNodeId() == 15 )
    {
        printf( "Angle from node %i read as %i\n", pThis->GetNodeId(), pThis->mAngle );
    }*/
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