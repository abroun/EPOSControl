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
const SDOField CANMotorController::POSITION_CONTROL_SETUP_COMMANDS[] = {
    SDOField::CreateWrite_U8( "Mode of Operation", 0x6060, 0, 1 ),      // Use profile position mode
    SDOField::CreateWrite_U32( "Profile Velocity", 0x6081, 0, 500 ),    // Default to a slow speed
    SDOField::CreateWrite_U16( "Motion profile type", 0x6086, 0, 1 ),   // Use a sinusoidal profile
    SDOField::CreateWrite_U16( "Controlword", 0x6040, 0, 0x0006 ),      // Shutdown
    SDOField::CreateWrite_U16( "Controlword", 0x6040, 0, 0x000F ),      // Switch On
    
    SDOField( SDOField::eT_Invalid, "LIST END MARKER", 0, 0 )
};

const SDOField CANMotorController::FAULT_RESET_COMMANDS[] = {
    SDOField::CreateWrite_U16( "Controlword", 0x6040, 0, 0x0080 ),      // Reset
    SDOField::CreateWrite_U16( "Controlword", 0x6040, 0, 0x0006 ),      // Shutdown
    SDOField::CreateWrite_U16( "Controlword", 0x6040, 0, 0x000F ),      // Switch On
    
    SDOField( SDOField::eT_Invalid, "LIST END MARKER", 0, 0 )
};

//------------------------------------------------------------------------------
// CANMotorController
//------------------------------------------------------------------------------
CANMotorController::CANMotorController()
    : mbInitialised( false )
{
    // Fill in command buffers
    
    // Set desired angle
    mSetDesiredAngleCommands[ 0 ] = SDOField::CreateWrite_S32( "Target Position", 0x607A, 0, 0 );
    mSetDesiredAngleCommands[ 1 ] = SDOField::CreateWrite_U16( "Controlword", 0x6040, 0, 0x003F );  // Start positioning
    mSetDesiredAngleCommands[ 2 ] = SDOField( SDOField::eT_Invalid, "LIST END MARKER", 0, 0 );

    // Set profile velocity
    mSetProfileVelocityCommands[ 0 ] = SDOField::CreateWrite_U32( "Profile Velocity", 0x6081, 0, 500 ),
    mSetProfileVelocityCommands[ 1 ] = SDOField( SDOField::eT_Invalid, "LIST END MARKER", 0, 0 );

    // Set maximum following error
    mSetMaxFollowingErrorCommands[ 0 ] = SDOField::CreateWrite_U32( "Maximum Following Error", 0x6065, 0, 2000 ),
    mSetMaxFollowingErrorCommands[ 1 ] = SDOField( SDOField::eT_Invalid, "LIST END MARKER", 0, 0 );
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
    
        mLastKnownNMTState = eNMTS_Unknown;
        mSdoReadState = eSCS_Inactive;
        mSdoWriteState = eSCS_Inactive;
        mpActiveSdoReadField = NULL;
        mState = eS_Inactive;
        mConfiguration = eC_None;
        mpConfigurationSetupCommands = NULL;
        mRunningTask = eRT_None;
        mpRunningTaskCommands = NULL;
        mbPresent = false;
        mbAngleValid = false;
        mbStatusValid = false;
        mLastStatusPollFrameIdx = 0;
        
        mbFaultResetRequested = false;
        mbNewDesiredAngleRequested = false;
        mbNewProfileVelocityRequested = false;
        mbNewMaximumFollowingErrorRequested = false;
        
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
    mpActiveSdoReadField = NULL;
    mSdoReadState = eSCS_Inactive;
    mSdoWriteState = eSCS_Inactive;
    mLastKnownNMTState = eNMTS_Unknown;
    mbInitialised = false;
}

//------------------------------------------------------------------------------
void CANMotorController::Update( S32 frameIdx )
{
    static U16 oldStatusWord = 0;
    if ( mbStatusValid && 3 == mNodeId )
        //&& ( ( mEposStatusword & 0x0008 ) ) ) //|| 3 == mNodeId ) )
    {
        if ( oldStatusWord != mEposStatusword )
        {
            printf( "Node %i status is 0x%X\n", mNodeId, mEposStatusword );
            oldStatusWord = mEposStatusword;
        }
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
                if ( eC_None != mConfiguration )
                {
                    mState = eS_SettingUp;
                }
                
                break;
            }
            case eS_SettingUp:
            {
                // Process the current SDO write
                const SDOField* pCurCommand = &mpConfigurationSetupCommands[ mCurConfigurationSetupCommandIdx ];
                if ( SDOField::eT_Invalid != pCurCommand->mType )
                {
                    if ( ProcessSDOWrite( *pCurCommand ) )
                    {
                        mCurConfigurationSetupCommandIdx++;
                        pCurCommand = &mpConfigurationSetupCommands[ mCurConfigurationSetupCommandIdx ];
                    }
                }
                
                if ( SDOField::eT_Invalid == pCurCommand->mType
                    && eSCS_Inactive == mSdoWriteState )
                {
                    // All setup commands have been sent and received
                    
                    // Switch to the Running state
                    mbFaultResetRequested = false;
                    mbNewDesiredAngleRequested = false;
                    mbNewProfileVelocityRequested = false;
                    mbNewMaximumFollowingErrorRequested = false;
                    mRunningTask = eRT_None;
                    mState = eS_Running;
                }
                break;
            }
            case eS_Running:
            {
                if ( eRT_None == mRunningTask )
                {
                    // Check to see what we should be doing
                    // NOTE: The order here implies the priority of the tasks
                    if ( mbFaultResetRequested )
                    {
                        mpRunningTaskCommands = FAULT_RESET_COMMANDS;
                        mCurRunningTaskCommandIdx = 0;
                        mbFaultResetRequested = false;
                        mRunningTask = eRT_SendFaultReset;
                    }
                    else if ( mbNewProfileVelocityRequested )
                    {
                        mSetProfileVelocityCommands[ 0 ].SetU32( mNewProfileVelocity );
                        mpRunningTaskCommands = mSetProfileVelocityCommands;
                        mCurRunningTaskCommandIdx = 0;
                        mbNewProfileVelocityRequested = false;
                        mRunningTask = eRT_SetProfileVelocity;
                    }
                    else if ( mbNewMaximumFollowingErrorRequested )
                    {
                        mSetMaxFollowingErrorCommands[ 0 ].SetU32( mNewMaximumFollowingError );
                        mpRunningTaskCommands = mSetMaxFollowingErrorCommands;
                        mCurRunningTaskCommandIdx = 0;
                        mbNewMaximumFollowingErrorRequested = false;
                        mRunningTask = eRT_SetMaximumFollowingError;
                    }
                    else if ( mbNewDesiredAngleRequested )
                    {
                        mSetDesiredAngleCommands[ 0 ].SetS32( mNewDesiredAngle );
                        mpRunningTaskCommands = mSetDesiredAngleCommands;
                        mCurRunningTaskCommandIdx = 0;
                        mbNewDesiredAngleRequested = false;
                        mRunningTask = eRT_SetDesiredAngle;
                    }
                }
                
                switch ( mRunningTask )
                {
                    case eRT_None:
                    {
                        // Nothing to do
                        break;
                    }
                    case eRT_SetDesiredAngle:
                    case eRT_SendFaultReset:
                    case eRT_SetProfileVelocity:
                    case eRT_SetMaximumFollowingError:
                    {
                        // Process the current SDO write
                        const SDOField* pCurCommand = &mpRunningTaskCommands[ mCurRunningTaskCommandIdx ];                        
                        if ( SDOField::eT_Invalid != pCurCommand->mType )
                        {
                            if ( ProcessSDOWrite( *pCurCommand ) )
                            {
                                mCurRunningTaskCommandIdx++;
                                //printf( "Processed write %i for task of type %i\n", mCurRunningTaskCommandIdx, mRunningTask );
                                
                                pCurCommand = &mpRunningTaskCommands[ mCurRunningTaskCommandIdx ];
                            }
                        }
                        
                        if ( SDOField::eT_Invalid == pCurCommand->mType
                            && eSCS_Inactive == mSdoWriteState )
                        {
//                             if ( eRT_SetDesiredAngle == mRunningTask )
//                             {
//                                 printf( "The angle was sent\n" );
//                             }
                            
                            // All commands have been sent and received
                            mRunningTask = eRT_None;
                        }
                        break;
                    }
                    default:
                    {
                        assert( false && "Unhandled Running Task encountered" );
                    }
                    
                }
                break;
            }
            case eS_Homing:
            {
                break;
            }
            default:
            {
                assert( false && "Unhandled state encountered" );
            }
        }
        
        if ( eS_Running == mState || eS_Homing == mState )
        {
            // Handle communications that poll for information
            switch ( mSdoReadState )
            {
                case eSCS_Inactive:
                {
                    assert( NULL == mpActiveSdoReadField );
                    
                    // We can poll for either position or status
                    if ( !mbStatusValid
                        || ( frameIdx - mLastStatusPollFrameIdx > 100 ) )    // Poll for status periodically TODO: Make this nicer
                    {
                        mpActiveSdoReadField = &mReadStatusAction;
                        mSdoReadState =  eSCS_Active;
                        if ( COI_ProcessSDOField( mpOwner, mNodeId, mReadStatusAction ) )
                        {
                            mLastStatusPollFrameIdx = frameIdx;
                        }
                        else
                        {
                            // Reset
                            mpActiveSdoReadField = NULL;
                            mSdoReadState = eSCS_Inactive;
                        }
                    }
                    else
                    {
                        // Poll for angle
                        mpActiveSdoReadField = &mReadAction;
                        mSdoReadState =  eSCS_Active;
                        if ( COI_ProcessSDOField( mpOwner, mNodeId, mReadAction ) )
                        {
                            // Poll complete
                        }
                        else
                        {
                            // Reset
                            mpActiveSdoReadField = NULL;
                            mSdoReadState = eSCS_Inactive;
                        }
                    }
                    
                    break;
                }
                case eSCS_Active:
                {
                    // Nothing to do
                    break;
                }
                default:
                {
                    assert( false && "Unhandled communication state" );
                }
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
    assert( eSCS_Active == mSdoWriteState );
       
    mSdoWriteState = eSCS_Inactive;
}

//------------------------------------------------------------------------------
void CANMotorController::OnSDOFieldReadComplete( U8* pData, U32 numBytes )
{
    assert( eSCS_Active == mSdoReadState );
    assert( mpActiveSdoReadField != NULL );
    
    memcpy( mpActiveSdoReadField->mData, pData, numBytes );
    mpActiveSdoReadField->mReadCallback( *mpActiveSdoReadField );
    
    mpActiveSdoReadField = NULL;
    mSdoReadState = eSCS_Inactive;
}

//------------------------------------------------------------------------------
void CANMotorController::SetDesiredAngle( S32 desiredAngle, S32 frameIdx )
{
    if ( (eRT_SetDesiredAngle == mRunningTask || mbNewDesiredAngleRequested)
        && desiredAngle == mNewDesiredAngle )
    {
        // We're already trying to set the desired angle so ignore the request
        //printf( "Ignoring request\n" );
        return;
    }
    
    mNewDesiredAngle = desiredAngle;
    mbNewDesiredAngleRequested = true;
    //printf( "Got new angle of %i encoder ticks\n", desiredAngle );
}

//------------------------------------------------------------------------------
void CANMotorController::SetProfileVelocity( U32 profileVelocity )
{
    if ( (eRT_SetProfileVelocity == mRunningTask || mbNewProfileVelocityRequested)
        && profileVelocity == mNewProfileVelocity )
    {
        // We're already trying to set the desired angle so ignore the request
        //printf( "Ignoring speed request\n" );
        return;
    }
    
    mNewProfileVelocity = profileVelocity;
    mbNewProfileVelocityRequested = true;
}

//------------------------------------------------------------------------------
void CANMotorController::SetMaximumFollowingError( U32 maximumFollowingError )
{
    if ( (eRT_SetMaximumFollowingError == mRunningTask || mbNewMaximumFollowingErrorRequested)
        && maximumFollowingError == mNewMaximumFollowingError )
    {
        // We're already trying to set the max following error so ignore the request
        //printf( "Ignoring max following error request\n" );
        return;
    }

    mNewMaximumFollowingError = maximumFollowingError;
    mbNewMaximumFollowingErrorRequested = true;
}

//------------------------------------------------------------------------------
void CANMotorController::SendFaultReset()
{
    mbFaultResetRequested = true;
}

//------------------------------------------------------------------------------
void CANMotorController::SetConfiguration( eConfiguration configuration )
{
    if ( eC_None != configuration 
        && (eS_Inactive == mState || eS_Running == mState)  // Only want to debug a couple of states for now
        && configuration != mConfiguration )
    {
        const SDOField* pConfigSetupCommands = NULL;
        
        if ( eC_PositionControl == configuration )
        {
            pConfigSetupCommands = POSITION_CONTROL_SETUP_COMMANDS;
        }
        
        if ( NULL == pConfigSetupCommands )
        {
            fprintf( stderr, "Error: Unable to find commands for configuration %i\n", configuration );
            return;
        }
        
        mpConfigurationSetupCommands = pConfigSetupCommands;
        mCurConfigurationSetupCommandIdx = 0;
        mConfiguration = configuration;
        
        if ( eS_Inactive != mState )
        {
            // We can start setting up straight away
            mState = eS_SettingUp;
        }
    }
}

//------------------------------------------------------------------------------
bool CANMotorController::ProcessSDOWrite( const SDOField& sdoField, bool bDebug )
{
    assert( SDOField::eT_Write == sdoField.mType );
    
    bool bWriteComplete = false;
    
    switch ( mSdoWriteState )
    {
        case eSCS_Inactive:
        {   
            bWriteComplete = true;
            mSdoWriteState = eSCS_Active;
            if ( COI_ProcessSDOField( mpOwner, mNodeId, sdoField ) )
            {    
                mSDOWriteFrameIdx = mpOwner->GetFrameIdx();
            }
            else 
            {
                // Reset
                bWriteComplete = false;
                mSdoWriteState = eSCS_Inactive;
                
                if ( bDebug )
                {
                    printf( "Couldn't send action\n" );
                }
            }
            
            break;
        }
        case eSCS_Active:
        {
            // Already performing an SDO write so wait
            break;
        }
        default:
        {
            assert( false && "Unhandled communication state" );
        }
    }
    
    return bWriteComplete;
}

//------------------------------------------------------------------------------
void CANMotorController::HandleSDOReadComplete( SDOField& field )
{
    CANMotorController* pThis = (CANMotorController*)field.mpUserData;
    
    if ( &(pThis->mReadAction) == pThis->mpActiveSdoReadField )
    {
        pThis->mAngle = *((S32*)field.mData);
        pThis->mbAngleValid = true;
    }
    else if ( &(pThis->mReadStatusAction) == pThis->mpActiveSdoReadField )
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
