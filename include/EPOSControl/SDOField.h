//------------------------------------------------------------------------------
// File: SDOField.h
// Desc: An object that configures either a read or a write of an SDO field
//       from a CAN Open node
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef SDO_FIELD_H
#define SDO_FIELD_H

//------------------------------------------------------------------------------
#include <stdlib.h>
#include "Common.h"

//------------------------------------------------------------------------------
struct SDOField;
typedef void (*SDOReadCallback)( SDOField& field );

//------------------------------------------------------------------------------
struct SDOField
{
    //--------------------------------------------------------------------------
    enum eType
    {
        eT_Invalid = -1,
        eT_Read,
        eT_Write,
        eT_NumTypes
    };
    
    //--------------------------------------------------------------------------
    SDOField();
    SDOField( eType type, const char* pDescription, U16 mIndex, U8 mSubIndex,
              SDOReadCallback readCallback = NULL, void* pUserData = NULL );
    
    //--------------------------------------------------------------------------
    static SDOField CreateWrite_U8( const char* pDescription, U16 mIndex, U8 mSubIndex, U8 data ) 
    {
        SDOField field( eT_Write, pDescription, mIndex, mSubIndex );
        field.SetU8( data );
        
        return field;
    }
    
    static SDOField CreateWrite_U16( const char* pDescription, U16 mIndex, U8 mSubIndex, U16 data ) 
    {
        SDOField field( eT_Write, pDescription, mIndex, mSubIndex );
        field.SetU16( data );
        
        return field;
    }
    
    static SDOField CreateWrite_U32( const char* pDescription, U16 mIndex, U8 mSubIndex, U32 data ) 
    {
        SDOField field( eT_Write, pDescription, mIndex, mSubIndex );
        field.SetU32( data );
        
        return field;
    }
    
    static SDOField CreateWrite_S32( const char* pDescription, U16 mIndex, U8 mSubIndex, S32 data ) 
    {
        SDOField field( eT_Write, pDescription, mIndex, mSubIndex );
        field.SetS32( data );
        
        return field;
    }
              
    //--------------------------------------------------------------------------
    void SetU8( U8 data ) 
    { 
        mData[ 0 ] = data;
        mNumBytes = 1;
    }
    
    void SetU16( U16 data ) 
    { 
        ((U16*)mData)[ 0 ] = data;
        mNumBytes = 2;
    }
    
    void SetU32( U32 data ) 
    { 
        ((U32*)mData)[ 0 ] = data;
        mNumBytes = 4;
    }
    
    void SetS32( S32 data ) 
    { 
        SetU32( (U32)data );
    }
    
    //--------------------------------------------------------------------------
    static const S32 MAX_DESC_LENGTH = 31;
    
    eType mType;
    
    char mDescription[ MAX_DESC_LENGTH + 1 ];  // A printable string for debug purposes
    U16 mIndex;
    U8 mSubIndex;
    U8 mData[ 8 ];
    U32 mNumBytes;
    
    SDOReadCallback mReadCallback;
    void* mpUserData;   // Mainly present so that the field can be identified in a read callback    
};

#endif // SDO_FIELD_H