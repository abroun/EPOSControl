//------------------------------------------------------------------------------
// Exposes the EPOSControl library as a Python extension
//------------------------------------------------------------------------------

#include "Python.h"
#include <math.h>
#include "EPOSControl/EPOSControl.h"

//------------------------------------------------------------------------------
// Returns the current joint angles of the motor controllers on the CAN
// channels. The joint angles are returned in a dictionary where the keys
// are the node indices of the CAN nodes and these dictionaries are in turn
// returned in a dictionary where the keys are the index of the CAN channel
static PyObject* getJointAngles( PyObject* pSelf, PyObject* args )
{
/*    
    // Create a copy to output
    PyObject* pOutputArray = PyArray_NewCopy( pImgArray, NPY_CORDER );
    
    // Process the data
    unsigned char* pData = (unsigned char*)PyArray_DATA( pOutputArray );
    int yStride = PyArray_STRIDE( pOutputArray, 0 );
    int xStride = PyArray_STRIDE( pOutputArray, 1 );
    int height = PyArray_DIM( pOutputArray, 0 );
    int width = PyArray_DIM( pOutputArray, 1 );
     
    int numBlobs = SegmentByteArray( pData, width, height, xStride, yStride, eCT_Connect8 );
    */
    // Return the processed data
    return Py_BuildValue( "i", 4 );
}

//------------------------------------------------------------------------------
// Methods table
static PyMethodDef PyEPOSControlMethods[] = {
    { "getJointAngles", getJointAngles, METH_VARARGS, "Get the motor controller joint angles" },
    { NULL, NULL, 0, NULL }     // Sentinel - marks the end of this structure
};

//------------------------------------------------------------------------------
PyMODINIT_FUNC initPyEPOSControl()  
{
    Py_InitModule( "PyEPOSControl", PyEPOSControlMethods );
    
    // Setup stuff here
    printf( "Loaded PyEPOSControl\n" );
}


