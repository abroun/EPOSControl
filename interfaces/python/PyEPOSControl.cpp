//------------------------------------------------------------------------------
// Exposes the EPOSControl library as a Python extension
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include "Python.h"
#include "structmember.h"

#include <math.h>
#include "EPOSControl/EPOSControl.h"

//------------------------------------------------------------------------------
static CANChannel* gpChannel = NULL;

//------------------------------------------------------------------------------
typedef struct 
{
    PyObject_HEAD    
} EPOSControlObject;

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
static void EPOSControlObject_dealloc( EPOSControlObject* self )
{
    // Shut down the EPOSControl library
    EPOS_CloseCANChannel( gpChannel );
    EPOS_DeinitLibrary();
    
    self->ob_type->tp_free((PyObject*)self);
}

//------------------------------------------------------------------------------
static PyObject* EPOSControlObject_new( PyTypeObject *type, 
                                         PyObject *args, PyObject *kwds )
{
    EPOSControlObject *self = (EPOSControlObject *)type->tp_alloc(type, 0);
    return (PyObject *)self;
}

//------------------------------------------------------------------------------
static int EPOSControlObject_init( EPOSControlObject *self, 
                                   PyObject *args, PyObject *kwds )
{
    // Hacky check
    if ( NULL != gpChannel )
    {
        fprintf( stderr, "Error: Channel already in use\n" );
        return -1;
    }
    
    // Start up the EPOSControl library
    if ( !EPOS_InitLibaray() )
    {
        fprintf( stderr, "Error: Unable to open EPOSControl library\n" );
        return -1;
    }
    
    gpChannel = EPOS_OpenCANChannel( "32", eBR_1M );
    if ( NULL == gpChannel )
    {
        fprintf( stderr, "Error: Unable top open CAN bus channel\n" );
        return -1;
    }
    
    gpChannel->ConfigureAllMotorControllersForPositionControl();
    
    return 0;
}

//------------------------------------------------------------------------------
static PyMethodDef EPOSControlObjectMethods[] = 
{
    { "getJointAngles", getJointAngles, METH_VARARGS, "Get the motor controller joint angles" },
    {NULL}  /* Sentinel */
};

//------------------------------------------------------------------------------
static PyMemberDef EPOSControlObjectMembers[] = 
{
    {NULL}  /* Sentinel */
};

//------------------------------------------------------------------------------
static PyTypeObject EPOSControlObjectType = 
{
    PyObject_HEAD_INIT(NULL)
    0,                         /*ob_size*/
    "PyEPOSControl.EPOSControl",             /*tp_name*/
    sizeof(EPOSControlObject), /*tp_basicsize*/
    0,                         /*tp_itemsize*/
    (destructor)EPOSControlObject_dealloc, /*tp_dealloc*/
    0,                         /*tp_print*/
    0,                         /*tp_getattr*/
    0,                         /*tp_setattr*/
    0,                         /*tp_compare*/
    0,                         /*tp_repr*/
    0,                         /*tp_as_number*/
    0,                         /*tp_as_sequence*/
    0,                         /*tp_as_mapping*/
    0,                         /*tp_hash */
    0,                         /*tp_call*/
    0,                         /*tp_str*/
    0,                         /*tp_getattro*/
    0,                         /*tp_setattro*/
    0,                         /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /*tp_flags*/
    "Object to manage the lifetime of communication with the EPOS motor controllers",          /* tp_doc */
    0,                     /* tp_traverse */
    0,                     /* tp_clear */
    0,                     /* tp_richcompare */
    0,                     /* tp_weaklistoffset */
    0,                     /* tp_iter */
    0,                     /* tp_iternext */
    EPOSControlObjectMethods,             /* tp_methods */
    EPOSControlObjectMembers,             /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)EPOSControlObject_init,      /* tp_init */
    0,                         /* tp_alloc */
    EPOSControlObject_new,                 /* tp_new */
};

//------------------------------------------------------------------------------
static PyMethodDef PyEPOSControlMethods[] = 
{
    { NULL, NULL, 0, NULL }     // Sentinel - marks the end of this structure
};

//------------------------------------------------------------------------------
PyMODINIT_FUNC initPyEPOSControl()  
{
    if ( PyType_Ready( &EPOSControlObjectType ) < 0 )
    {
        return;
    }
    
    PyObject * pModule = Py_InitModule( "PyEPOSControl", PyEPOSControlMethods );
    
    Py_INCREF( &EPOSControlObjectType );
    PyModule_AddObject( pModule, "EPOSControl", (PyObject *)&EPOSControlObjectType );
}


