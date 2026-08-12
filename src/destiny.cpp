// Copyright © 2000 CCP ehf.

#include "StdAfx.h"


#include "Ball.h"
#include "Ballpark.h"
#include "Partition.h"
#include "DstConstants.h"
#include "Settings.h"

// Define the interfaces we use from Trinity
BLUE_DEFINE_INTERFACE( ITriVectorFunction );
BLUE_DEFINE_INTERFACE( ITriQuaternionFunction );

// interfaces
BLUE_DEFINE_INTERFACE(IBall);
BLUE_DEFINE_INTERFACE(IBallpark);

// classes
BLUE_DEFINE(Ballpark);
BLUE_DEFINE(Ball);
BLUE_DEFINE(ClientBall);
BLUE_DEFINE(MiniBall);
BLUE_DEFINE(MiniBox);
BLUE_DEFINE(MiniCapsule);

#include "Quaternion.h"

#include <limits>
#ifdef max //remove silly max macro, for limits
#undef max
#endif

char _bcfix[8];
BLUE_STANDARD_MODULE_INIT( _destiny );

static CcpLogChannel_t s_chPark = CCP_LOG_DEFINE_CHANNEL( "BallPark" );

struct HeapEntry
{
	HeapEntry(PyObject *o) : mObject(o, true) {}
	bool operator < (const HeapEntry &rhs) const {
		double a1 = PyFloat_AS_DOUBLE(PyList_GET_ITEM(mObject.o, 7));
		double a2 = PyFloat_AS_DOUBLE(PyList_GET_ITEM(rhs.mObject.o, 7));
		return a1 < a2;
	}
	operator PyObject * () const { return mObject; }
	operator PyListObject * () const { return (PyListObject*)mObject.o; }
	BluePy mObject;
};

//////////////////////////////////////////////////////////////////////
//
// DLL Startup / Shutdown
//
//////////////////////////////////////////////////////////////////////

// reduce CRT link
extern "C" void _setargv(){}
extern "C" void _setenvp(){}

PyObject* gThisModule = NULL;

#ifdef _WIN32
//--------------------------------------------------------------------
BOOL APIENTRY DllMain(HINSTANCE instance, DWORD  reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(instance);
    }
    return TRUE;
}
#endif
