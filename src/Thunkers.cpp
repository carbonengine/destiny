// Copyright © 2014 CCP ehf.

#include "StdAfx.h"
#include "Ballpark.h"
#include "Ball.h"
#include "Box.h"
#include "Settings.h"
#include "Triangle.h"
#define ABS(X) ((X)<0.0?-(X):(X))
constexpr double PI = 3.141592654;


#include <IBluePersist.h>
#include <ITaskletTimer.h>

#include <array>

static CcpLogChannel_t s_chPThunk = CCP_LOG_DEFINE_CHANNEL( "ParkThunkers" );
static CcpLogChannel_t s_chPark = CCP_LOG_DEFINE_CHANNEL( "BallPark" );

#define TASKLET(A) AutoTasklet _at(TheTimer, A)
extern PyObject* Timer_AdditionsAndDeletions;
extern PyObject* Timer_GetBallIdsInRange;
extern PyObject* Timer_WriteBallsToStream;
extern ITaskletTimer *TheTimer;
extern const std::array<int,4> followModes;

static size_t byteCount = 0;

void RaiseBallNotInParkError(const char* callerName, ID ballID)
{
	PyErr_Format(PyExc_RuntimeError, "%s: Ball %lld not in park", callerName, ballID);
}

//---------------------------------------------------------------------------------------
// PYTHON THUNKERS

PyObject* Ballpark::Py__init__(
	PyObject* args
	)
{
	if( !PyArg_ParseTuple( args, "|p", &isMaster ) )
		return nullptr;

	Py_RETURN_NONE;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyAddDynamicallyOrientedBall
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyAddDynamicallyOrientedBall(
	PyObject* args )
{
	ID srcId;
	double mass;
	float radius;
	float maxVel;
	float maxAngularSpeed;
	int isFree;
	int isGlobal;
	int isMassive;
	int isInteractive;
	int isSpaceJunk;
	double x;
	double y;
	double z;
	double vx;
	double vy;
	double vz;
	float agility;
	float angularAgility;
	float speedFraction;

	if( !PyArg_ParseTuple( args, "Ldffiiiiiddddddffff", &srcId, &mass, &radius, &maxVel, &isFree, &isGlobal, &isMassive, &isInteractive, &isSpaceJunk, &x, &y, &z, &vx, &vy, &vz, &agility, &speedFraction, &maxAngularSpeed, &angularAgility ) )
		return NULL;

	if( inEvolve )
	{
		PyErr_SetString( PyExc_RuntimeError, "Can not add ball during Evolve()." );
		return nullptr;
	}

	Ball* b = AddDynamicallyOrientedBall(
		srcId,
		mass,
		radius,
		maxVel,
		maxAngularSpeed,
		isFree != 0,
		isGlobal != 0,
		isMassive != 0,
		isInteractive != 0,
		isSpaceJunk != 0,
		x, y, z,
		vx, vy, vz,
		0.0f, 0.0f, 0.0f, 1.0f,
		0.0f, 0.0f, 0.0f,
		agility,
		angularAgility,
		speedFraction );

	// The python add ball currently knows nothing about orientation or
	// rotational velocity.  Snap the rotation to the velocity vector
	b->SnapOrientation();

	return BlueWrapObjectForPython( (IBall*)b );
}


//---------------------------------------------------------------------------------------
// Ballpark::PyAddOldStyleOrientedBall
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyAddOldStyleOrientedBall(
	PyObject* args )
{
	ID srcId;
	double mass;
	float radius;
	float maxVel;
	int isFree;
	int isGlobal;
	int isMassive;
	int isInteractive;
	int isSpaceJunk;
	double x;
	double y;
	double z;
	double vx;
	double vy;
	double vz;
	float agility;
	float speedFraction;

	if( !PyArg_ParseTuple( args, "Ldffiiiiiddddddff", &srcId, &mass, &radius, &maxVel, &isFree, &isGlobal, &isMassive, &isInteractive, &isSpaceJunk, &x, &y, &z, &vx, &vy, &vz, &agility, &speedFraction ) )
		return nullptr;

	if( inEvolve )
	{
		PyErr_SetString( PyExc_RuntimeError, "Can not add ball during Evolve()." );
		return nullptr;
	}

	Ball* b = AddOldStyleOrientedBall( srcId, mass, radius, maxVel, isFree != 0, isGlobal != 0, isMassive != 0, isInteractive != 0, isSpaceJunk != 0, x, y, z, vx, vy, vz, agility, speedFraction );
	return BlueWrapObjectForPython( (IBall*)b );
}

//---------------------------------------------------------------------------------------
// Ballpark::PyAddBall
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyAddBall(
	PyObject* args
	)
{
	Ball* b = nullptr;
	if( g_useDynamicalOrientation )
	{
		return PyAddDynamicallyOrientedBall(args);
	}
	return PyAddOldStyleOrientedBall(args);
}

PyObject* Ballpark::PyAddCapsule(
	PyObject* args
	)
{
	ID srcId;
	ID parentId;

	double ax;
	double ay;
	double az;

	double bx;
	double by;
	double bz;

	float radius;

	if(!PyArg_ParseTuple(args, "LLddddddf",
		&srcId,
		&parentId,
		&ax, &ay, &az,
		&bx, &by, &bz,
		&radius
		))
		return NULL;
	
	Capsule *c = AddCapsule(
		srcId,
		parentId,
		ax, ay, az,
		bx, by, bz,
		radius);
	return BlueWrapObjectForPython((IRoot *)c);
}

PyObject* Ballpark::PyAddOrientedBox(
	PyObject* args
	)
{
	ID srcId;
	ID parentId;

	double corner_0;
	double corner_1;
	double corner_2;

	double x0;
	double x1;
	double x2;

	double y0;
	double y1;
	double y2;

	double z0;
	double z1;
	double z2;


	if(!PyArg_ParseTuple(args, "LLdddddddddddd",
		&srcId,
		&parentId,
		&corner_0, &corner_1, &corner_2,
		&x0, &x1, &x2,
		&y0, &y1, &y2,
		&z0, &z1, &z2))
		return NULL;

	OrientedBox *b = AddOrientedBox(
		srcId,
		parentId,
		corner_0, corner_1, corner_2,
		x0, x1, x2,
		y0, y1, y2,
		z0, z1, z2);

	return BlueWrapObjectForPython((IRoot *)b);
}

//---------------------------------------------------------------------------------------
// Ballpark::PyFollowBall
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyFollowBall(
	PyObject* args
	)
{
	ID srcId;
	ID dstId;
	float range = 1.0;
	int stamp = 0;

	if (!PyArg_ParseTuple(args, "LL|f",
		&srcId,
		&dstId,
		&range
		))
		return NULL;

	if(dstId < 0)
	{
		BeOS->SetError(BEDEF, Clsid(), "You can not Follow a negative ballID, src(%I64d), dst(%I64d)",srcId, dstId);
		return NULL;
	}

	FollowBall(srcId, dstId, range);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyMissileFollow
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyMissileFollow(
	PyObject* args
	)
{
	ID srcId;
	ID dstId;
	ID ownerId;

	if (!PyArg_ParseTuple(args, "LLL",
		&srcId,
		&dstId,
		&ownerId
		))
		return NULL;

	if(dstId < 0)
	{
		BeOS->SetError(BEDEF, Clsid(), "You can not MissileFollow a negative ballID, src(%I64d), dst(%I64d)",srcId, dstId);
		return NULL;
	}

	MissileFollow(srcId, dstId, ownerId);

	Py_INCREF(Py_None);
	return Py_None;
}


//---------------------------------------------------------------------------------------
// Ballpark::PyAddMushroom
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyAddMushroom(
	PyObject* args
	)
{
	ID ownerId;
	float range;
	double time;

	if (!PyArg_ParseTuple(args, "Lfd",
		&ownerId,
		&range,
		&time
		))
		return NULL;

	AddMushroom(ownerId, range, time);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyOrbit
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyOrbit(
	PyObject* args
	)
{
	ID srcId;
	ID dstId;
	float range = 1.0;

	if (!PyArg_ParseTuple(args, "LL|f",
		&srcId,
		&dstId,
		&range
		))
		return NULL;

	if(dstId < 0)
	{
		BeOS->SetError(BEDEF, Clsid(), "You can not Orbit a negative ballID, src(%I64d), dst(%I64d)",srcId, dstId);
		return NULL;
	}

	Orbit(srcId, dstId, range);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyWarpTo
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyWarpTo(
	PyObject* args
	)
{
	ID srcId;
	double x,y,z;
	double minRange = 20000.0;
	int warpFactor = 20;

	if (!PyArg_ParseTuple(args, "Lddd|di",
		&srcId,
		&x,
		&y,
		&z,
		&minRange,
		&warpFactor
		))
		return NULL;

	WarpTo(srcId, x,y,z, minRange, warpFactor);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyEntityWarpIn
//---------------------------------------------------------------------------------------
// const ID& srcId, double dst_x, double dst_y, double dst_z
PyObject* Ballpark::PyEntityWarpIn(
	PyObject* args
	)
{
	ID srcId;
	double x,y,z;
	int warpFactor;

	if (!PyArg_ParseTuple(args, "Ldddi",
		&srcId,
		&x,
		&y,
		&z,
		&warpFactor
		))
		return NULL;

	EntityWarpIn(srcId, x,y,z, warpFactor);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyAdjustTimes
//---------------------------------------------------------------------------------------

PyObject* Ballpark::PyAdjustTimes(
	PyObject* args
	)
{
    Be::Time timeDelta;

    if (!PyArg_ParseTuple(args, "L",
        &timeDelta
        ))
        return NULL;

    AdjustTimes(timeDelta);

    Py_RETURN_NONE;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyGotoPoint
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyGotoPoint(
	PyObject* args
	)
{
	ID srcId;
	double x;
	double y;
	double z;

	if (!PyArg_ParseTuple(args, "Lddd",
		&srcId,
		&x,
		&y,
		&z
		))
		return NULL;

	GotoPoint(srcId, x, y, z);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyGotoDirection
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyGotoDirection(
	PyObject* args
	)
{
	ID srcId;
	double x;
	double y;
	double z;

	if (!PyArg_ParseTuple(args, "Lddd",
		&srcId,
		&x,
		&y,
		&z
		))
		return NULL;

	GotoDirection(srcId, x, y, z);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBallMass
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallMass(
	PyObject* args
	)
{
	ID srcId;
	double mass;

	if (!PyArg_ParseTuple(args, "Ld",
		&srcId,
		&mass
		))
		return NULL;

	SetBallMass(srcId, mass);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBallRigid
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallRigid(
	PyObject* args
	)
{
	ID srcId;

	if (!PyArg_ParseTuple(args, "L",
		&srcId
		))
		return NULL;

	SetBallRigid(srcId);

	Py_INCREF(Py_None);
	return Py_None;
}


//---------------------------------------------------------------------------------------
// Ballpark::PySetBallTroll
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallTroll(
	PyObject* args
	)
{
	ID srcId;
	int delay;

	if (!PyArg_ParseTuple(args, "Li",
		&srcId,
		&delay
		))
		return NULL;

	SetBallTroll(srcId, delay);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBallRadius
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallRadius(
	PyObject* args
	)
{
	ID srcId;
	float radius;

	if (!PyArg_ParseTuple(args, "Lf",
		&srcId,
		&radius
		))
		return NULL;

	SetBallRadius(srcId, radius);

	Py_INCREF(Py_None);
	return Py_None;
}
//---------------------------------------------------------------------------------------
// Ballpark::PySetMaxSpeed
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetMaxSpeed(
	PyObject* args
	)
{
	ID srcId;
	float speed;

	if (!PyArg_ParseTuple(args, "Lf",
		&srcId,
		&speed
		))
		return NULL;

	SetMaxSpeed(srcId, speed);

	Py_INCREF(Py_None);
	return Py_None;
}


//---------------------------------------------------------------------------------------
// Ballpark::PyStop
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyStop(
	PyObject* args
	)
{
	ID srcId;

	if (!PyArg_ParseTuple(args, "L",
		&srcId
		))
		return NULL;

	Stop(srcId);

	Py_INCREF(Py_None);
	return Py_None;
}


//---------------------------------------------------------------------------------------
// Ballpark::PyClearAll
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyClearAll(
	PyObject* args
	)
{
	ClearAll();

	Py_INCREF(Py_None);
	return Py_None;
}


//---------------------------------------------------------------------------------------
// Ballpark::PyStart()
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyStart(
	PyObject* args
	)
{
	Start();

	Py_INCREF(Py_None);
	return Py_None;
}



//---------------------------------------------------------------------------------------
// Ballpark::PyPause
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyPause(
	PyObject* args
	)
{
	Pause();

	Py_INCREF(Py_None);
	return Py_None;
}



//---------------------------------------------------------------------------------------
// Ballpark::PySetBallPosition
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallPosition(
	PyObject* args
	)
{
	ID objectId;
	double x;
	double y;
	double z;

	if (!PyArg_ParseTuple(args, "Lddd",
		&objectId,
		&x,
		&y,
		&z
		))
		return NULL;

	SetBallPosition(objectId,  x, y, z);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBallAgility
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallAgility(
	PyObject* args
	)
{
	ID objectId;
	float agility;

	if (!PyArg_ParseTuple(args, "Lf",
		&objectId,
		&agility
		))
		return NULL;

	SetBallAgility(objectId,  agility);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetSpeedFraction
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetSpeedFraction(
	PyObject* args
	)
{
	ID objectId;
	float fraction;

	if (!PyArg_ParseTuple(args, "Lf",
		&objectId,
		&fraction
		))
		return NULL;

	SetSpeedFraction(objectId,  fraction);

	Py_INCREF(Py_None);
	return Py_None;
}


//---------------------------------------------------------------------------------------
// Ballpark::PySetNotificationRange
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetNotificationRange(
	PyObject* args
	)
{
	ID objectId;
	double range;

	if (!PyArg_ParseTuple(args, "Ld",
		&objectId,
		&range
		))
		return NULL;

	SetNotificationRange(objectId,  range);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetTargetTracking
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetTargetTracking(PyObject* args)
{
    ID sourceID;
    ID targetID;
    double range;

    if (!PyArg_ParseTuple(args, "LLd;srcID, targetID, fRange",
        &sourceID,
        &targetID,
        &range
        ))
        return NULL;

    SetTargetTracking(sourceID, targetID, range);

    Py_INCREF(Py_None);
    return Py_None;
}


//---------------------------------------------------------------------------------------
// Ballpark::PySetBallVelocity
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallVelocity(
	PyObject* args
	)
{
	ID objectId;
	double vx;
	double vy;
	double vz;

	if (!PyArg_ParseTuple(args, "Lddd",
		&objectId,
		&vx,
		&vy,
		&vz
		))
		return NULL;

	SetBallVelocity(objectId, vx, vy, vz);

	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* Ballpark::PySetMaxAngularSpeed( PyObject* args )
{
	ID srcId;
	float speed;

	if( !PyArg_ParseTuple( args, "Lf", &srcId, &speed ) )
		return nullptr;

	SetMaxAngularSpeed( srcId, speed );

	Py_RETURN_NONE;
}

PyObject* Ballpark::PySetBallAngularVelocity( PyObject* args )
{
	ID srcId;
	double wx, wy, wz;

	if( !PyArg_ParseTuple( args, "Lddd", &srcId, &wx, &wy, &wz ) )
		return nullptr;

	SetBallAngularVelocity( srcId, wx, wy, wz );

	Py_RETURN_NONE;
}

PyObject* Ballpark::PySetBallRotation( PyObject* args )
{
	ID srcId;
	double rx, ry, rz, rw;

	if( !PyArg_ParseTuple( args, "Ldddd", &srcId, &rx, &ry, &rz, &rw ) )
		return nullptr;

	SetBallRotation( srcId, rx, ry, rz, rw );

	Py_RETURN_NONE;
}
PyObject* Ballpark::PySetBallAngularAgility( PyObject* args )
{
	ID objectId;
	float agility;

	if( !PyArg_ParseTuple( args, "Lf", &objectId, &agility ) )
		return nullptr;

	SetBallAngularAgility( objectId, agility );

	Py_RETURN_NONE;
}

#pragma warning (push)
#pragma warning (disable: 4533)

//---------------------------------------------------------------------------------------
// Ballpark::PyLaunchMissile
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyLaunchMissile(
	PyObject* args
	)
{
	ID srcID;
	ID dstID;
	ID ownerID;
	char aimedLaunch = 0;
	char massive = 0;

	if (!PyArg_ParseTuple(args, "LLLbb",
		&srcID,
		&dstID,
		&ownerID,
		&aimedLaunch,
		&massive
		))
		return NULL;

	if(dstID < 0)
	{
		PyErr_SetString(PyExc_RuntimeError, "Can not launch missile on a negative ballID");
		return 0;
	}

    ID ownerIDCheck = ownerID;
    if(ownerID < 0)
        ownerIDCheck = -ownerID;

	Ball *launcherBall = mBalls[ownerIDCheck];
	Ball *targetBall = 0;

	if(!launcherBall)
	{
		//PyErr_SetString(PyExc_RuntimeError, "BallNotInPark");
        Py_RETURN_NONE;
	}

    if(dstID!=ownerID)
        targetBall = mBalls[dstID];
    
	double maxVelocity = 0.0;
	Vector3d ps = launcherBall->mNewPos;
	Vector3d v0;

    if(targetBall && aimedLaunch==1 && !targetBall->isCloaked)
	{
        // For aimed missiles, we instruct them to in the direction of the target
        Vector3d pt = targetBall->mNewPos;
        v0 = pt - ps;
		v0.Normalize();
	}
	else
	{
        // For normal missiles, we give them an initial velocity in direction of the launcher
        Vector3d vs =  launcherBall->mNewVel;
        maxVelocity = launcherBall->mMaxVel;
        Vector3d direction = vs;
		direction.Normalize();

        // If launcher is at a standstill, get the direction of the launcher
        if(direction.LengthSq()==0.0)
		{
			Vector3 v( 0.0f, 0.0f, -1.0f );
			launcherBall->GetRotatedVector(v);
            direction = Vector3d(v);
		}

		v0 = vs +std::max(150.0, maxVelocity)*direction;
	}

    if(massive)
        SetBallMassive(srcID, 1);

	SetBallPosition(srcID, ps.x, ps.y, ps.z);
	SetBallVelocity(srcID, v0.x, v0.y, v0.z);

    if(targetBall && targetBall->isCloaked)
        Py_RETURN_NONE;

    if(!targetBall)
        if(maxVelocity > 0.0)
			GotoDirection(srcID, v0.x, v0.y, v0.z);
        else
            Py_RETURN_NONE;
    else
		MissileFollow(srcID, dstID, ownerID);

    Py_RETURN_NONE;
}

#pragma warning (pop)
//---------------------------------------------------------------------------------------
// Ballpark::PySetBallHarmonic
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallHarmonic(
	PyObject* args
	)
{
	ID objectId;
	int64_t harmonic;
	int corporationID;
	int allianceID;
	int field = 0;

	if (!PyArg_ParseTuple(args, "LLiii",
		&objectId,
		&harmonic,
		&corporationID,
		&allianceID,
		&field
		))
		return NULL;

	SetBallHarmonic(objectId, harmonic, corporationID, allianceID, !!field);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBallFree
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallFree(
	PyObject* args
	)
{
	ID objectId;
	int isFree;

	if (!PyArg_ParseTuple(args, "Li",
		&objectId,
		&isFree
		))
		return NULL;

	SetBallFree(objectId, isFree!=0);

	Py_INCREF(Py_None);
	return Py_None;
}


//---------------------------------------------------------------------------------------
// Ballpark::PySetBoid
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBoid(
	PyObject* args
	)
{
	ID srcId;
	int turnOn;

	if (!PyArg_ParseTuple(args, "Li",
		&srcId,
		&turnOn
		))
		return NULL;

	Ball *b = mBalls[srcId];

	if(b)
	{
		if(turnOn)
			b->SetMode(DSTBALL_BOID);
		else
			Stop(b);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBallGlobal
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallGlobal(
	PyObject* args
	)
{
	ID srcId;
	int isGlobal;

	if (!PyArg_ParseTuple(args, "Li",
		&srcId,
		&isGlobal
		))
		return NULL;

	SetBallGlobal(srcId, isGlobal!=0);

	Py_INCREF(Py_None);
	return Py_None;
}
//---------------------------------------------------------------------------------------
// Ballpark::PySetBallMassive
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallMassive(
	PyObject* args
	)
{
	ID srcId;
	int isMassive;

	if (!PyArg_ParseTuple(args, "Li",
		&srcId,
		&isMassive
		))
		return NULL;

	SetBallMassive(srcId, isMassive!=0);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBallInteractive
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallInteractive(
	PyObject* args
	)
{
	ID srcId;
	int isInteractive;

	if (!PyArg_ParseTuple(args, "Li",
		&srcId,
		&isInteractive
		))
		return NULL;

	SetBallInteractive(srcId, isInteractive!=0);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyJoinFormation
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyFormationFollow(
	PyObject* args
	)
{
	ID srcId;
	ID dstId;
	int slot;

	if (!PyArg_ParseTuple(args, "LLi",
		&srcId,
		&dstId,
		&slot
		))
		return NULL;

	FormationFollow(srcId, dstId, char(slot));
	
	// A slot has been reserved. Set the mode.
	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyInitializeBubbles
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyInitializeBubbles(
	PyObject* args
	)
{
	ID srcId=0;

	if (!PyArg_ParseTuple(args, "|L",
		&srcId
		))
		return NULL;

	if(srcId == 0)
		InitializeBubbles();
	else
		UpdateBallBubble(srcId);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::GetFollowers
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyGetFollowers(
	PyObject* args
	)
{
	ID targetId;

	if (!PyArg_ParseTuple(args, "L",
		&targetId
		))
		return NULL;

	Ball *src = mBalls[targetId];

	if(!src)
		return NULL;

	int length = (int)src->mFollowers.size();
	PyObject* ret = PyList_New(0);

	auto foll = src->mFollowers.begin();
	auto end = src->mFollowers.end();
	for(; foll != end; ++foll)
	{
		auto id = *foll;
		Ball *dst = mBalls[id];
		if( !dst )
		{
			CCP_LOGERR_CH( s_chPark, "Ballpark::GetFollowers follower %I64d is not valid", id );
			continue;
		}

		if( dst->isMoribund )
		{
			CCP_LOGWARN_CH( s_chPark, "Ballpark::GetFollowers follower %I64d is moribund", id );
		}

		PyObject *val = PyLong_FromLongLong(dst->mId);
		PyList_Append(ret,val);
		Py_DECREF(val);
	}

	return ret;
}

//---------------------------------------------------------------------------------------
// Ballpark::GetRemoteFollowers
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyGetRemoteFollowers(
	PyObject* args
	)
{
	PyObject* ret = PyList_New(0);

	// Cycle over all free balls
	DictOfFreeBalls::iterator sit;
	Ball *src, *dst;
	for(sit = mFreeBalls.begin(); sit != mFreeBalls.end(); ++sit)
	{
		src = sit->second;
		uint8_t ballMode = src->mMode;
		if(std::any_of(followModes.begin(), followModes.end(), [ballMode](int mode){return ballMode == mode;}) )
		{
			if(src->isMoribund)
				CCP_LOGWARN_CH( s_chPark,"Ballpark::GetRemoteFollowers follower %I64d is moribund",src->mId);

			dst = mBalls[src->mFollowId];

			if(dst)
			{
				if(src->mNewBubble == dst->mNewBubble)
					continue;
			}
			else
			{
				// This is an inconsistent follower state, but include it in the stop list nonetheless.
				CCP_LOGERR_CH( s_chPark,"Ballpark::GetRemoteFollowers follower %I64d in mode %d following %I64d but without a pointer", src->mId, src->mMode, src->mFollowId);
			}

			// This is a follow across bubbles. Stop them.
			PyObject *val = PyLong_FromLongLong(src->mId);
			PyList_Append(ret,val);
			Py_DECREF(val);
		}
		else
		{
			continue;
		}

	}

	return ret;
}



//---------------------------------------------------------------------------------------
// Ballpark::Evolve
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyEvolve(
	PyObject* args
	)
{
	//Evolve(mTime);
	Evolve(0);
	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::GetActiveBoxes
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyGetActiveBoxes(
	PyObject* args
	)
{
	int level;

	if (!PyArg_ParseTuple(args, "i",
		&level
		))
		return NULL;

	PyObject *ret = 0;

	if(mPartition)
	{
		ret = mPartition->GetActiveBoxes(level);
	}

	if(ret)
		return ret;

	Py_INCREF(Py_None);
	return Py_None;
}
//---------------------------------------------------------------------------------------
// Ballpark::PyAddProximitySensor
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyAddProximitySensor(
	PyObject* args
	)
{
	ID srcId;
	float range;
	double period = 2.0;
	int shuffle = 0;
	char onlyInteractives = 0;

	if (!PyArg_ParseTuple(args, "Lf|dib",
		&srcId,
		&range,
		&period,
		&shuffle,
		&onlyInteractives
		))
		return NULL;

	AddProximitySensor(srcId, range, period, shuffle, !!onlyInteractives);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyRemoveProximitySensor
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyRemoveProximitySensor(
	PyObject* args
	)
{
	ID srcId;

	if (!PyArg_ParseTuple(args, "L",
		&srcId
		))
		return NULL;

	RemoveProximitySensor(srcId);


	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* Ballpark::PyGetBallIdsInRange(
	PyObject* args
	)
{
	return GetBallIdsInRange(args,false);
}

PyObject* Ballpark::PyGetBallIdsAndDistInRange(
    PyObject* args
	)
{
	return GetBallIdsInRange(args,true);
}

PyObject* Ballpark::GetBallIdsInRange(
    PyObject* args,
	bool includeDist
	)
{
    ID srcId;
	double range;
	char includeCloaked = 0;
	double x,y,z;
	Vector3d center;
	long bubbleID = -1;
	bool useBall=true;

	if (!PyArg_ParseTuple(args, "Ld|b",
		&srcId,
		&range,
		&includeCloaked
		))
	{
		PyErr_Clear();
		// This didn't go in, check the alternate semantic
		if (!PyArg_ParseTuple(args, "dddd|b",
			&x,
			&y,
			&z,
			&range,
			&includeCloaked
			))
		{
			return 0;
		}

		useBall = false;
	}

	Ball *otherBall=0;
	Ball *refBall=0;

	if(useBall)
	{
		refBall = mBalls[srcId];
		if(!refBall)
		{
			PyErr_Format(PyExc_RuntimeError, "GetBallIdsInRange: Ball %" CCP_INT64_FORMAT " not in park", srcId);
			return 0;
		}

		center = refBall->mNewPos;
		bubbleID = refBall->mNewBubble;

	}else
	{
		center = Vector3d(x,y,z);
		Box *top = mPartition->GetExistingTopBox(center);
		if(top)
		{
			bubbleID = top->mBubble;
		}
	}


	if(!bubbles)
	{
		PyErr_Format(PyExc_RuntimeError, "GetBallIdsInRange: bubbles not initialized yet");
		return 0;
	}


	PyObject* ret = PyList_New(0);

	// If we don't have a bubble at the current location, return an empty list
	if(bubbleID==-1)
		return ret;

	TASKLET(Timer_GetBallIdsInRange);

	// Cycle over all balls in same bubble
	PyObject *newBubbleId = PyLong_FromLong(bubbleID);
	PyObject *theBubble = PyDict_GetItem(bubbles, newBubbleId);

	if(theBubble)
	{
		// Cycle over all balls
		Py_ssize_t pos = 0;
		PyObject *key;
		PyObject *value;
		while (GetNextValidBallInBubble(theBubble, pos, key, value, otherBall, refBall))
		{
			if( otherBall->isCloaked && !includeCloaked )
			{
				continue;
			}
			double rangeChecked = (otherBall->mNewPos - center).LengthSq();
			if (rangeChecked <= (range + otherBall->mRadius) * (range + otherBall->mRadius))
			{
				if(includeDist)
				{
					PyObject *tuple = PyTuple_New(2);
					PyTuple_SET_ITEM(tuple,0,PyFloat_FromDouble(rangeChecked));
					PyTuple_SET_ITEM(tuple,1,PyLong_FromLongLong(otherBall->mId));
					PyList_Append(ret,tuple);
					Py_DECREF(tuple);
				}
				else
				{
					PyObject *val = PyLong_FromLongLong(otherBall->mId);
					PyList_Append(ret,val);
					Py_DECREF(val);
				}
			}
		}

	}

	Py_DECREF(newBubbleId);

	return ret;
}

PyObject* Ballpark::GetBallIdsInRangeOfTriangle(
	PyObject* args
	)
{
	ID srcId;
	Vector3d u;
	Vector3d v;
	double range;

	if (!PyArg_ParseTuple(args, "Lddddddd",
		&srcId,
		&u.x,
		&u.y,
		&u.z,
		&v.x,
		&v.y,
		&v.z,
		&range
		))
	{
		return 0;
	}

	Ball *otherBall = NULL;
	Ball *refBall = mBalls[srcId];
	if(!refBall)
	{
		RaiseBallNotInParkError("GetBallIdsInRangeOfTriangle", srcId);
		return 0;
	}
	PyObject *theBubble = GetActiveBubbleForBall(refBall);
	PyObject* ret = PyList_New(0);
	const Vector3d center = refBall->mNewPos;
	
	if(theBubble)
	{
		// Cycle over all balls
		Py_ssize_t pos = 0;
		PyObject *key;
		PyObject *value;

		Triangle t(center, center + u, center + v);

		while (GetNextValidBallInBubble(theBubble, pos, key, value, otherBall, refBall))
		{	
			Vector3d closestPoint = t.GetClosestPoint(otherBall->mNewPos);
			double rangeChecked = (otherBall->mNewPos - closestPoint).LengthSq();
			if (rangeChecked <= (range + otherBall->mRadius) * (range + otherBall->mRadius))
			{
				PyObject *val = PyLong_FromLongLong(otherBall->mId);
				PyList_Append(ret,val);
				Py_DECREF(val);
			}
		}
	}

	return ret;
}


PyObject* Ballpark::GetBallIdsInCone(
	PyObject* args
	)
{
	ID srcId;
	Vector3d v;
	double angle;

	if (!PyArg_ParseTuple(args, "Ldddd",
		&srcId,
		&v.x,
		&v.y,
		&v.z,
		&angle
		))
	{
		return 0;
	}

	Ball *otherBall = NULL;
	Ball *refBall = mBalls[srcId];
	if(!refBall)
	{
		RaiseBallNotInParkError("GetBallIdsInCone", srcId);
		return 0;
	}
	PyObject *theBubble = GetActiveBubbleForBall(refBall);
	PyObject* ret = PyList_New(0);
	const Vector3d center = refBall->mNewPos;
	double coneHeight = v.Length();
	v.Normalize();
	
	if(theBubble)
	{
		// Cycle over all balls
		Py_ssize_t pos = 0;
		PyObject *key;
		PyObject *value;

		Vector3d u;
		Vector3d d;
		
		double sinAngle = sin(angle);
		double cosAngle = cos(angle);
		double sinReciprocal;
		if( sinAngle > 0.0 )
		{
			sinReciprocal = 1.0 / sinAngle;
		}
		else
		{
			sinReciprocal = 0;
		}
		double sinSq = sinAngle * sinAngle;
		double cosSq = cosAngle * cosAngle;

		while (GetNextValidBallInBubble(theBubble, pos, key, value, otherBall, refBall))
		{	
			double ballRadSq = otherBall->mRadius * otherBall->mRadius;
			double maxDist = (coneHeight + otherBall->mRadius);
			if( maxDist*maxDist < (refBall->mNewPos - otherBall->mNewPos).LengthSq() ) // The cone has a rounded bottom, not a flat one.
			{
				continue; // We are too far away from the apex (The cone is not big enough to reach us).
			}

			// Check if we are inside the infinite cone defined by the source vector, normalized direction and angle.
			// Reference documentation on the algorithm for the curious: http://www.geometrictools.com/Documentation/IntersectionSphereCone.pdf
			u = refBall->mNewPos - v * (otherBall->mRadius * sinReciprocal);
			d = otherBall->mNewPos - u;
			double dSq = d*d;
			double e = v*d;
			if( e>0 && e*e > dSq * cosSq)
			{
				d = otherBall->mNewPos - refBall->mNewPos;
				dSq = d*d;
				e = -(v*d);
				if( e>0 &&  e*e >= dSq * sinSq )
				{
					if( dSq > ballRadSq )
					{
						continue;
					}
				}
				PyObject *val = PyLong_FromLongLong(otherBall->mId);
				PyList_Append(ret,val);
				Py_DECREF(val);
			}
		}
	}
	return ret;
}

PyObject* Ballpark::GetActiveBubbleForBall(const Ball* ball)
{
	if(!ball)
	{
		PyErr_Format(PyExc_RuntimeError, "GetActiveBubbleForBall: Ball is NULL");
		return 0;
	}

	const long bubbleID = ball->mNewBubble;

	if(!bubbles)
	{
		PyErr_Format(PyExc_RuntimeError, "GetActiveBubbleForBall: bubbles not initialized yet");
		return 0;
	}

	// If we don't have a bubble at the current location, return an empty dict
	if(bubbleID==-1)
	{
		return PyDict_New();
	}
	
	PyObject *newBubbleId = PyLong_FromLong(bubbleID);
	PyObject *theBubble = PyDict_GetItem(bubbles, newBubbleId);
	Py_DECREF(newBubbleId);
	return theBubble;
}

bool Ballpark::BallIsValidForIteration(ID ballID, Ball*& ball, const Ball* refBall)
{
	if (ballID == -1 && PyErr_Occurred()) 
	{
		//bogus ball?  continue.
		PyErr_Clear();
		return false;
	}

	ball = mBalls[ballID];
	if(!ball)
	{
		return false;
	}

	if(ball->isMoribund)
	{
		return false;
	}

	if(ball==refBall)
	{
		return false;
	}
	
	return true;
}

int Ballpark::GetNextValidBallInBubble(PyObject* theBubble, Py_ssize_t& pos, PyObject*& key, PyObject*& value, Ball*& otherBall, const Ball* refBall)
{
	int validBallIndexInDict = 0;
	while(!validBallIndexInDict)
	{
		validBallIndexInDict = PyDict_Next(theBubble, &pos, &key, &value);
		if(validBallIndexInDict)
		{
			ID ballID = PyLong_AsLongLong(key);
			if(!BallIsValidForIteration(ballID, otherBall, refBall))
			{
				validBallIndexInDict = 0;
			}
		}
		else
		{
			return validBallIndexInDict;
		}
	}
	return validBallIndexInDict;
}

double GetSquaredDistanceFromClosestPointOnLineSegment(const Vector3d& a, const Vector3d& ab, const Vector3d& p)
/*
	A is the start point of the line segment AB
	AB is the vector from A to B
	P is the point for which we want to find the squared distance
*/
{
	Vector3d ap = p - a;
	double c1 = ap * ab;
			
	if(c1 <= 0)
	{
		// A is the closest point
		return ap.LengthSq();
	}
	double c2 = ab * ab;
	if( c2 <= c1 )
	{
		// B is the closest point
		Vector3d b = a + ab;
		return (p - b).LengthSq();
	}
	
	// The closest point is between the ends of the line segment
	double b = c1 / c2;
	Vector3d closestPoint = a + b * ab;
	return (closestPoint - p).LengthSq();
}

PyObject* Ballpark::GetBallIdsInCapsule(
	PyObject* args
	)
{
	ID srcId;
	Vector3d v;
	double radius;

	if (!PyArg_ParseTuple(args, "Ldddd",
		&srcId,
		&v.x,
		&v.y,
		&v.z,
		&radius
		))
	{
		return 0;
	}
	Ball *otherBall = NULL;
	Ball *refBall = mBalls[srcId];
	if(!refBall)
	{
		RaiseBallNotInParkError("GetBallIdsInCapsule", srcId);
		return 0;
	}
	PyObject *theBubble = GetActiveBubbleForBall(refBall);
	PyObject* ret = PyList_New(0);
	const Vector3d center = refBall->mNewPos;

	if(theBubble)
	{
		// Cycle over all balls
		Py_ssize_t pos = 0;
		PyObject *key;
		PyObject *value;

		while (GetNextValidBallInBubble(theBubble, pos, key, value, otherBall, refBall))
		{	
			double distanceSq = GetSquaredDistanceFromClosestPointOnLineSegment(center, v, otherBall->mNewPos);
			double inflatedRadiusSq = (radius + otherBall->mRadius) * (radius + otherBall->mRadius);
			if (distanceSq <= inflatedRadiusSq)
			{
				PyObject *val = PyLong_FromLongLong(otherBall->mId);
				PyList_Append(ret,val);
				Py_DECREF(val);
			}
		}
	}

	return ret;
}

PyObject* Ballpark::PyGetCenterDist(
    PyObject* args
	)
{
    ID id1;
    ID id2;
	if (!PyArg_ParseTuple(args, "LL",
		&id1,
		&id2
		))
	{
		return NULL;
	}

	Ball *ball1,*ball2;
	ball1 = mBalls[id1];
	ball2 = mBalls[id2];
	if (ball1 && ball2)
		return PyFloat_FromDouble((ball1->mNewPos - ball2->mNewPos).Length());

	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* Ballpark::PyGetSurfaceDist(
    PyObject* args
	)
{
    ID id1;
    ID id2;
	if (!PyArg_ParseTuple(args, "LL",
		&id1,
		&id2
		))
	{
		return NULL;
	}

	Ball *ball1,*ball2;
	ball1 = mBalls[id1];
	ball2 = mBalls[id2];
	if (ball1 && ball2)
		return PyFloat_FromDouble((ball1->mNewPos - ball2->mNewPos).Length()-ball1->mRadius - ball2->mRadius);

	Py_INCREF(Py_None);
	return Py_None;
}
//---------------------------------------------------------------------------------------
// Ballpark::PyCheckVisibility
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyCheckVisibility(
	PyObject* args
	)
{
	ID srcId;
	ID dstId;

	if (!PyArg_ParseTuple(args, "LL",
		&srcId,
		&dstId
		))
		return NULL;

	ID visible = CheckVisibility(srcId,dstId);

	return PyLong_FromLongLong(visible);
}


//---------------------------------------------------------------------------------------
// Ballpark::PyCloakBall
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyCloakBall(
	PyObject* args
	)
{
	ID srcId;
	char cloakMode = DSTNORMALCLOAK; 
	float range = 2000.0; // Note: optional argument

	if (!PyArg_ParseTuple(args, "Lb|f",
		&srcId,
		&cloakMode,
		&range
		))
		return NULL;

	CloakBall(srcId, cloakMode, range);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyUncloakBall
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyUncloakBall(
	PyObject* args
	)
{
	ID srcId;

	if (!PyArg_ParseTuple(args, "L",
		&srcId
		))
		return NULL;

	UncloakBall(srcId);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBallFormation
//
// WARNING:
// The formation code is old abandoned code that needs to get removed or seriously worked on,
// because it relies on orientation which is not guaranteed to be equal between server and client
// because ball orientation gets "snapped" when a new ball is added to the park, which can result
// in inconsistencies between client/server when it comes to rotation for that tick,
// at least when using the old collision method. This formation code is not used for the formation
// flight which is currently available in Eve Online,
// that formation flight behavior was put together in Python
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBallFormation(
	PyObject* args
	)
{
	ID srcId;
	int formationID;

	if (!PyArg_ParseTuple(args, "Li",
		&srcId,
		&formationID
		))
		return NULL;

	SetBallFormation(srcId, formationID);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyLoadFormations
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyLoadFormations(
	PyObject* args
	)
{
	PyObject *formationList;

	if (!PyArg_ParseTuple(args, "O!",
		&PyTuple_Type,
		&formationList
		))
		return NULL;

	mFormations.clear();

	size_t numberOfFormations = PyTuple_Size(formationList);
	for(size_t i = 0; i < numberOfFormations; i++)
	{
		PyObject *formation = PyTuple_GetItem(formationList, i);
		PyObject *vectorList = PyTuple_GetItem(formation, 1);
		AddFormation(vectorList);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PySetBoxObject()
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PySetBoxObject(
	PyObject* args
	)
{
	PyObject *object;
	double x,y,z;

	if (!PyArg_ParseTuple(args, "dddO!",
		&x,
		&y,
		&z,
		&PyDict_Type,
		&object
		))
		return NULL;

	Box *top = mPartition->GetExistingTopBox(Vector3d(x,y,z));

	int success = 0;
	if(top)
	{
		top->SetObject(object);
		success=1;
	}

	return PyLong_FromLong(success);
}

//---------------------------------------------------------------------------------------
// Ballpark::PyGetBoxObject()
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyGetBoxObject(
	PyObject* args
	)
{
	double x,y,z;
	if (!PyArg_ParseTuple(args, "ddd",
		&x,
		&y,
		&z
		))
		return NULL;

	Box *top = mPartition->GetExistingTopBox(Vector3d(x,y,z));

	if(top)
	{
		if(top->mObject)
		{
			Py_INCREF(top->mObject);
			return top->mObject;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyGetBubbleAtCoordinates()
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyGetBubbleAtCoordinates(
	PyObject* args
	)
{
	double x,y,z;
	if (!PyArg_ParseTuple(args, "ddd",
		&x,
		&y,
		&z
		))
		return NULL;

	Box *top = mPartition->GetExistingTopBox(Vector3d(x,y,z));

	if(top)
		return PyLong_FromLong(top->mBubble);

	Py_INCREF(Py_None);
	return Py_None;
}

//---------------------------------------------------------------------------------------
// Ballpark::PyCopyBubbles
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyCopyBubbles(
	PyObject* args
	)
{
	CopyBubbles();
	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* Ballpark::PyGetAccuracy(
	PyObject* args
	)
{
	ID srcId;
	ID dstId;

	double fallOff;
	double optimalRange;
	double trackingSpeed;
	double signatureRadius; 
	double optimalSignatureRadius; // Note: optional argument

	if (!PyArg_ParseTuple(args,"LLddddd",
		&srcId,
		&dstId,
		&optimalRange,
		&fallOff,
		&trackingSpeed,
		&signatureRadius,
		&optimalSignatureRadius)
		)
		return NULL;

	
	double acc = GetAccuracy(srcId, dstId, optimalRange, fallOff, trackingSpeed, signatureRadius, optimalSignatureRadius);

	PyObject *list = PyList_New(3);
	PyList_SET_ITEM(list,0,PyFloat_FromDouble(acc));
	PyList_SET_ITEM(list,1,PyFloat_FromDouble(optimalRange));
	PyList_SET_ITEM(list,2,PyFloat_FromDouble(trackingSpeed));

	return list;
}


//---------------------------------------------------------------------------------------
// Ballpark::PyApplyDump()
//---------------------------------------------------------------------------------------
PyObject* Ballpark::PyReadFullStateFromStream(
	PyObject* args
	)
{
	PyObject* s;
	int partial = 0;
	if (!PyArg_ParseTuple(args, "O|i", &s, &partial))
		return NULL;

	IBlueStreamPtr sp(s);
	if( !sp )
	{
		PyErr_SetString( PyExc_TypeError, "Argument 1 is not a Blue stream" );
		return NULL;
	}

	bool errorCondition = false;
	char packet;

	if(!partial)
	{
		mFreeBalls.clear();
		mProximityBalls.clear();
		if(bubbleInteractives)
			PyDict_Clear(bubbleInteractives);
	}

	byteCount = 0;
	size_t tmpCnt = 0;

	sp->Seek(0, ICcpStream::SO_BEGIN);
	while((tmpCnt = sp->Read(&packet,1)) > 0)
	{
		byteCount += tmpCnt;
		switch(packet)
		{
		case DESTINY_FULLSTATE:
		case DESTINY_BALLS:
			{
				errorCondition = ReadFullStateFromStream(sp, partial);
				break;
			}
		default:
			{
				PyErr_SetString(PyExc_ValueError, "Unknown packet type");
				return 0;
			}
		}

		if(errorCondition)
		{
			PyErr_SetString(PyExc_ValueError, "Some error reading packet");
			return 0;
		}
	}

	CCP_LOG_CH( s_chPark,"Read %d bytes from stream", byteCount);

	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* Ballpark::PyWriteFullStateToStream(
	PyObject* args
	)
{
	PyObject* s;
	ID srcID = -1;
	if (!PyArg_ParseTuple(args, "O|L",&s, &srcID))
		return NULL;

	IBlueStreamPtr sp(s);
	if( !sp )
	{
		PyErr_SetString( PyExc_TypeError, "Argument 1 is not a Blue stream" );
		return NULL;
	}
	
	int32_t timestamp = int32_t( mCurrentTime );
	char packet = DESTINY_FULLSTATE;

	sp->Seek(0,ICcpStream::SO_BEGIN);
	byteCount = sp->Write(&packet,sizeof(packet));
	byteCount += sp->Write(&timestamp,sizeof(timestamp));

	BallIterator it(&mBalls);
	Ball* b;
	Ball *src = 0;

	if(srcID!=-1)
		src = mBalls[srcID];

	while( ( b = it++ ) )
	{
		if(b->mId < DSTLOCALBALLS)
			continue;

		if(src && isMaster)
		{
			// We are on the server, and we are retrieving a bubble limited state
			if(src->mNewBubble != b->mNewBubble && !b->isGlobal)
				continue;

			if(b->isCloaked && b!=src)
				continue;
		}

		WriteBallToStream(b,sp);

	} // End of writing balls
	
	// Seek to start of stream
	sp->Seek(0,ICcpStream::SO_BEGIN);

	CCP_LOG_CH( s_chPark,"Wrote %d bytes to stream", byteCount);
	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* Ballpark::PyWriteBallsToStream(
	PyObject* args
	)
{
	TASKLET(Timer_WriteBallsToStream);

	PyObject* s;
	PyObject* ballList;
	if (!PyArg_ParseTuple(args, "O!O",&PyList_Type, &ballList, &s))
		return NULL;

	IBlueStreamPtr sp(s);
	if( !sp )
	{
		PyErr_SetString( PyExc_TypeError, "Argument 1 is not a Blue stream" );
		return NULL;
	}

	size_t cnt = PyList_Size(ballList);

	int32_t timestamp = int32_t( mCurrentTime );
	char packet = DESTINY_BALLS;

	// Seek to start of stream
	sp->Seek(0,ICcpStream::SO_BEGIN);

	// Write the header
	byteCount = sp->Write(&packet,sizeof(packet));
	byteCount += sp->Write(&timestamp,sizeof(timestamp));

	for(size_t i=0; i < cnt ; i++)
	{
		PyObject *val = PyList_GET_ITEM(ballList, i);
		ID ballID = PyLong_AsLongLong(val);
		if (ballID == -1 && PyErr_Occurred()) {
			//bogus id.  continue
			PyErr_Clear();
			continue;
		}
		
		Ball *ball = mBalls[ballID];

		if(!ball)
			continue;

		WriteBallToStream(ball, sp);

	}

	// Seek to start of stream
	sp->Seek(0,ICcpStream::SO_BEGIN);

	CCP_LOG_CH( s_chPark,"Wrote %d bytes to stream", byteCount);
	Py_INCREF(Py_None);
	return Py_None;
}


PyObject* Ballpark::PyGetBallBox(
	PyObject* args
	)
{
	ID srcID;

	if (!PyArg_ParseTuple(args, "L",
		&srcID
		))
		return NULL;

	Ball *ball = mBalls[srcID];

	if(!ball)
	{
		Py_INCREF(Py_None);
		return Py_None;
	}

	// Find the box of that ball
	Box *box  = ball->mMyBox;

	if(!box)
	{
		Py_INCREF(Py_None);
		return Py_None;
	}

	PyObject *ret = PyTuple_New(2);
	PyObject *pos = PyTuple_New(3);
	Vector3d p = box->lo + (box->hi - box->lo)*0.5;

	PyTuple_SET_ITEM(pos, 0, PyFloat_FromDouble( p.x ));
	PyTuple_SET_ITEM(pos, 1, PyFloat_FromDouble( p.y ));
	PyTuple_SET_ITEM(pos, 2, PyFloat_FromDouble( p.z ));

	PyTuple_SET_ITEM(ret, 0, PyFloat_FromDouble( box->hi.x - box->lo.x));
	PyTuple_SET_ITEM(ret, 1, pos);

	return ret;
}

PyObject* Ballpark::PyGetStaticCollidableBox(
	PyObject* args
	)
{
	ID srcID;

	if (!PyArg_ParseTuple(args, "L",
		&srcID
		))
		return NULL;

	StaticCollidable *collidable;
	auto it = mStaticCollidables.find(srcID);
	if( it != mStaticCollidables.end() )
	{
		collidable = it->second;
	}
	else
	{
		Py_INCREF(Py_None);
		return Py_None;
	}

	// Find the box of that ball
	Box *box  = collidable->mMyBox;

	if(!box)
	{
		Py_INCREF(Py_None);
		return Py_None;
	}

	PyObject *ret = PyTuple_New(2);
	PyObject *pos = PyTuple_New(3);
	Vector3d p = box->lo + (box->hi - box->lo)*0.5;

	PyTuple_SET_ITEM(pos, 0, PyFloat_FromDouble( p.x ));
	PyTuple_SET_ITEM(pos, 1, PyFloat_FromDouble( p.y ));
	PyTuple_SET_ITEM(pos, 2, PyFloat_FromDouble( p.z ));

	PyTuple_SET_ITEM(ret, 0, PyFloat_FromDouble( box->hi.x - box->lo.x));
	PyTuple_SET_ITEM(ret, 1, pos);

	return ret;
}

PyObject* Ballpark::PyGetBoxCenter(
	PyObject* args
	)
{
	Vector3d pos;
	int level;

	if (!PyArg_ParseTuple(args, "iddd",
		&level,
		&pos.x,
		&pos.y,
		&pos.z
		))
		return NULL;

	if(level < 0 || level >= mPartition->mNumberOfLevels)
	{
		PyErr_SetString(PyExc_TypeError, "Illegal level");
		return 0;
	}

	mPartition->GetBoxCenter(level, pos);

	PyObject *ret = PyTuple_New(3);
	PyTuple_SET_ITEM(ret, 0, PyFloat_FromDouble( pos.x ));
	PyTuple_SET_ITEM(ret, 1, PyFloat_FromDouble( pos.y ));
	PyTuple_SET_ITEM(ret, 2, PyFloat_FromDouble( pos.z ));

	return ret;
}

PyObject* Ballpark::PyScanCone(
	 PyObject* args
	 )
{
	double angle;
	double range;
	Vector3d dir;
	ID srcID;

	if (!PyArg_ParseTuple(args, "Lddddd",
		&srcID,
		&angle,
		&range,
		&dir[0],
		&dir[1],
		&dir[2]
		))
		return NULL;

	Ball *srcBall = mBalls[srcID];

	// Check validity of arguments
	if(range <= 0.0  || !srcBall)
	{
		Py_INCREF(Py_None);
		return Py_None;
	}

	dir.Normalize();

	if(dir.LengthSq() < 0.5)
	{
		Py_INCREF(Py_None);
		return Py_None;
	}

	// Create a list to hold the return values
	PyObject *addList = PyList_New(0);

	// Cycle over all balls
	BallIterator it(&mBalls);
	Ball *ball;
	Vector3d d0;
	double dird0;
	double d0d0;
	double r2 = range*range;

	// The angle is the half angle of the cone
	angle = angle*0.5;
	double c2 = cos(angle)*cos(angle);
	bool sphere = false;

	if(angle > PI)
	{
		// We are only interested in range
		sphere = true;
	}

	while( ( ball = it++ ) )
	{
		if(ball->mId==srcID || ball->mId < 0 || ball->isCloaked)
			continue;

		d0 = ball->mNewPos - srcBall->mNewPos;
		dird0 = dir*d0;
		d0d0 = d0*d0;

		
		if( (sphere || (dird0 >= 0.0 && dird0*dird0 >= c2*d0d0)) && d0d0 <= r2)
		{
			// Add ball to list
			PyObject *aKey = PyLong_FromLongLong(ball->mId);
			PyList_Append(addList,aKey);
			Py_DECREF(aKey);
		}
		else
		{
			continue;
		}
	}

	return addList;
}

bool Ballpark::ReadFullStateFromStream(IBlueStreamPtr s, int partial)
{
	int32_t timestamp;

	byteCount += s->Read(&timestamp,sizeof(timestamp));

	mCurrentTime = timestamp;
	Ball *ball;
	VectorOfBalls l;
	while( ( ball = ReadBallFromStream(s, partial) ) )
	{
		l.push_back(ball);
	}

	VectorOfBalls::iterator addition;
	Ball* b;
	for(addition=l.begin(); addition != l.end(); ++addition)
	{
		b = *addition;
		switch(b->mMode)
		{
		case DSTBALL_FOLLOW:
		case DSTBALL_MISSILE:
		case DSTBALL_ORBIT:
		case DSTBALL_FORMATION:
			{
				Ball *dest = mBalls[ID(b->mFollowId)];
				CCP_ASSERT(dest);
				if (!dest) {
					CCP_LOGERR_CH( s_chPark, "ball %I64d to follow not found for ball %I64d", b->mFollowId, b->mId);
					b->mMode = DSTBALL_GOTO;
				} else {
					b->mFollowPtr = dest;
					dest->mFollowers.insert(b->mId);
				}
				break;
			}
		default:
			{
				break;
			}
		}
	}
	return false;
}

void Ballpark::PopulateAddDelForBubble(PyObject *bubble, PyObject *bubbleAddList, PyObject *bubbleDelList)
{
	PyObject *key, *value;
	Py_ssize_t pos = 0;
	// Cycle over balls in the bubble, collecting added and deleted balls
	while (PyDict_Next(bubble, &pos, &key, &value))
	{
		ID ballID = PyLong_AsLongLong(key);
		if(ballID == -1 && PyErr_Occurred())
		{
			// Something wrong with key
			PyObject *e, *v, *t;
			PyErr_Fetch(&e, &v, &t);
			PyObject *repr = PyObject_Repr(key);
			CCP_LOGERR_CH( s_chPark,"%s, %d: Key is not __int64: type=%s value=%s", __FILE__, __LINE__, key->ob_type->tp_name, repr? PyUnicode_AsUTF8(repr):"<bork>");
			Py_XDECREF(repr);
			PyErr_Restore(e, v, t);
			return;
		}

		long action = PyLong_AS_LONG(value);

		if(ballID < DSTLOCALBALLS)
			continue;

		Ball *ball = mBalls[ballID];
		if(action != -1) // The ball might already be deleted...
		{				
			if(!ball || ball->isGlobal || ball->isCloaked)
			{
				if(mProbe==ballID)
					CCP_LOGWARN_CH( s_chPark, "[%d] Skipping myself in same", mCurrentTime);
				continue;
			}
		}
		else
		{
			// Don't include cloaked balls removals
			if(ball && ball->isCloaked)
				continue;
		}

		if(action == 1)
		{
			PyObject *aKey = PyLong_FromLongLong(ballID);
			PyList_Append(bubbleAddList, aKey);
			Py_DECREF(aKey);				
			if(mProbe==ballID)
				CCP_LOGWARN_CH( s_chPark, "[%d] Adding myself to same", mCurrentTime);
		}
		else if(action == -1)
		{
			PyObject *aKey = PyLong_FromLongLong(ballID);
			PyList_Append(bubbleDelList, aKey);
			Py_DECREF(aKey);
			if(mProbe==ballID)
				CCP_LOGWARN_CH( s_chPark, "[%d] Deleting myself from same", mCurrentTime);
		}
	} // End of cycling over bubble
}

PyObject* Ballpark::PyAdditionsAndDeletions(PyObject* args)
{
	/*
	Pre-builds the add/del list for each bubble, which we can use for any user ship
	which hasn't changed bubbles this tick.  It looks something like this:
	   for each bubble:
		    if bubble has one or more user ship:
			    build a list of balls added/deleted this frame, and store in a per-bubble dict
	   for each user ship in a ballpark:
	       if ship is in same bubble as last tick:
	           store the bubble-id in the per-user list (can be used to look-up into the per-bubble list)
	       else:
	           determine the set of balls added/deleted for this user, taking account of the cross-bubble movement
	           add this set to a per-user list

	We return the per-user add/del dicts, and also per-bubble add/del dicts.
	*/

	TASKLET(Timer_AdditionsAndDeletions);
	PyObject *additionsPerPlayer = NULL, *deletionsPerPlayer = NULL; 
	PyObject *additionsPerBubble = NULL, *deletionsPerBubble = NULL;
	PyObject *userShips = NULL;

	if (!PyArg_ParseTuple(args, "O!O!O!O!O!",
		&PyDict_Type, &additionsPerPlayer,
		&PyDict_Type, &deletionsPerPlayer,
		&PyDict_Type, &additionsPerBubble,
		&PyDict_Type, &deletionsPerBubble,
		&PyList_Type, &userShips
		))
	{
		return NULL;
	}

	if( !bubbles )
	{
		PyErr_SetString( PyExc_RuntimeError, "PyAdditionsAndDeletions: Bubbles not initialized yet" );
		return nullptr;
	}

	// Clear whatever is here
	PyDict_Clear(additionsPerPlayer);
	PyDict_Clear(deletionsPerPlayer);
	PyDict_Clear(additionsPerBubble);
	PyDict_Clear(deletionsPerBubble);

	// For each bubble, pre-build a list of added/deleted balls, as observed by a user
	// who was in that bubble last frame and is still there this frame.
	// (For heavily loaded ballparks (eg a fleet battle), this will probably be most of them)
	// Then, when iterating over the users, for any user who hasn't changed bubbles, 
	// we can simply give them the bubble-id as an index into these add/delete lists.
	// For users who have changed bubble since the last tick, we'll need to do a per-user
	// operation, accounting for the difference between their old and new bubbles
	PyObject *key1, *bubble;
	Py_ssize_t pos1 = 0;

	// Cycle over all bubbles in the park
	while (PyDict_Next(bubbles, &pos1, &key1, &bubble))
	{
		if (!PyDict_GetItem(bubbleInteractives, key1))
		{
			// If there are no interactives in this bubble, then we can skip pre-building a list of adds/dels
			// as there is no-one to be interested in them
			continue;
		}

		PyObject *bubbleAddList = PyList_New(0);
		PyDict_SetItem(additionsPerBubble, key1, bubbleAddList);
		Py_DECREF(bubbleAddList);

		PyObject *bubbleDelList = PyList_New(0);
		PyDict_SetItem(deletionsPerBubble, key1, bubbleDelList);
		Py_DECREF(bubbleDelList);

		PopulateAddDelForBubble(bubble, bubbleAddList, bubbleDelList);
	} // End of cycle over all bubbles in the park
	// We now have a pre-calculated set of added/deleted balls for each bubble containing one or more user ships

	size_t i;
	size_t bounds = PyList_Size(userShips);
	PyObject *shipBubbleTransitions = PyList_New(0);
	if( !shipBubbleTransitions )
	{
		CCP_LOGWARN_CH( s_chPark, "[%d] Failed to create new list for bubble transitions", mCurrentTime );
		PyErr_Clear();
	}

	for(i=0; i < bounds ; i++)
	{
		// Cycle over all interactives
		PyObject *key;
		PyObject *val = PyList_GET_ITEM(userShips, i);
		ID userID = PyLong_AsLongLong(val);

		if(userID == -1 && PyErr_Occurred())
		{
			// Something wrong with val
			PyObject *e, *v, *t;
			PyErr_Fetch(&e, &v, &t);
			PyObject *repr = PyObject_Repr(val);
			CCP_LOGERR_CH( s_chPark,"%s, %d: Value is not __int64: type=%s value=%s", __FILE__, __LINE__, val->ob_type->tp_name, repr? PyUnicode_AsUTF8(repr):"<bork>");
			Py_XDECREF(repr);
			Py_XDECREF(shipBubbleTransitions);
			PyErr_Restore(e, v, t);
			return 0;
		}

		// This is the ball corresponding to the interactive
		Ball *uBall = mBalls[userID];

		if(!uBall)
			continue;

		PyObject *bubble = 0;
		PyObject *oldBubble = 0;
		if(mProbe==uBall->mId)
			CCP_LOGWARN_CH( s_chPark,"[%d] Checking adds and dels", mCurrentTime);

		if(uBall->mNewBubble != -1)
		{
			key = PyLong_FromLong(uBall->mNewBubble);
			bubble = PyDict_GetItem(bubbles, key);
			Py_DECREF(key);
		}

		if(uBall->mOldBubble != -1)
		{
			key = PyLong_FromLong(uBall->mOldBubble);
			oldBubble = PyDict_GetItem(bubbles, key);
			Py_DECREF(key);
		}

		// First, if old and new bubble haven't changed, then the updates are
		// straightforward

		bool isBubbleTheSame = uBall->mNewBubble == uBall->mOldBubble && uBall->mNewBubble != -1 && bubble;
		if(isBubbleTheSame)
		{
			// Use the list of per-bubbles adds/deletes that we calculated earlier

			PyObject *bubbleId;

			bubbleId = PyLong_FromLong(uBall->mNewBubble);

			if (PyDict_Contains(additionsPerBubble, bubbleId) && PyDict_Contains(deletionsPerBubble, bubbleId))
			{
				PyDict_SetItem(additionsPerPlayer, val, bubbleId);
				PyDict_SetItem(deletionsPerPlayer, val, bubbleId);
			}
			else
			{
				// It seems we didn't pre-build the add/del list earlier, due to believing there were no users in this bubble.
				// This appears to be a legitimate occurence when a pilot has boarded a ship whist being the only person in the bubble,
				// otherwise this might indicate something has gone wrong.
				CCP_LOGWARN_CH( s_chPark,"%s, %d: Interactive %I64d had no pre-calc add/del lists, despite not changing bubbles (bubble %d) - Perhaps the pilot has just boarded it in space?", __FILE__, __LINE__, userID, uBall->mNewBubble);

				// We need to build the list of adds/dels now, as it didn't get pre-built earlier
				PyObject *addList = PyList_New(0);
				PyObject *delList = PyList_New(0);

				PopulateAddDelForBubble(bubble, addList, delList);

				PyDict_SetItem(additionsPerPlayer, val, addList);
				PyDict_SetItem(deletionsPerPlayer, val, delList);

				Py_DECREF(addList);
				Py_DECREF(delList);
			}

			Py_DECREF(bubbleId);
		}
		else
		{
			// Create a list object mapped to the userID, both for additions and deletions
			PyObject *addList = PyList_New(0);
			PyDict_SetItem(additionsPerPlayer, val, addList);
			Py_DECREF(addList);

			PyObject *delList = PyList_New(0);
			PyDict_SetItem(deletionsPerPlayer, val, delList);
			Py_DECREF(delList);

			// I have probably changed bubbles
			// Cycle over elements of the new bubble
			Py_ssize_t pos = 0;
			PyObject *value;

			while (bubble && PyDict_Next(bubble, &pos, &key, &value))
			{
				ID ballID = PyLong_AsLongLong(key);
				if(ballID == -1 && PyErr_Occurred())
				{
					// Something wrong with key
					PyObject *e, *v, *t;
					PyErr_Fetch(&e, &v, &t);
					PyObject *repr = PyObject_Repr(key);			
					CCP_LOGERR_CH( s_chPark,"%s, %d: Key is not __int64: type=%s value=%s", __FILE__, __LINE__, key->ob_type->tp_name, repr? PyUnicode_AsUTF8(repr):"<bork>");

					Py_XDECREF(repr);
					Py_XDECREF(shipBubbleTransitions);
					PyErr_Restore(e, v, t);
					return 0;
				}
				long action = PyLong_AS_LONG(value);

				// Don't include miniballs
				if(ballID < DSTLOCALBALLS)
					continue;

				Ball *ball = mBalls[ballID];

				// Don't include global or cloaked balls
				if(ball && (ball->isGlobal || ball->isCloaked))
				{
					if(mProbe==uBall->mId && mProbe==ballID)
						CCP_LOGWARN_CH( s_chPark, "[%d] Skipping myself in new bubble", mCurrentTime);

					continue;
				}

				// Must check that this guy wasn't already there
				bool wasIn = false;
				if(oldBubble)
				{
					PyObject *v = PyDict_GetItem(oldBubble, key);
					if(v)
						wasIn = true;
				}

				if(action >= 0 && !wasIn)
				{
					PyObject *aKey = PyLong_FromLongLong(ballID);
					PyList_Append(addList,aKey);
					Py_DECREF(aKey);
					if(mProbe==uBall->mId && mProbe==ballID)
						CCP_LOGWARN_CH( s_chPark, "[%d] Adding myself because I wasn't in old bubble", mCurrentTime);
				}
				else if(action == -1 && (wasIn || !oldBubble)) // add deletions if the ball was in the old one, or if we are just coming in (probably a change of state)
				{
					// Will probably never happen
					PyObject *aKey = PyLong_FromLongLong(ballID);
					PyList_Append(delList, aKey);
					Py_DECREF(aKey);
					if(mProbe==uBall->mId && mProbe==ballID)
						CCP_LOGWARN_CH( s_chPark, "[%d] Deleting myself from new bubble", mCurrentTime);
				}

			} // End of cycling over new bubble

			pos = 0;
			while (oldBubble && PyDict_Next(oldBubble, &pos, &key, &value))
			{
				ID ballID = PyLong_AsLongLong(key);
				if(PyErr_Occurred())
				{
					// Something wrong with key
					PyObject *e, *v, *t;
					PyErr_Fetch(&e, &v, &t);
					PyObject *repr = PyObject_Repr(key);
					CCP_LOGERR_CH( s_chPark,"%s, %d: Key is not int: type=%s value=%s", __FILE__, __LINE__, key->ob_type->tp_name, repr? PyUnicode_AsUTF8(repr):"<bork>");

					Py_XDECREF(repr);
					Py_XDECREF(shipBubbleTransitions);
					PyErr_Restore(e, v, t);
					return 0;
				}

				long action = PyLong_AS_LONG(value);

				// Don't include miniballs
				if(ballID < DSTLOCALBALLS)
					continue;

				Ball *ball = mBalls[ballID];

				if(action != -1)
				{
					if(!ball)
						continue;

					// Don't include global balls or cloaked balls
					if(ball->isGlobal || ball->isCloaked)
					{
						if(mProbe==uBall->mId && mProbe==ballID)
							CCP_LOGWARN_CH( s_chPark, "[%d] Skipping myself in old bubble", mCurrentTime);

						continue;
					}
				}else
				{
					// Don't include cloaked ball removals
					if(ball && ball->isCloaked)
						continue;
				}

				// Must check that this guy isn't in the other place
				bool isIn = false;
				if(bubble)
				{
					PyObject *v = PyDict_GetItem(bubble, key);
					if(v)
						isIn = true;
				}

				if(!isIn && action <= 0)
				{
					PyObject *aKey = PyLong_FromLongLong(ballID);
					PyList_Append(delList, aKey);
					Py_DECREF(aKey);
					if(mProbe==uBall->mId && mProbe==ballID)
						CCP_LOGWARN_CH( s_chPark, "[%d] Removing myself from old bubble", mCurrentTime);
				}

			} // End of cycling over old bubble

			AddTransitionToList(uBall, shipBubbleTransitions);
		}
		uBall->mOldBubble = uBall->mNewBubble;

	} // End of cycling over interactives

	NotifyOfBubbleTransitions(shipBubbleTransitions);
	Py_XDECREF(shipBubbleTransitions);
	Py_INCREF(Py_None);
	return Py_None;
}

Ball* Ballpark::ReadBallFromStream(IBlueStreamPtr s, int partial)
{
	Ball *ball = 0;
	ID id;
	size_t tmpCnt;
	if((tmpCnt = s->Read(&id,sizeof(id))) > 0)
	{
		double mass = 1.0e34;
		float radius;
		Vector3d pos;
		uint8_t flags;
		int8_t isCloaked = 0;
		uint8_t mode;
		float maxVel = 0.0;
		Vector3d vel;
		float agility = 1.0;
		float speedFraction = 0.0;
		int64_t harmonic = -1;
		int32_t corporationID = -1;
		int32_t allianceID = -1;
		Quaternion rot( 0.f, 0.f, 0.f, 1.f );
		Vector3 angVel( 0.f, 0.f, 0.f );
		float rotAgility( 1.0f );
		float maxAngVel( 0.0f );

		byteCount += tmpCnt;

		byteCount += s->Read(&mode           , sizeof(mode));
		byteCount += s->Read(&radius         , sizeof(radius));
		byteCount += s->Read(&pos.x          , sizeof(Vector3d));
		byteCount += s->Read(&flags          , sizeof(flags));

		if(mode!=DSTBALL_RIGID)
		{
			byteCount += s->Read(&mass           , sizeof(mass));
			byteCount += s->Read(&isCloaked      , sizeof(isCloaked));
			byteCount += s->Read(&harmonic       , sizeof(harmonic));
			byteCount += s->Read(&corporationID  , sizeof(corporationID));
			byteCount += s->Read(&allianceID     , sizeof(allianceID));
		}

		if(flags & DSTBALL_ISFREE)
		{
			byteCount += s->Read(&maxVel,sizeof(maxVel));
			byteCount += s->Read(&vel.x,sizeof(Vector3d));
			byteCount += s->Read(&agility,sizeof(agility));
			byteCount += s->Read(&speedFraction,sizeof(speedFraction));
			if( g_useDynamicalOrientation )
			{
				byteCount += s->Read( &rot.x, sizeof( Quaternion ) );
				byteCount += s->Read( &maxAngVel, sizeof( maxAngVel ) );
				byteCount += s->Read( &angVel.x, sizeof( Vector3 ) );
				byteCount += s->Read( &rotAgility, sizeof( rotAgility ) );
			}
		}

		if( g_useDynamicalOrientation )
		{
			ball = AddDynamicallyOrientedBall(
				ID( id ), mass, radius, maxVel,
				maxAngVel,
				bool( flags & DSTBALL_ISFREE ),
				!!( flags & DSTBALL_ISGLOBAL ),
				!!( flags & DSTBALL_ISMASSIVE ),
				!!( flags & DSTBALL_ISINTERACTIVE ),
				!!( flags & DSTBALL_ISSPACEJUNK ),
				pos.x, pos.y, pos.z,
				vel.x, vel.y, vel.z,
				rot.x, rot.y, rot.z, rot.w,
				angVel.x, angVel.y, angVel.z,
				agility,
				rotAgility,
				speedFraction
			);
		}
		else
		{
			ball = AddOldStyleOrientedBall(
				ID(id), mass, radius, maxVel,
				bool(flags & DSTBALL_ISFREE),
				!!(flags & DSTBALL_ISGLOBAL),
				!!(flags & DSTBALL_ISMASSIVE),
				!!(flags & DSTBALL_ISINTERACTIVE),
				!!(flags & DSTBALL_ISSPACEJUNK),
				pos.x, pos.y, pos.z,
				vel.x, vel.y, vel.z,
				agility,
				speedFraction);
		}
		int8_t formationID;
		byteCount += s->Read(&formationID, sizeof(formationID));
		ball->mFormationID = formationID;

		if(partial!=1 && !(flags&DSTBALL_ISFREE))
        {
            for(ssize_t i=ball->mMiniBalls.GetSize()-1 ; i >= 0; i--)
            {
                MiniBall *mb = (MiniBall *)(void *)(ball->mMiniBalls.GetAt(i));
                RemoveBall(mb->mId);
            }
			ball->mMiniBalls.Remove(-1);

			for(ssize_t i=ball->mMiniCapsules.GetSize()-1 ; i >= 0; i--)
			{
				MiniCapsule *mc = (MiniCapsule *)(void *)(ball->mMiniCapsules.GetAt(i));
				RemoveCapsule(mc->mId);
			}
			ball->mMiniCapsules.Remove(-1);

			for(ssize_t i=ball->mMiniBoxes.GetSize()-1 ; i >= 0; i--)
			{
				MiniBox *mbo = (MiniBox *)(void *)(ball->mMiniBoxes.GetAt(i));
				RemoveOrientedBox(mbo->mId);
			}
			ball->mMiniBoxes.Remove(-1);
        }

		ball->mHarmonic = harmonic;
		ball->mCorporationID = corporationID;
		ball->mAllianceID = allianceID;
		ball->mMode = (DSTBALLMODE)mode;
		ball->isCloaked = isCloaked;
		int32_t effectStamp;
		
		switch(mode)
		{
		case DSTBALL_FOLLOW:
			{

				byteCount += s->Read(&ball->mFollowId,    sizeof(ball->mFollowId)    );
				byteCount += s->Read(&ball->mFollowRange, sizeof(ball->mFollowRange) );

				break;
			}
		case DSTBALL_FORMATION:
			{

				byteCount += s->Read(&ball->mFollowId,    sizeof(ball->mFollowId)    );
				byteCount += s->Read(&ball->mFollowRange, sizeof(ball->mFollowRange) );
				byteCount += s->Read(&effectStamp, sizeof(effectStamp) );
				ball->mEffectStamp = effectStamp;

				break;
			}
		case DSTBALL_MISSILE:
			{

				byteCount += s->Read(&ball->mFollowId,    sizeof(ball->mFollowId)    );
				byteCount += s->Read(&ball->mFollowRange, sizeof(ball->mFollowRange) );
				byteCount += s->Read(&ball->mOwnerId,     sizeof(ball->mOwnerId)     );
				byteCount += s->Read(&effectStamp, sizeof(effectStamp) );
				ball->mEffectStamp = effectStamp;
				byteCount += s->Read(&ball->mGoto.x,         sizeof(ball->mGoto)           );

				break;
			}
		case DSTBALL_ORBIT:
			{

				byteCount += s->Read(&ball->mFollowId,    sizeof(ball->mFollowId)    );
				byteCount += s->Read(&ball->mFollowRange, sizeof(ball->mFollowRange) );
				break;
			}
		case DSTBALL_GOTO:
			{
				byteCount += s->Read(&ball->mGoto.x,    sizeof(ball->mGoto)      );

				break;
			}
		case DSTBALL_WARP:
			{
				byteCount += s->Read(&ball->mGoto.x,      sizeof(ball->mGoto)        ); // Warp destination point
				byteCount += s->Read(&effectStamp, sizeof(effectStamp) ); // Timestamp at start of warp
				ball->mEffectStamp = effectStamp;
				byteCount += s->Read(&ball->mLastCollision, sizeof(ball->mLastCollision) ); // Total length of warp
				byteCount += s->Read(&ball->mFollowId,    sizeof(ball->mFollowId)    ); // Minimum range (as double)
				byteCount += s->Read(&ball->mOwnerId,    sizeof(ball->mOwnerId)     ); // Warp factor (speed multiplier)

				break;
			}
		case DSTBALL_MUSHROOM:
			{
				double span;

				byteCount += s->Read(&ball->mFollowRange, sizeof(ball->mFollowRange) );
				byteCount += s->Read(&span              , sizeof(span)               );
				byteCount += s->Read(&effectStamp, sizeof(effectStamp) );
				ball->mEffectStamp = effectStamp;
				byteCount += s->Read(&ball->mOwnerId,     sizeof(ball->mOwnerId)     );
				ball->mGoto.x = span;

				break;
			}
		case DSTBALL_TROLL:
			{
				byteCount += s->Read(&effectStamp, sizeof(effectStamp) );
				ball->mEffectStamp = effectStamp;

				break;
			}
		case DSTBALL_STOP:
		case DSTBALL_FIELD:
		case DSTBALL_RIGID:
			{
				// Nothing to do here
				break;
			}
		default:
			{
				CCP_LOGERR_CH( s_chPThunk,"ReadBallFromStream: Unknown ball mode %d in dump", mode);
				return 0;
				break;
			}
		}

		// Read in miniballs if any
		//CCP_LOG_CH( s_chPThunk,"ReadBallFromStream: ball %I64d has miniballflag set to %d", id, !!(flags&DSTBALL_HASMINIBALLS));
		if(flags&DSTBALL_HASMINIBALLS)
		{
			uint16_t cnt;
			byteCount += s->Read(&cnt, sizeof(cnt));
			for(int i = 0; i < cnt; i++)
			{
				Vector3d center;
				float radius;
				byteCount += s->Read(&center.x, sizeof(center));
				byteCount += s->Read(&radius, sizeof(radius));
				
				if(partial!=1)
                {
                    //CCP_LOG_CH( s_chPThunk,"Adding miniball %d for %I64d", i, id);
					ball->AddMiniBall(center.x, center.y, center.z, radius);
                }
			}
		}
		if(flags&DSTBALL_HASMINICAPSULES)
		{
			uint16_t capsuleCnt;
			byteCount += s->Read(&capsuleCnt, sizeof(capsuleCnt));
			for(int i = 0; i < capsuleCnt; i++)
			{
				Vector3d hemisphereA;
				Vector3d hemisphereB;
				float radius;
				byteCount += s->Read(&hemisphereA, sizeof(hemisphereA));
				byteCount += s->Read(&hemisphereB, sizeof(hemisphereB));
				byteCount += s->Read(&radius, sizeof(radius));

				if(partial!=1)
				{
					ball->AddMiniCapsule(
						hemisphereA.x, hemisphereA.y, hemisphereA.z, 
						hemisphereB.x, hemisphereB.y, hemisphereB.z, 
						radius);
				}
			}
		}
		if(flags&DSTBALL_HASMINIBOXES)
		{
			uint16_t boxCnt;
			byteCount += s->Read(&boxCnt, sizeof(boxCnt));
			for(int i = 0; i < boxCnt; i++)
			{
				Vector3d corner;
				Vector3d localX;
				Vector3d localY;
				Vector3d localZ;
				byteCount += s->Read(&corner, sizeof(corner));
				byteCount += s->Read(&localX, sizeof(localX));
				byteCount += s->Read(&localY, sizeof(localY));
				byteCount += s->Read(&localZ, sizeof(localZ));

				if(partial!=1)
				{
					ball->AddMiniBox(corner, localX, localY, localZ);
				}
			}
		}
	}

	return ball;
}

void Ballpark::WriteBallToStream(Ball* b, IBlueStreamPtr s)
{
	// Don't write moribund balls to the state stream.
	// They're actually not helping you recreate any state, 
	// since they're supposed to be fake.
	if( b->isMoribund )
	{
		return;
	}

	ID id = b->mId;
	double mass = b->mMass;
	float radius = b->mRadius;
	Vector3d pos = b->mNewPos;
	uint8_t flags = (b->isFree?DSTBALL_ISFREE:0) |(b->isGlobal?DSTBALL_ISGLOBAL:0) |(b->isMassive?DSTBALL_ISMASSIVE:0)|(b->isInteractive?DSTBALL_ISINTERACTIVE:0)|(b->isSpaceJunk?DSTBALL_ISSPACEJUNK:0);
	int8_t isCloaked = b->isCloaked;
	uint16_t miniBallCnt = (unsigned short)(b->mMiniBalls.GetSize());
	uint16_t miniBoxCnt = (unsigned short)(b->mMiniBoxes.GetSize());
	uint16_t miniCapsuleCnt = (unsigned short)(b->mMiniCapsules.GetSize());
    //CCP_LOG_CH( s_chPThunk,"WriteBallToStream: ball %d has %d miniballs", id, miniBallCnt);

	if(miniBallCnt)
		flags = flags | DSTBALL_HASMINIBALLS;
	if(miniCapsuleCnt)
		flags = flags | DSTBALL_HASMINICAPSULES;
	if(miniBoxCnt)
		flags = flags | DSTBALL_HASMINIBOXES;

	uint8_t mode = b->mMode;
	float maxVel = b->mMaxVel;
	Vector3d vel = b->mNewVel;
	float agility = b->mAgility;
	float speedFraction = b->mSpeedFraction;
	Quaternion rot = b->mNewRot;
	Vector3 angVel = b->mNewAngVel;
	float rotAgility = b->mRotAgility;
	float maxAngVel = b->mMaxAngVel;

	byteCount += s->Write(&id       , sizeof(id));
	byteCount += s->Write(&mode     , sizeof(mode));
	byteCount += s->Write(&radius   , sizeof(radius));
	byteCount += s->Write(&pos.x    , sizeof(Vector3d));
	byteCount += s->Write(&flags    , sizeof(flags));

	if(mode != DSTBALL_RIGID)
	{
		byteCount += s->Write(&mass     , sizeof(mass));
		byteCount += s->Write(&isCloaked, sizeof(isCloaked));
		byteCount += s->Write(&b->mHarmonic, sizeof(b->mHarmonic));
		int32_t corporationID = int32_t( b->mCorporationID );
		byteCount += s->Write(&corporationID, sizeof(corporationID));
		int32_t allianceID = int32_t( b->mAllianceID );
		byteCount += s->Write(&allianceID, sizeof(allianceID));
	}

	if(flags & DSTBALL_ISFREE)
	{
		byteCount += s->Write(&maxVel,sizeof(maxVel));
		byteCount += s->Write(&vel.x,sizeof(Vector3d));
		byteCount += s->Write(&agility,sizeof(agility));
		byteCount += s->Write(&speedFraction,sizeof(speedFraction));
		if( g_useDynamicalOrientation )
		{
			byteCount += s->Write( &rot.x, sizeof( Quaternion ) );
			byteCount += s->Write( &maxAngVel, sizeof( maxAngVel ) );
			byteCount += s->Write( &angVel.x, sizeof( Vector3 ) );
			byteCount += s->Write( &rotAgility, sizeof( rotAgility ) );
		}
	}

	int8_t formationID = int8_t( b->mFormationID );
	byteCount += s->Write(&formationID, sizeof(formationID));
	int32_t effectStamp = int32_t( b->mEffectStamp );

	switch(mode)
	{
	case DSTBALL_FOLLOW:
		{

			byteCount += s->Write(&b->mFollowId,    sizeof(b->mFollowId)    );
			byteCount += s->Write(&b->mFollowRange, sizeof(b->mFollowRange) );

			break;
		}
	case DSTBALL_FORMATION:
		{

			byteCount += s->Write(&b->mFollowId,    sizeof(b->mFollowId)    );
			byteCount += s->Write(&b->mFollowRange, sizeof(b->mFollowRange) );
			byteCount += s->Write(&effectStamp, sizeof(effectStamp) );

			break;
		}
	case DSTBALL_MISSILE:
		{

			byteCount += s->Write(&b->mFollowId,    sizeof(b->mFollowId)    );
			byteCount += s->Write(&b->mFollowRange, sizeof(b->mFollowRange) );
			byteCount += s->Write(&b->mOwnerId,     sizeof(b->mOwnerId)     );
			byteCount += s->Write(&effectStamp, sizeof(effectStamp) );
			byteCount += s->Write(&b->mGoto.x,      sizeof(b->mGoto)        );

			break;
		}
	case DSTBALL_ORBIT:
		{

			byteCount += s->Write(&b->mFollowId,    sizeof(b->mFollowId)    );
			byteCount += s->Write(&b->mFollowRange, sizeof(b->mFollowRange) );
			
			break;
		}
	case DSTBALL_GOTO:
		{
			byteCount += s->Write(&b->mGoto.x,    sizeof(b->mGoto)      );

			break;
		}
	case DSTBALL_WARP:
		{
			byteCount += s->Write(&b->mGoto.x,      sizeof(b->mGoto)        ); // Warp destination point
			byteCount += s->Write(&effectStamp, sizeof(effectStamp) );
			byteCount += s->Write(&b->mLastCollision, sizeof(b->mLastCollision) ); // Total length of warp
			byteCount += s->Write(&b->mFollowId,    sizeof(b->mFollowId)    ); // Minimum range (as double)
			byteCount += s->Write(&b->mOwnerId,     sizeof(b->mOwnerId)     ); // Warp factor (speed multiplier)

			break;
		}
	case DSTBALL_MUSHROOM:
		{
			double span = b->mGoto.x;

			byteCount += s->Write(&b->mFollowRange, sizeof(b->mFollowRange) );
			byteCount += s->Write(&span           , sizeof(span)               );
			byteCount += s->Write(&effectStamp, sizeof(effectStamp) );
			byteCount += s->Write(&b->mOwnerId,     sizeof(b->mOwnerId)     );

			break;
		}
	case DSTBALL_TROLL:
		{
			byteCount += s->Write(&effectStamp, sizeof(effectStamp) );

			break;
		}
	case DSTBALL_STOP:
	case DSTBALL_FIELD:
	case DSTBALL_RIGID:
		{
			// Nohting to do here

			break;
		}
	default:
		{
			CCP_LOGERR_CH( s_chPThunk,"WriteBallToStream: Unknown ball mode %d in dump", mode);
			return;
			break;
		}
	}


	// Write out any miniballs
	if(miniBallCnt)
		byteCount += s->Write(&miniBallCnt, sizeof(miniBallCnt));

	for(int i = 0; i < miniBallCnt; i++)
	{
		MiniBall *mb = (MiniBall *)(void *)(b->mMiniBalls.GetAt(i));
		byteCount += s->Write(&mb->mPos.x, sizeof(mb->mPos));
		byteCount += s->Write(&mb->mRadius, sizeof(mb->mRadius));
	}

	// Minicapsules
	if(miniCapsuleCnt)
		byteCount += s->Write(&miniCapsuleCnt, sizeof(miniCapsuleCnt));

	for(int i =0; i < miniCapsuleCnt; i++)
	{
		MiniCapsule *mc = (MiniCapsule *)(void *)(b->mMiniCapsules.GetAt(i));
		byteCount += s->Write(&mc->mHemisphereA, sizeof(mc->mHemisphereA));
		byteCount += s->Write(&mc->mHemisphereB, sizeof(mc->mHemisphereB));
		byteCount += s->Write(&mc->mRadius, sizeof(mc->mRadius));
	}

	// Miniboxes
	if(miniBoxCnt)
		byteCount += s->Write(&miniBoxCnt, sizeof(miniBoxCnt));

	for(int i = 0; i < miniBoxCnt; i++)
	{
		MiniBox *mc = (MiniBox *)(void *)(b->mMiniBoxes.GetAt(i));
		byteCount += s->Write(&mc->mCorner, sizeof(mc->mCorner));
		byteCount += s->Write(&mc->mLocalX, sizeof(mc->mLocalX));
		byteCount += s->Write(&mc->mLocalY, sizeof(mc->mLocalY));
		byteCount += s->Write(&mc->mLocalZ, sizeof(mc->mLocalZ));
	}
}

PyObject* Ballpark::PyGetCurrentEgoPos(
	PyObject* args
	)
{
	Vector3d pos;

	if (!PyArg_ParseTuple(args, ""))
		return NULL;

	GetReferencePoint(&pos, BeOS->GetInfo()->mSimTime);

	PyObject *ret = PyTuple_New(3);
	PyTuple_SET_ITEM(ret, 0, PyFloat_FromDouble( pos.x ));
	PyTuple_SET_ITEM(ret, 1, PyFloat_FromDouble( pos.y ));
	PyTuple_SET_ITEM(ret, 2, PyFloat_FromDouble( pos.z ));

	return ret;
}

PyObject* Ballpark::PyGetBallBoxKeys(
	PyObject* args
	)
{
		ID srcID;
	
		if (!PyArg_ParseTuple(args, "L",
			&srcID
			))
			return NULL;
	
		PyObject* boxList = PyList_New(0);

		Ball *ball = mBalls[srcID];
		if (ball)
		{
			SortedSetOfBoxes::iterator it_boxes;
			Box* box;
			for (it_boxes = ball->mBoxes.begin(); it_boxes != ball->mBoxes.end(); ++it_boxes)
			{
				box = *it_boxes;

				PyObject *tuple = PyTuple_New(4);
				PyTuple_SET_ITEM(tuple, 0, PyLong_FromLong(box->mLevel));
				PyTuple_SET_ITEM(tuple, 1, PyLong_FromLongLong(box->mKey.ix));
				PyTuple_SET_ITEM(tuple, 2, PyLong_FromLongLong(box->mKey.iy));
				PyTuple_SET_ITEM(tuple, 3, PyLong_FromLongLong(box->mKey.iz));
				PyList_Append(boxList, tuple);
				Py_DECREF(tuple);
			}
		}
		return boxList;
}

PyObject* Ballpark::PyGetBoxChildren(
	PyObject* args
	)
{
	int level;
	int64_t ix, iy, iz;

	if (!PyArg_ParseTuple(args, "iLLL",
		&level,
		&ix,
		&iy,
		&iz
		))
		return NULL;

	PyObject* childBoxList = PyList_New(0);
	PyObject* childBallList = PyList_New(0);
	PyObject* childStaticCollidableList = PyList_New(0);

	Box* parentBox = mPartition->CheckBox(ix, iy, iz, level);
	if (parentBox)
	{
		SortedSetOfBoxes::iterator it_boxes;
		Box* childBox;
		for (it_boxes = parentBox->mChildren.begin(); it_boxes != parentBox->mChildren.end(); ++it_boxes)
		{
			childBox = *it_boxes;

			PyObject *tuple = PyTuple_New(4);
			PyTuple_SET_ITEM(tuple, 0, PyLong_FromLong(childBox->mLevel));
			PyTuple_SET_ITEM(tuple, 1, PyLong_FromLongLong(childBox->mKey.ix));
			PyTuple_SET_ITEM(tuple, 2, PyLong_FromLongLong(childBox->mKey.iy));
			PyTuple_SET_ITEM(tuple, 3, PyLong_FromLongLong(childBox->mKey.iz));
			PyList_Append(childBoxList, tuple);
			Py_DECREF(tuple);
		}

		SortedSetOfBalls::iterator it_balls;
		Ball* ball;
		for (it_balls = parentBox->balls.begin(); it_balls != parentBox->balls.end(); ++it_balls)
		{
			ball = *it_balls;

			PyObject *val = PyLong_FromLongLong(ball->mId);
			PyList_Append(childBallList, val);
			Py_DECREF(val);
		}

		SortedSetOfStaticCollidables::iterator it_staticCollidables;
		StaticCollidable* staticCollidable;
		for (it_staticCollidables = parentBox->mStaticCollidables.begin(); it_staticCollidables != parentBox->mStaticCollidables.end(); ++it_staticCollidables)
		{
			staticCollidable = *it_staticCollidables;

			PyObject *tuple = PyTuple_New(2);
			PyTuple_SET_ITEM(tuple, 0, PyLong_FromLongLong(staticCollidable->mId));
			PyTuple_SET_ITEM(tuple, 1, PyLong_FromLongLong(staticCollidable->mParentBallId));
			PyList_Append(childStaticCollidableList, tuple);
			Py_DECREF(tuple);
		}
	}

	PyObject *ret = PyTuple_New(3);
	PyTuple_SET_ITEM(ret, 0, childBoxList);
	PyTuple_SET_ITEM(ret, 1, childBallList);
	PyTuple_SET_ITEM(ret, 2, childStaticCollidableList);
	return ret;
}

PyObject* Ballpark::PyGetBoxKey(
    PyObject* args
    )
{
    Vector3d pos;
    int level;

    if (!PyArg_ParseTuple(args, "iddd",
        &level,
        &pos.x,
        &pos.y,
        &pos.z
        ))
        return NULL;

    if (level < 0 || level >= mPartition->mNumberOfLevels)
    {
        PyErr_SetString(PyExc_TypeError, "Illegal level");
        return 0;
    }

    int64_t ix, iy, iz;
    mPartition->GetBoxKey(pos, level, ix, iy, iz);

    PyObject *ret = PyTuple_New(3);
    PyTuple_SET_ITEM(ret, 0, PyLong_FromLongLong(ix));
    PyTuple_SET_ITEM(ret, 1, PyLong_FromLongLong(iy));
    PyTuple_SET_ITEM(ret, 2, PyLong_FromLongLong(iz));

    return ret;
}
