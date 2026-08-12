// Copyright © 2014 CCP ehf.

#include "StdAfx.h"

#include "Ball.h"
#include "Ballpark.h"

const Be::ClassInfo* Ball::ExposeToBlue()
{
	
	EXPOSURE_BEGIN(Ball, "")
		MAP_INTERFACE(INotify)
		MAP_INTERFACE(IBall)
		MAP_INTERFACE(Ball)

		/////////////////////////////////////////////////////////////////////////////
		//               name of ball

		MAP_ATTRIBUTE
		(
			"id",
			mId,
			"id of ball",
			Be::READ| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               mass
		MAP_ATTRIBUTE
		(
			"mass",
			mMass,
			"Mass of ball (kg)",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               radius
		MAP_ATTRIBUTE
		(
			"radius",
			mRadius,
			"Radius of ball (m)",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               maxVelocity
		MAP_ATTRIBUTE
		(
			"maxVelocity",
			mMaxVel,
			"Maximum velocity of ball (m/s)",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               maxAngularVelocity
		MAP_ATTRIBUTE(
			"maxAngularVelocity",
			mMaxAngVel,
			"Maximum angular velocity of ball (rad/s)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//               isFree
		MAP_ATTRIBUTE
		(
			"isFree",
			isFree,
			"True if ball is free to move",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               isGlobal
		MAP_ATTRIBUTE
		(
			"isGlobal",
			isGlobal,
			"True if ball should be visible from all",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               isMassive
		MAP_ATTRIBUTE
		(
			"isMassive",
			isMassive,
			"True if ball is solid",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               isInteractive
		MAP_ATTRIBUTE
		(
			"isInteractive",
			isInteractive,
			"True if ball is interactive",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               isCloaked
		MAP_ATTRIBUTE
		(
			"isCloaked",
			isCloaked,
			"True if ball is cloaked",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               isMoribund
		MAP_ATTRIBUTE
		(
			"isMoribund",
			isMoribund,
			"True if ball is moribund",
			Be::READ
		)

		/////////////////////////////////////////////////////////////////////////////
		//               mHarmonic
		MAP_ATTRIBUTE
		(
			"harmonic",
			mHarmonic,
			"Shield harmonic value of ball",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               mCorporationID
		MAP_ATTRIBUTE
		(
			"corporationID",
			mCorporationID,
			"corporation ID of ball",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               mAllianceID
		MAP_ATTRIBUTE
		(
			"allianceID",
			mAllianceID,
			"alliance ID of ball",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//              x of ball
		MAP_ATTRIBUTE
		(
			"x",
			mNewPos.x,
			"x position of ball (m)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//              y of ball
		MAP_ATTRIBUTE
		(
			"y",
			mNewPos.y,
			"y position of ball (m)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)


		/////////////////////////////////////////////////////////////////////////////
		//              z of ball
		MAP_ATTRIBUTE
		(
			"z",
			mNewPos.z,
			"z position of ball (m)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)


		/////////////////////////////////////////////////////////////////////////////
		//              vx of ball
		MAP_ATTRIBUTE
		(
			"vx",
			mNewVel.x,
			"x velocity of ball (m/s)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)


		/////////////////////////////////////////////////////////////////////////////
		//              vy of ball
		MAP_ATTRIBUTE
		(
			"vy",
			mNewVel.y,
			"y velocity of ball (m/s)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)


		/////////////////////////////////////////////////////////////////////////////
		//              vz of ball
		MAP_ATTRIBUTE
		(
			"vz",
			mNewVel.z,
			"z velocity of ball (m/s)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//              wx of ball
		MAP_ATTRIBUTE(
			"wx",
			mNewAngVel.x,
			"x angular velocity of ball (rad/s)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//              wy of ball
		MAP_ATTRIBUTE(
			"wy",
			mNewAngVel.y,
			"y angular velocity of ball (rad/s)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//              wz of ball
		MAP_ATTRIBUTE(
			"wz",
			mNewAngVel.z,
			"z angular velocity of ball (rad/s)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//              rx of ball
		MAP_ATTRIBUTE(
			"rx",
			mNewRot.x,
			"x component of rotation quaternion",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//              ry of ball
		MAP_ATTRIBUTE(
			"ry",
			mNewRot.y,
			"y component of rotation quaternion",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//              rz of ball
		MAP_ATTRIBUTE(
			"rz",
			mNewRot.z,
			"z component of rotation quaternion",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//              rw of ball
		MAP_ATTRIBUTE(
			"rw",
			mNewRot.w,
			"w component of rotation quaternion",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//              yaw of ball
		MAP_ATTRIBUTE
		(
			"yaw",
			mNewYaw,
			"Yaw of ball",
			Be::READ
		)

		/////////////////////////////////////////////////////////////////////////////
		//              pitch of ball
		MAP_ATTRIBUTE
		(
			"pitch",
			mNewPitch,
			"Pitch of ball",
			Be::READ
		)

		/////////////////////////////////////////////////////////////////////////////
		//              roll of ball
		MAP_ATTRIBUTE
		(
			"roll",
			mNewRoll,
			"Roll of ball",
			Be::READ
		)

		/////////////////////////////////////////////////////////////////////////////
		//               Agility

		MAP_ATTRIBUTE
		(
			"Agility",
			mAgility,
			"Agility modifier of ship",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               Rotational agility

		MAP_ATTRIBUTE(
			"RotationalAgility",
			mRotAgility,
			"Rotational agility modifier of ship",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST )

		/////////////////////////////////////////////////////////////////////////////
		//               speedFraction
		MAP_ATTRIBUTE
		(
			"speedFraction",
			mSpeedFraction,
			"Cruise velocity as a fraction of maximum velocity",
			Be::READWRITE | Be::NOTIFY| Be::PERSIST
		)
		/////////////////////////////////////////////////////////////////////////////
		//               mode
		MAP_ATTRIBUTE_WITH_CHOOSER
		(
			"mode",
			mMode,
			"Dynamic mode of ball",
			Be::READ| Be::ENUM| Be::PERSIST,
			DstBallMode
		)

		/////////////////////////////////////////////////////////////////////////////
		//              x of goto
		MAP_ATTRIBUTE
		(
			"gotoX",
			mGoto.x,
			"x destination of ball (m)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//              y of goto
		MAP_ATTRIBUTE
		(
			"gotoY",
			mGoto.y,
			"y destination of ball (m)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//              z of goto
		MAP_ATTRIBUTE
		(
			"gotoZ",
			mGoto.z,
			"z destination of ball (m)",
			Be::READWRITE | Be::NOTIFY | Be::PERSIST
		)


		MAP_ATTRIBUTE
		(
			"followId",
			mFollowId,
			"id of ball",
			Be::READ| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               followRange
		MAP_ATTRIBUTE
		(
			"followRange",
			mFollowRange,
			"range of follow",
			Be::READ| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//              ownerId


		MAP_ATTRIBUTE
		(
			"ownerId",
			mOwnerId,
			"id of ball",
			Be::READWRITE| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               effectStamp
		MAP_ATTRIBUTE
		(
			"effectStamp",
			mEffectStamp,
			"time stamp of effect",
			Be::READ| Be::PERSIST
		)

		/////////////////////////////////////////////////////////////////////////////
		//               new bubbleId
		MAP_ATTRIBUTE
		(
			"newBubbleId",
			mNewBubble,
			"id of bubble ball currently belongs to",
			Be::READ
		)
		/////////////////////////////////////////////////////////////////////////////
		//               old bubbleId
		MAP_ATTRIBUTE
		(
			"oldBubbleId",
			mOldBubble,
			"id of bubble ball used to belong to",
			Be::READ
		)
		/////////////////////////////////////////////////////////////////////////////
		//               formation ID
		MAP_ATTRIBUTE
		(
			"formationID",
			mFormationID,
			"id of current formation",
			Be::READ
		)
		////////////////////////////////////////////////////////////////////////////
		//               keys
		MAP_ATTRIBUTE
		(
			"miniBalls",
			mMiniBalls,
			"na",
			Be::READ | Be::PERSIST
		)

		MAP_ATTRIBUTE
		(
			"miniCapsules",
			mMiniCapsules,
			"na",
			Be::READ | Be::PERSIST
		)

		MAP_ATTRIBUTE
		(
			"miniBoxes",
			mMiniBoxes,
			"na",
			Be::READ | Be::PERSIST
		)


		////////////////////////////////////////////////////////////////////////////
		//               ballpark
		MAP_ATTRIBUTE
		(
			"ballpark",
			mPark,
			"The ballpark",
			Be::READ
		)

		////////////////////////////////////////////////////////////////////////////
		//               __init__
		MAP_METHOD_AS_METHOD
		(
			"__init__",
			Py__init__,
			"Constructor arguments"
		)

		MAP_METHOD_AS_METHOD
		(
			"AddMiniBall",
			PyAddMiniBall,
			"Adds a miniball to the miniballs list"
		)

		MAP_METHOD_AS_METHOD
		(
			"AddMiniCapsule",
			PyAddMiniCapsule,
			"Adds a minicapsule to the minicapsule list"
		)

		MAP_METHOD_AS_METHOD
		(
			"AddMiniBox",
			PyAddMiniBox,
			"Adds a minibox to the minibox list"
		)

		MAP_METHOD_AS_METHOD
		(
			"GetRotatedVector",
			PyGetRotatedVector,
			"Returns the given vector rotated with the balls current rotation"
		)
		MAP_METHOD_AS_METHOD
		(
			"AddProximitySensor",
			PyAddProximitySensor,
			"Adds a proximity sensor to the given ball"
		)
		MAP_METHOD_AS_METHOD
		(
			"ReserveFormationSlot",
			PyReserveFormationSlot,
			"Reserves a formation slot"
		)
		MAP_METHOD_AS_METHOD
		(
			"FreeFormationSlot",
			PyFreeFormationSlot,
			"Frees a formation slot"
		)
	EXPOSURE_END()
}


const Be::ClassInfo* ClientBall::ExposeToBlue()
{

	EXPOSURE_BEGIN(ClientBall, "")
		MAP_INTERFACE(ITriQuaternionFunction)
		MAP_INTERFACE(ITriVectorFunction)
		MAP_INTERFACE(IEveReferencePoint)
		MAP_INTERFACE(ClientBall)


		MAP_ATTRIBUTE
		(
			"showBoxes",
			mShowBoxes,
			"True to display partition boxes around ball",
			Be::READWRITE
		)

		MAP_PROPERTY_READONLY
		(
			"centerDist",
			GetCenterDistance,
			"center-center distance from ego"
		)

		MAP_PROPERTY_READONLY
		(
			"surfaceDist",
			GetSurfaceDistance,
			"surface-surface distance from ego"
		)

		MAP_ATTRIBUTE
		(
		    "maxAngle",
		    mMaxAngle,
		    "max impact angle",
		    Be::READWRITE
		)
		
		MAP_ATTRIBUTE
		(
		    "maxAngularVelocity",
		    mMaxAngularVelocity,
		    "max angular velocity",
		    Be::READWRITE
		)

		MAP_ATTRIBUTE
		(
		    "speedModifier",
		    mSpeedModifier,
		    "How much faster does the ship move when it is hit versus when it is recovering",
		    Be::READWRITE
		)

		MAP_ATTRIBUTE
		(
		    "minimumAngularVelocity",
		    mMinimumAngularVelocity,
		    "Minimum angular velocity, impacts producing angular velocity less than this will not be processed",
		    Be::READWRITE
		)

		MAP_METHOD_AS_METHOD
		(
			"GetPartitionBoxes",
			PyGetPartitionBoxes,
			"Testing python method call."
		)

		MAP_METHOD_AND_WRAP
		(
			"ApplyImpulsiveForceAtPosition",
			ApplyImpulsiveForceAtPosition,
			"Apply impulsive force at a position"
		)

	EXPOSURE_CHAINTO(Ball)
}
