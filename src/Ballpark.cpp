// Copyright © 2014 CCP ehf.

#include "StdAfx.h"
#include <math.h>
#ifdef _MSC_VER
#include <fpieee.h>
#include <excpt.h>
#endif
#include <float.h>

#include "Ballpark.h"
#include "Ball.h"
#include "Box.h"
#include "Quaternion.h"
#include "Collision.h"
#include "Settings.h"

#include <ITaskletTimer.h>

#include <list>
#include <algorithm>
#include <array>

PyObject* Timer_ResolveBubbleConflicts;
PyObject* Timer_HandleProximities;
PyObject* Timer_InsertBallInBoxes;
PyObject* Timer_Gradient;
PyObject* Timer_CheckVisibility;
PyObject* Timer_CopyBubbles;
PyObject* Timer_AdditionsAndDeletions;
PyObject* Timer_GetBallIdsInRange;
PyObject* Timer_WarpDistance;
PyObject* Timer_CalculateYawPitchRoll;
PyObject* Timer_WriteBallsToStream;

ITaskletTimer *TheTimer = PyOS->GetTaskletTimer();
#define TASKLET(A) AutoTasklet _at(TheTimer, A)

PyObject *Plus;
PyObject *Zero;
PyObject *Minus;


static CcpLogChannel_t s_chPark = CCP_LOG_DEFINE_CHANNEL( "BallPark" );
static CcpLogChannel_t s_chPStat = CCP_LOG_DEFINE_CHANNEL( "ParkStats" );

BLUE_DEFINE_INTERFACE( IEveBallpark );

const char* TICK_EVOLVE = "Destiny::Tick";
#define SIGNUM(X) (X>=0.0?1.0:-1.0)
#define SIGNUMF( X ) ( X >= 0.0f ? 1.0f : -1.0f )
#define ABS(X) ((X)<0.0f?-(X):(X))

static constexpr double _PI_ = 3.141592653589793238462643383279502884197;

PyObject * Ballpark::s_ballNotInParkCallback = NULL;
static int ballparkCounter = 0;

// These constants describe how warpFactor maps in to top-speed, rate of acceleration, and rate of deceleration
// See warpSpeedToAUPerSecond in appConst.py
double WARP_FACTOR_TO_AU_PER_SECOND = 0.001; // mAU/s -> AU/s
double WARP_FACTOR_TO_ACCELERATION = 1.0 / 1000; // Higher value means shorter 0-to-max-warp time
double WARP_FACTOR_TO_DECELERATION = 1.0 / 3000; // Higher value means shorter max-warp-to-0 time
// The acceleration:deceleration ratio approx follows the ratio of time spent acceleration vs time spent decelerating
// ('Approx' is because the scaling isn't totally linear, but is reasonably close for the scales we use)

// These ball movement modes use mFollowId and mFollowPtr
extern const std::array<int,4> followModes = {DSTBALL_FOLLOW, DSTBALL_ORBIT, DSTBALL_MISSILE, DSTBALL_FORMATION};
// Used in ORBIT move-mode - Controls how quickly the rotational axis itself will rotate (rad/s)
double ORBITAL_PRECESSION = 0.001;

#ifndef _MSC_VER
namespace
{
    inline float _finite( double x )
    {
        return isfinite( x );
    }
}
#endif

#pragma region Constructor and destructor
//---------------------------------------------------------------------------------------
// Ballpark Constructor
//---------------------------------------------------------------------------------------
Ballpark::Ballpark(IRoot* lockobj) :
    mTickInterval(1000),
    mEgo(0),
    mProbe(0),
    mSolarsystemID(-1),
    mLastEgo(0),
    mLastEgoBall(0),
    mTime(0),
    mLastReferenceTime(-1),
    mFirstTime(true),
    mCurrentTime(0),
    isMaster(false),
    mPartition(0),
    mLagDamping(0.1),
    mFriction(1000000.0),
    mWarpSpeed(2.0),
    mBubbleId(0),
    bubbles(0),
    mBubbleSubscriptions(0),
    mSomeWeirdHackToFixSomething(1.0f),
    inEvolve(false),
    mPara1(0.0),
    mPara2(0.0),
    mInt1(0),
    mInt2(0),
    mLocalCnt(-1),
    mLocalHiCnt(DSTLOCALBALLS),
    mMoribundBallRemovalCount(7),
    mMoribundBallRemovalBuffer(2),
	mRollSpeedAcceleration(-3.0),
	mRollSpeedDecay(-0.9),
	mRollAcceleration(1.0),
	mRollDecay(-0.5)
{
    dt = mTickInterval * 0.001;
    mHaveTicks = false;
    mPartition = new Partition();
    mBubbleSubscriptions = PyDict_New();
    bubbleInteractives = PyDict_New();
    bubbleKeepAlives = PyDict_New();
	bubbleKeepAliveBalls = PySet_New(NULL);

    if (ballparkCounter == 0)
    {   // these strings are really static and we make sure that they are only allocated one time
        Timer_ResolveBubbleConflicts = PyUnicode_FromString("Destiny::ResolveBubbleConflicts");
        Timer_HandleProximities = PyUnicode_FromString("Destiny::HandleProximities");
        Timer_InsertBallInBoxes = PyUnicode_FromString("Destiny::InsertBallInBoxes");
        Timer_Gradient = PyUnicode_FromString("Destiny::Gradient");
        Timer_CheckVisibility = PyUnicode_FromString("Destiny::CheckVisibility");
        Timer_CopyBubbles = PyUnicode_FromString("Destiny::CopyBubbles");
        Timer_AdditionsAndDeletions = PyUnicode_FromString("Destiny::AdditionsAndDeletions");
        Timer_GetBallIdsInRange = PyUnicode_FromString("Destiny::GetBallIdsInRange");
        Timer_WarpDistance = PyUnicode_FromString("Destiny::WarpDistance");
        Timer_CalculateYawPitchRoll = PyUnicode_FromString("Destiny::CalculateYawPitchRoll");
        Timer_WriteBallsToStream = PyUnicode_FromString("Destiny::WriteBallsToStream");
        Plus  = PyLong_FromLong(1);
        Zero  = PyLong_FromLong(0);
        Minus = PyLong_FromLong(-1);
    }

    bubbles = PyDict_New();

    ballparkCounter++;
    CCP_LOG_CH( s_chPark, "Ballpark:%d created with time-step %d msec. Total of %d\n",this, mTickInterval,ballparkCounter);
}


//---------------------------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------------------------
Ballpark::~Ballpark()
{
    ballparkCounter--;
    CCP_LOG_CH( s_chPark, "Ballpark:%d destroyed. Total of %d remaining\n",this,ballparkCounter);

    if (ballparkCounter == 0)
    {   // these strings are really static and we have to make sure they are only
        // deallocated when ALL ballparks are gone.
        Py_DECREF(Timer_ResolveBubbleConflicts);
        Py_DECREF(Timer_HandleProximities);
        Py_DECREF(Timer_InsertBallInBoxes);
        Py_DECREF(Timer_Gradient);
        Py_DECREF(Timer_CheckVisibility);
        Py_DECREF(Timer_CopyBubbles);
        Py_DECREF(Timer_AdditionsAndDeletions);
        Py_DECREF(Timer_GetBallIdsInRange);
        Py_DECREF(Timer_WarpDistance);
        Py_DECREF(Timer_CalculateYawPitchRoll);
        Py_DECREF(Plus);
        Py_DECREF(Zero);
        Py_DECREF(Minus);
    }

    ClearAll();
    if(mPartition)
    {
        delete mPartition;
        mPartition = 0;
    }

    if(mBubbleSubscriptions)
        Py_DECREF(mBubbleSubscriptions);

    if(bubbleInteractives)
        Py_DECREF(bubbleInteractives);

    if(bubbleKeepAlives)
        Py_DECREF(bubbleKeepAlives);

	if(bubbleKeepAliveBalls)
        Py_DECREF(bubbleKeepAliveBalls);

    Py_XDECREF( Ballpark::s_ballNotInParkCallback );
    Ballpark::s_ballNotInParkCallback = NULL;

    // Always need to unregister from ticks if we get destroyed
    Pause();
}
#pragma endregion


#pragma region Ballpark ticking and evolution
#pragma region Whole-park tick
//---------------------------------------------------------------------------------------
// IBlueEvents::OnTick
//
// Ballpark requests timer events with the specified tick interval parameter
// which is also used as the time step for the simulation.
//
// The simulation is run for as many time steps as has elapsed since the last
// tick event.
//
//---------------------------------------------------------------------------------------
void Ballpark::OnTick(
    Be::Time realTime,
    Be::Time simTime,
    void* cookie
    )
{
	BeTimer timer = BeTimer();

    // This is the delta since I was last called (the first time round it equals timestamp)
    Be::Time lastDelta = simTime - mTime;

    // Remove Moribund balls every frame
    BringOutTheDeads();


    // If there hasn't yet passed a whole tick interval, then return
    if(lastDelta < mTickInterval*10000)
    {
        return;
    }

    // lastDelta is at least one whole tick interval


    // just to be sure
    dt = mTickInterval*0.001;

    if(isMaster)
    {
        if(lastDelta/(mTickInterval*10000) > 5)
        {
            // There are more than 5 ticks in this batch
            // Rather than calculate all of them, let's reset our clock
            if(mTime != 0)
            {  // Don't spam if this is our first tick, it's expected
                CCP_LOGWARN_CH( s_chPark,"%d time intervals in batch: resynching",lastDelta/(mTickInterval*10000));
            }
            lastDelta = mTickInterval*10000;
        }
    }

    if(mFirstTime)
    {
        // If this is the first time, lastDelta is humbug..Just evolve once and get it over with

        // The master copy keeps track of bubbles
        //if(isMaster)
        //    InitializeBubbles();

        timer.Reset();
        Evolve(simTime);

        lastDelta  = 0;
        mFirstTime = false;
    }
    else
    {
        // Let's evolve as many times as there are whole tick intervals in lastDelta
        int intervals = int(lastDelta/(mTickInterval*10000));
        //CCP_LOG_CH( s_chPStat,"[%I64d] %d ticks in batch", mSolarsystemID, intervals);

        for(int i=0 ; i < intervals ; i++)
        {
            /*
            if(!mBubbleConflicts.empty())
            {
                //CCP_LOG_CH( s_chPark,"Total of %d conflicts",mBubbleConflicts.size());
                ResolveBubbleConflicts();
            }
            */

            if (!PyOS->SendEvent(
                (IEveBallpark*)this, "Destiny::DoPreTick",
                "DoPreTick", NULL, "(i)", mCurrentTime
                ))
            {
                PyOS->PyError();
            }

            timer.Reset();
            Evolve(simTime - lastDelta);
            if(mProbe != 0)
            {
                Ball *b = mBalls[mProbe];
                if(b)
                {
                    CCP_LOGWARN_CH( s_chPark," [%d]pos: -> ( %f , %f, %f ) Time Factor: %.16f",mCurrentTime, b->mNewPos.x, b->mNewPos.y, b->mNewPos.z, b->mTimeFactor);
                    CCP_LOGWARN_CH( s_chPark," [%d]vel: -> ( %f , %f, %f ) Speed: %f ",mCurrentTime, b->mNewVel.x, b->mNewVel.y, b->mNewVel.z, b->mNewVel.Length()/b->mMaxVel);
                    CCP_LOGWARN_CH( s_chPark," [%d] -> [ %s, %s, %s, %s ]", mCurrentTime,(b->isFree?"Free":"Fixed"),(b->isMassive?"Massive":"Ligth"),(b->isCloaked?"Cloaked":"Visible"),(b->isInteractive?"Interactive":"Inert"));
                    CCP_LOGWARN_CH( s_chPark," [%d] -> [ Mode: %d, effectStamp: %d ]", mCurrentTime, b->mMode, b->mEffectStamp);

                }
            }

            //CCP_LOG_CH( s_chPStat,"[%I64d] Total Evolve[%d]: %f", mSolarsystemID, mCurrentTime, timer.GetTime()/10000.0);

            if (!PyOS->SendEvent(
                (IEveBallpark*)this, "Destiny::DoPostTick",
                "DoPostTick", NULL, "(i)", mCurrentTime
                ))
            {
                PyOS->PyError();
            }

            lastDelta -= mTickInterval*10000;
        }
    }

    // Now lastDelta contains the 'dust' that was left.
    // Flag the last time I was called as the time minus the 'dust'..
    mTime = simTime - lastDelta;
}

void Ballpark::CalculateIterativeCollisionResponses()
{
	// Use a special loop to calculate collision responses iteratively
	// Re-initialize the previous responses.
	for (auto &pair : mCollisionLocations)
		pair.second.Initialize(pair.second.m_ball, dt);

	// We keep the vectors around so memory is not constantly being allocated for them
	for (auto &pair : mPotentialCollisionBalls)
		pair.second.clear();
	for (auto &pair : mPotentialCollidables)
		pair.second.clear();

	// Keep track of balls that no longer require their responses calculated to reduce load on server
	// Store both Id and pointer to save on a lookup in freeballs.  This has to be ordered
	DictOfFreeBalls currentBalls, nextBalls;

	// First loop is over free balls, all others over currentBalls
	auto ballIt = mFreeBalls.begin();
	auto ballItEnd = mFreeBalls.end();

	size_t collisionIteration = 0;
	Ball *ball = nullptr;
	do
	{
		for (; ballIt != ballItEnd; ++ballIt)
		{
			ball = ballIt->second;

			//Ignore dead balls
			if (ball->isMoribund)
				continue;

			if (ball->mInDeadBubble)
				continue;

			// Ignore non-massive balls
			if (!ball->isMassive)
				continue;

			// Calculate responses for the ball
			bool found = CalculateCollisionResponse(ball, collisionIteration);

			// Look for a new collision response and add all free balls within the collision zone to the next balls
			if (found)
			{
				// Find the balls, and add the free balls.  This does not include current ball
				for (auto* neighbor : mPotentialCollisionBalls[ball->mId])
				{
					if (neighbor->isFree)
						nextBalls[neighbor->mId] = neighbor;
				}
				nextBalls[ball->mId] = ball;
			}
		}

		if( nextBalls.empty() )
			break;

		++collisionIteration;

		// Performance cap, reducing this allows agile ships to fly through thin collision regions.
		if( collisionIteration >= g_collisionMaxIterations )
			break;

		// Next balls become current and we empty next balls
		std::swap( currentBalls, nextBalls );
		nextBalls.clear();
		ballIt = currentBalls.begin();
		ballItEnd = currentBalls.end();
	} while( true );
}


//---------------------------------------------------------------------------------------
// Check if dynamical orientation is disabled for this ball.
//---------------------------------------------------------------------------------------
bool DynamicalOrientationDisabledForBall( Ball* ball )
{
	if( g_disableDynamicalOrientationForMissiles )
	{
		return ball->mMode == DSTBALL_MISSILE;
	}
	return false;
}

//---------------------------------------------------------------------------------------
// Evolve advances the whole simulation by one timestep
//---------------------------------------------------------------------------------------
void Ballpark::Evolve(Be::Time timestamp)
{
	CCP_STATS_ZONE( __FUNCTION__ );

	Ball *ball;
    inEvolve = true;

    DictOfFreeBalls::iterator sit;
    for(sit = mFreeBalls.begin(); sit != mFreeBalls.end(); ++sit)
    {
        ball = sit->second;

        if(ball->isMoribund)
            continue; // don't handle dead balls

        // Cache this call
        if (InDeadBubble(ball))
        {
            ball->mInDeadBubble = true;
            continue;
        }

        EvolveBehaviorForBall(ball);
    }


    //CCP_LOGNOTICE_CH( s_chPStat,"[%I64d] Accelerations calculations: %f", mSolarsystemID, timer.GetTime()/10000.0);
    //timer.Reset();

    // Now that we have accumulated accelerations due to intrisic ball behavior
    // Calculate the possible collision responses
	if( g_useIterativeCollision )
	{
		CalculateIterativeCollisionResponses();
	}

    for(sit = mFreeBalls.begin(); sit != mFreeBalls.end(); ++sit)
    {
        ball = sit->second;

        if(ball->isMoribund)
            continue; // don't handle dead balls

        if(ball->mInDeadBubble)
            continue;

		if( !g_useIterativeCollision && ball->isMassive )
		{
			// Get the gradient term, with contribution from nearby objects
			ball->mLastCollision = -1.0;
			Gradient(ball);
		}

        Vector3d p,v;
        p = ball->mNewPos;
        v = ball->mNewVel;

        if(ball->IsWarping())
        {
            WarpDistance(ball, p, v, (mCurrentTime - ball->mEffectStamp)*dt, false);
            if(ball->mMode == DSTBALL_STOP)
            {   // Just got dropped out of warp, now the warp result becomes the new oldPos
                // and we integrate one step ahead so that interpolation can continue from the
                // point we dropped out of warp.
                ball->mNewPos = p; // (old and new get swapped at the end)
                ball->mNewVel = v;
                Integrate(p,v, ball->mLastG + ball->mLastC, ball->mMass * ball->mAgility, mFriction, ball->mTimeFactor, dt);

                if (ball->mNewVel.LengthSq() < 0.1)
                {   // Under extreme cases, it is possible for a ball to end its warp in a new bubble and hit 0 velocity in a single tick.
                    // (Typically capsules with very high warp speeds and very good agility)
                    // If this happens, then it won't get an updated box insertion until it starts moving again.
                    // Mitigate this by forcing a box update.
                    InsertBallInBoxes(ball);
                }
            }
        }
        else
        {
			CalculateBallPositionVelocity( ball, dt, p, v );
        }
		if( g_useDynamicalOrientation && !DynamicalOrientationDisabledForBall(ball) )
		{
			//Update the orientation
			Quaternion rotation = ball->mNewRot;
			Vector3 angVel = ball->mNewAngVel;

			if(g_useIterativeCollision)
			{
				CalculateBallRotationVelocity( ball, 1.0, rotation, angVel );
			}
			else
			{
				const float tau = ( float( ball->mMass ) * ball->mAgility * ball->mRotAgility ) / float( mFriction );
				ApplyTorque( rotation, angVel, ball->mTorque, ball->mRoll, tau, 1.0f );
			}
			ball->mOldRot = rotation;
			ball->mOldAngVel = angVel;
		}
        ball->mOldPos = p;
        ball->mOldVel = v;
    } // end of integrating over balls

    //CCP_LOGNOTICE_CH( s_chPStat,"[%I64d] Integration calculations: %f", mSolarsystemID, timer.GetTime()/10000.0);
    //timer.Reset();

    // Now let's update all values, moving them from their temporary values.
    VectorOfBalls trolls;

    for(sit = mFreeBalls.begin(); sit != mFreeBalls.end(); ++sit)
    {
        ball = sit->second;

        if(ball->isMoribund)
            continue; // don't handle dead balls

        if(ball->mInDeadBubble)
        {
            ball->mInDeadBubble = false;
            continue;
        }

        if(ball->mMode == DSTBALL_TROLL)
        {
            if(TrollReady(ball))
            {
                // Add troll to list of trolls to petrify
                trolls.push_back(ball);
            }
        }

        if(timestamp!=0)
        {
            // Assume that we are one time step ahead of time...
            if(ball->mNewTime==0)
                ball->mOldTime = timestamp - mTickInterval*10000;
            else
                ball->mOldTime = ball->mNewTime;
            ball->mNewTime = timestamp;
        }

        if(ball->mMode == DSTBALL_MUSHROOM)
        {
            long startTime = ball->mEffectStamp;
            float range = ball->mFollowRange;
            float timeFraction = (mCurrentTime - startTime)*mTickInterval*0.001f/(float)(ball->mGoto[0]);
            if(timeFraction < 1.0)
            {
                ball->mRadius = range*pow(timeFraction,0.25f);
                InsertBallInBoxes(ball);
                continue;
            }
            else
            {
                RemoveBall(ball->mId);
                continue;
            }
        }

        // Swap old and new values for position and velocity
        std::swap( ball->mNewPos, ball->mOldPos );
        std::swap( ball->mNewVel, ball->mOldVel );

		if( g_useDynamicalOrientation )
		{
			// Do not update the values if we end up with NaNs or infs.
			if( std::isfinite( LengthSq( ball->mOldRot ) ) )
				std::swap( ball->mNewRot, ball->mOldRot );
			if( std::isfinite( LengthSq( ball->mOldAngVel ) ) )
				std::swap( ball->mNewAngVel, ball->mOldAngVel );
			if( DynamicalOrientationDisabledForBall( ball ) )
			{
				ball->SnapOrientation();
			}
		}
		else
        {
            ball->CalculateYawPitchRoll();
        }

        Vector3d delta = ball->mNewPos - ball->mOldPos;
        if(delta.LengthSq() < 0.1)
        {
                continue; // No need to re-insert balls that haven't moved
        }

        // Re-insert the ball into the partition with transient flag false, which means this is the
        // tick final insert (voodoo)
        if(mProbe==ball->mId)
            CCP_LOGWARN_CH( s_chPark,"[%d] Insert from Evolve", mCurrentTime);

        InsertBallInBoxes(ball);

        // Dispatch collisions for missiles. If on server also check that the owner is still around,
        // otherwise trigger a missile explosion by adding a fake collision with self
        if (ball->mMode == DSTBALL_MISSILE)
        {
			if( g_useIterativeCollision )
			{
				// TODO: This should probably be changed, but for now use the mCollisions storage of ball
				// I don't think it is used anywhere else, but just be on the safe side
				auto coll_iter = mCollisionLocations.find(ball->mId);
				if (coll_iter != mCollisionLocations.end())
				{
					for (auto &loc : coll_iter->second.m_locations)
						if (loc.collider != -1)
							ball->mCollisions.push_back(loc.collider);
				}
			}
            if(isMaster)
            {
                // HACK ALERT: The ABS taken here is because of the defender missile hack (negative owner means defender)
                Ball *owner = mBalls[ABS(ball->mOwnerId)];
                if(!owner || (owner->mNewBubble != ball->mNewBubble && ball->mNewBubble != -1) || owner->isCloaked)
                    ball->mCollisions.push_back(ball->mId);
            }
            ball->DispatchCollisions();
        }
    }

    // Cycle over all trolls and petrify them. Need to do this seaparately, as the operation
    // changes the free ball list
    VectorOfBalls::iterator troll;
    for(troll = trolls.begin(); troll != trolls.end(); ++troll)
    {
        Ball *ball = *troll;
        PetrifyTroll(ball);
    }
    //CCP_LOGNOTICE_CH( s_chPStat,"[%I64d] Partition calculations: %f", mSolarsystemID, timer.GetTime()/10000.0);
    //timer.Reset();

    HandleProximities();

    // We're done. Increment the time by one.
    mCurrentTime++;

    // Release the deletion semaphore
    inEvolve = false;

    //CCP_LOGNOTICE_CH( s_chPStat,"[%I64d] Evolve cleanup: %f", mSolarsystemID, timer.GetTime()/10000.0);
    //timer.Reset();
}

Vector3d Ballpark::GetOrbitalNormal( Ball* ball, Vector3d& toVector, double& dist )
{
	Vector3d orbitalVector( 0.0, 0.0, 0.0 );

	if( !ball )
		return orbitalVector;

	if( ball->mMode != DSTBALL_ORBIT )
		return orbitalVector;

	Ball* other = ball->mFollowPtr;

	// Calculate this difference first, used in all paths below
	toVector = other->mNewPos - ball->mNewPos;

	if( !g_useNewOrbit )
	{
		dist = toVector.Length();
		// Make the vector unit
		if( dist > 0 )
			toVector /= dist;
		else
			// On top of the other ball, pick a "random" direction
			toVector = Vector3d( 0.0, 0.0, 1.0 );

		return DefaultOrbitalNormal( ball );
	}

	// Make a prediction on where he will be next timestep and calculate our
	// distance to that point.
	toVector += other->mNewVel * dt;
	dist = toVector.Length();
	// Make the vector unit
	if( dist > 0 )
		toVector /= dist;
	else
		// On top of the other ball, pick a "random" direction
		toVector = Vector3d( 0.0, 0.0, 1.0 );

	// Use the cross product between our velocity and the distance as the orbital plane.
	orbitalVector = toVector;
	orbitalVector.Cross( ball->mNewVel );
	auto length = orbitalVector.LengthSq();
	if( length > 1e-12 )
	{
		return orbitalVector / std::sqrt( length );
	}

	if( !g_useDynamicalOrientation )
	{
		return DefaultOrbitalNormal( ball );
	}

	// Use the cross product between our heading and the distance
	Vector3 heading( 0.0, 0.0, 1.0 );
	ball->GetRotatedVector( heading );
	orbitalVector = toVector;
	orbitalVector.Cross( Vector3d( heading ) );

	// Normalize, with a fallback in case we are pointing straight to/away from the target
	length = orbitalVector.LengthSq();
	if( length <= 1e-12 )
	{
		return DefaultOrbitalNormal( ball );
	}
	else
	{
		return orbitalVector / std::sqrt( length );
	}
}

Vector3d Ballpark::DefaultOrbitalNormal( Ball* ball )
{
	// Default precessing orbital plane
	double phi1 = mCurrentTime * ORBITAL_PRECESSION;
	// only use the last 16 bits of the mId
	// this ensures that different ships orbiting the same target will have different orbital planes
	double phi2 = ( ball->mId & 0x000000000000ffff ) + mCurrentTime * ORBITAL_PRECESSION;
	Vector3d orbitalVector( cos( phi1 ) * cos( phi2 ), sin( phi2 ), sin( phi1 ) * cos( phi2 ) );
	// shitround the components to 7 decimal digits
	orbitalVector.x = (double)( ( int64_t )( orbitalVector.x * 10000000 ) ) / 10000000;
	orbitalVector.y = (double)( ( int64_t )( orbitalVector.y * 10000000 ) ) / 10000000;
	orbitalVector.z = (double)( ( int64_t )( orbitalVector.z * 10000000 ) ) / 10000000;

	return orbitalVector;
}

void Ballpark::Integrate(Vector3d& p, Vector3d& v, const Vector3d& a, double m, double k, double timeFactor, double t)
{
	if (t == 0.0)
		return;

	if (t != dt)
		timeFactor = exp(-k / m * t);

	const double k2 = k * k;
	const double ook2 = 1. / k2;
	const double ook = 1. / k;

	if (k < 1e-10 * m * t)
	{
		//Taylor Series expansion
		//timeFactor = 1 - k/m*t
		p = (m * (a * (-k * t) + k * (a * t + v * k / m * t)) + p * k2) * ook2;
		v = (m * a - (m * a - v * k) * timeFactor) * ook;
	}
	else
	{
		p = (m * (m * a * (timeFactor - 1.0) + k * (a * t + v - timeFactor * v)) + p * k2) * ook2;
		v = (m * a - (m * a - v * k) * timeFactor) * ook;
	}

}
#pragma endregion


#pragma region Individual ball evolution
// -------------------------------------------------------------
//  Description:
//      Evolves a ball one tick given it's current mode (stored in ball->mMode).
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Nothing
// -------------------------------------------------------------
void Ballpark::EvolveBehaviorForBall(Ball* ball)
{
    ball->mLastG = Vector3d(0.0,0.0,0.0);
    ball->mLastC = Vector3d(0.0,0.0,0.0);
    ball->mCollisions.clear();

    Vector3d a = Vector3d(0.0,0.0,0.0);

    switch(ball->mMode){
    case DSTBALL_WARP:
        {
            a = EvolveWarp(ball);
            break;
        }
    case DSTBALL_GOTO:
        {
            a = EvolveGoto(ball);
            break;
        }
    case DSTBALL_MISSILE:
        {
            a = EvolveMissile(ball);
            break;
        }
    case DSTBALL_FORMATION:
        {
            a = EvolveFormation(ball);
            break;
        }
    case DSTBALL_FOLLOW:
        {
            a = EvolveFollow(ball);
            break;
        }
    case DSTBALL_ORBIT:
        {
            a = EvolveOrbit(ball, mCurrentTime);
            break;
        }
    case DSTBALL_STOP:
        {
            // This method operates directly on the velocity, no return value.
            EvolveStop(ball);
            break;
        }
    default:
        {
            break;
        }

    }; // End of switch statement

    if( g_useDynamicalOrientation && !DynamicalOrientationDisabledForBall( ball ) )
	{
		// This does not update the values for the ball, in case we adjust these
		// for collision responses.

		// Calculate the torque and roll
		// Roll into acceleration, if it is large enough
		const float accel = static_cast<float>( a.Length() );
		const float maxAccel = static_cast<float>( mFriction ) * ball->mSpeedFraction * ball->mMaxVel / ( static_cast<float>( ball->mMass ) * ball->mAgility );

		// To avoid rather annoying rotations when approaching a stationary ship using the follow command
		// The 1e-2 is not a sacred number, it may need to be changed
		if( accel > 1e-2 * maxAccel )
		{
			// Only need to evaluate the roll to adjust the acceleration
			ball->mTorque = GotoTorqueBreaks( ball, a.AsVector3() );
			ball->mRoll = GotoRollBreaks( ball, ball->mTorque );

			// Evaluate the new rotation to get the direction for the acceleration
			// Cheat here and use the new rotation for the acceleration
			// That stabilizes the algorithm
			auto rotation = ball->mNewRot;
			auto angvel = ball->mNewAngVel;

			const float tau = ( float( ball->mMass ) * ball->mAgility * ball->mRotAgility ) / float( mFriction );
			ApplyTorque( rotation, angvel, ball->mTorque, ball->mRoll, tau, 1.0f );

			const auto mat = RotationMatrix( rotation );
			const auto heading = TransformCoord( Vector3( 0., 0., 1. ), mat );

			const auto cosangle = std::min( 1.0f, Dot( heading, a.AsVector3() ) / accel );

			if( cosangle <= 0. )
				a = Vector3d( 0., 0., 0. );
			else
			{
				const auto factor = cosangle * cosangle;
				a = heading * ( factor * accel );
			}
		}
		else
		{
			// Heading to last known potision can lead to very strange rotations.  Keep current heading and roll upwards
			// When warping we should always align to goto position
			if( ball->IsWarping() )
				ball->mTorque = GotoTorqueBreaks( ball, ( ball->mGoto - ball->mNewPos ).AsVector3() );
			else
				ball->mTorque = Vector3( 0.0f, 0.0f, 0.0f );

			//Evaluate the new roll
			ball->mRoll = GotoRollBreaks( ball, ball->mTorque );
		}
	}
	else
	{
		CapAcceleration( ball, a );
	}

    ball->mLastG = a;
}


#pragma region Evolve methods for individual modes
// -------------------------------------------------------------
//  Description:
//      Calculates the acceleration vector for a ball in a "warp" mode.
//      If the ball is not aligned, we use the EvolveGoto method to align.
//      If we are, we call RealWarp to set up some values which are used later in WarpDistance and the return
//      value will be a zero vector.
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Acceleration vector for the ball.
// -------------------------------------------------------------
Vector3d Ballpark::EvolveWarp(Ball* ball)
{
    Vector3d zero = Vector3d(0.0,0.0,0.0);

    // Check if the warp proper is already in progress
    if(ball->IsWarping())
        return zero;

    //CCP_LOGWARN_CH( s_chPark,"... destination (%f, %f, %f), velocity (%f, %f, %f)", ball->mGoto.x, ball->mGoto.y, ball->mGoto.z, ball->mNewVel.x, ball->mNewVel.y, ball->mNewVel.z );
    //CCP_LOGWARN_CH( s_chPark,"... velDir (%f, %f, %f)", velDir.x, velDir.y, velDir.z);
    if( ball->IsAlignedForWarp())
    {
        if (!PyOS->PostEvent(
            (IEveBallpark*)this, "Destiny::OnActivatingWarp",
            "OnActivatingWarp", "Li", ball->mId, mCurrentTime
            ))
        {
            PyOS->PyError();
        }

        // Inititate the warp proper
        RealWarp(ball);

        return zero;
    }
    else
    {   // Not aligned, use goto...
        //CCP_LOG_CH(s_chPark, "EvolveWarp: Ball %I64d is still aligning at ts %d.", ball->mId, ABS(ball->mEffectStamp));
        --ball->mEffectStamp; // "increment" tick counter. The absolute value is used to determine if we should force-start warp in case the ship is stuck
        return EvolveGoto(ball);
    }
}

// -------------------------------------------------------------
//  Description:
//      Calculates the acceleration vector for a ball in a "goto" mode.
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Acceleration vector for the ball.
// -------------------------------------------------------------
Vector3d Ballpark::EvolveGoto(Ball* ball)
{
    // Make the acceleration the combination of collision avoidance
    // and a need to go to a point
    return GotoThrust(ball, ball->mGoto);
}


// -------------------------------------------------------------
//  Description:
//      Calculates the acceleration vector for a ball in a "missile" mode
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Acceleration vector for the ball.
// -------------------------------------------------------------
Vector3d Ballpark::EvolveMissile(Ball* ball)
{
    if( (mCurrentTime - ball->mEffectStamp)*mTickInterval <= 800 )
    {
        // Initial thrust in straight direction
        return GotoThrust(ball,ball->mGoto);
    }

    // This is the guy we are following
    Ball *other = ball->mFollowPtr;

    // This is where he currently is
    Vector3d otherPos = other->mNewPos;

    // This is the difference between me and the distance
    Vector3d delta = ball->mNewPos - otherPos;
    double dist = delta.Length();

    // If he was not moving I would get to him in:
    double cT = dist/ball->mMaxVel;

    // But during that time he will be at
    Vector3d target = otherPos + cT*other->mNewVel;
    double r = (double)ball->mFollowRange + (double)ball->mRadius + (double)other->mRadius;

    if(dist==0.0)
    {
        // I'm right on him. Choose to go out in some arbitrary direction
        target = otherPos + Vector3d(1.0,0.0,0.0)*r;
    }
    else
    {
        // put the goto point at the specified mFollowRange along the direction connecting us
        target = target + delta * r / dist;
    }

    // Save the goto direction for other to know
    ball->mGoto = target;
    return GotoThrust(ball, target, ball->mMode==DSTBALL_MISSILE);
}


// -------------------------------------------------------------
//  Description:
//      Calculates the acceleration vector for a ball in a "follow" mode. If not in formation,
//      the returned value will be a zero vector.
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Acceleration vector for the ball.
// -------------------------------------------------------------
Vector3d Ballpark::EvolveFormation(Ball* ball)
{
    Vector3d zero = Vector3d(0.0,0.0,0.0);

    // This is the guy we are following
    Ball *other = ball->mFollowPtr;

    if(other->mFormationID < 0)
    {
        return zero;
    }

    if(other->mFormationID > int(mFormations.size())-1)
    {
        return zero;
    }

    char slot = char(ball->mEffectStamp);
    if(slot > int(mFormations[other->mFormationID].size())-1)
    {
        return zero;
    }

    Vector3d offset = mFormations[other->mFormationID][slot];
    double length = offset.Length();

    if(length==0.0)
        length = 1.0;

    // This is where he currently is
    Vector3d otherPos = other->mNewPos;
    Vector3 vec = (((double)ball->mRadius + (double)other->mRadius + length)*offset/length).AsVector3();
    other->GetRotatedVector(vec);

    otherPos += vec;

    ball->mGoto = otherPos;
    return GotoThrust(ball, otherPos);
}


// -------------------------------------------------------------
//  Description:
//      Calculates the acceleration vector for a ball in a "follow" mode. From the perspective of a player,
//      this is the "keep at range" mode.
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Acceleration vector for the ball.
// -------------------------------------------------------------
Vector3d Ballpark::EvolveFollow(Ball* ball)
{
    // create a 'follow' potential at a given distance from a ball

    // This is the guy we are following
    Ball *other = ball->mFollowPtr;

    // This is where he currently is
    Vector3d otherPos = other->mNewPos;

    // This is the difference between me and the distance
    Vector3d delta = ball->mNewPos - otherPos;
    double dist = delta.Length();
    double r = (double)ball->mFollowRange + (double)ball->mRadius + (double)other->mRadius;

    Vector3d target;

    if(dist==0.0)
    {
        // I'm right on him. Choose to go out in some arbitrary direction
        target = otherPos + Vector3d(1.0,0.0,0.0)*r;
    }
    else
    {
        // put the goto point at the specified mFollowRange along the direction connecting us
        target = otherPos + delta * r/ dist;
    }

    // Save the goto direction for other to know
    ball->mGoto = target;
    return GotoThrust(ball, target, ball->mMode==DSTBALL_MISSILE);
}

// -------------------------------------------------------------
//  Description:
//      Calculates the acceleration vector for a ball in an "orbit" mode using some insane math.
//      Updates the goto position of the ball each time it is called.
//      Uses the appropriate method to evolve the orbit depending on settings value.
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Acceleration vector for the ball.
// -------------------------------------------------------------
Vector3d Ballpark::EvolveOrbit( Ball* ball, long currentTime )
{
	if(g_useNewOrbit)
	{
		return EvolveNewStyleOrbit( ball, currentTime );
	}
	return EvolveOldStyleOrbit(ball, currentTime);
}

// -------------------------------------------------------------
//  Description:
//      Calculates the acceleration vector for a ball in an "orbit" mode using some insane math.
//      Updates the goto position of the ball each time it is called.
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Acceleration vector for the ball.
// -------------------------------------------------------------
Vector3d Ballpark::EvolveNewStyleOrbit( Ball* ball, long currentTime )
{
	// The maximum thrust of this ball, given its mass and current cruise velocity
	const double cruiseVelocity = (double)( ball->mSpeedFraction ) * (double)( ball->mMaxVel );
	const double k = mFriction;
	const double tau = ( ball->mMass * ball->mAgility ) / k;
	const double maxThrust = cruiseVelocity / tau;

	// This is the guy we are following
	Ball* other = ball->mFollowPtr;

	// This is the desired distance between the balls
	double r = (double)ball->mFollowRange + (double)ball->mRadius + (double)other->mRadius;

	// Make a prediction on where he will be next timestep and calculate our
	// distance to that point
	Vector3d toVector;
	double dist;
	Vector3d orbitalVector = GetOrbitalNormal( ball, toVector, dist );

	// This is in the direction of the orbit
	Vector3d radialVector = orbitalVector;
	radialVector.Cross( toVector );
	radialVector.Normalize();
	// radial is now transverse to the toVector

	// Move along an exponential spiral, whose angle decreases linearly as we approach
	// It is parameterised as x(t) = d * exp(a*t) * cos(t) and y(t) = d * exp(a*t) * sin(t)
	// with a = tan(alpha) where alpha is the angle of impact
	// The connection between t and the distance along the spiral s is given by
	// s = d * sqrt(a**2 + 1)/a (exp(a*t) - 1)

	// It is possible to have rather rapid speed variations, which will cause this to change.
	// If we are currently moving at greater speed than our current speed, use that as our maximum speed
	const double maxSpeed = std::max( cruiseVelocity, ball->mNewVel.Length() );

	// Use the time to cover the distance at maximum velocity to set the
	// exponential factor and therefore angle of impact.  Cap the exponential factor between -1 and 1
	// equivalent to moving at most at 45 degree angle
	double angle = 0.1 * ( r - dist ) / maxSpeed;
	if( angle < -1 )
		angle = -1;
	if( angle > 1 )
		angle = 1;

	// put a lower limit on dist for the rest to work
	if( dist < 0.1 )
		dist = 0.1;


	// Estimate the maximum velocity based on the curvature of the track.  This limits the velocity
	// to a value where the maximum thrust is used to keep us on track, ignoring friction
	const double max_velocity = std::sqrt( dist * maxThrust );

	//Use the lower of cruise/current velocity and the curvature velocity.
	const double estimatedVelocity = std::min( maxSpeed, max_velocity );
	const double estimatedDistance = estimatedVelocity * dt;

	// Heading this way, relative to our position
	Vector3d gotoPoint;

	// At that point, we want this velocity
	Vector3d gotoVelocity;

	// If angle is small, we really just want a circle
	if( std::fabs( angle ) < 1e-10 )
	{
		const double t_desired = estimatedDistance / dist;
		gotoPoint = toVector * ( dist * ( 1.0 - std::cos( t_desired ) ) ) +
			radialVector * ( dist * std::sin( t_desired ) );

		gotoVelocity = radialVector * ( std::cos( t_desired ) * estimatedVelocity ) + toVector * ( std::sin( t_desired ) * estimatedVelocity );
	}
	else
	{
		const double factor = estimatedDistance * angle / dist / std::sqrt( angle * angle + 1 );
		const double t_desired = factor < -0.999 ?
			std::log( 0.001 ) / angle :
			( std::fabs( factor ) > 1e-10 ? std::log( 1 + factor ) / angle : factor / angle );

		// Our desired destination is the current point plus that travelled along the spiral.  Set that as a goto direction
		const auto dexpterm = dist * std::exp( angle * t_desired );
		gotoPoint = toVector * ( dist - dexpterm * std::cos( t_desired ) ) +
			radialVector * ( dexpterm * std::sin( t_desired ) );

		// We want the velocity to be along the trajectory at maximum
		gotoVelocity = toVector * ( -angle * std::cos( t_desired ) + std::sin( t_desired ) ) +
			radialVector * ( angle * std::sin( t_desired ) + std::cos( t_desired ) );
		gotoVelocity.Normalize();
		gotoVelocity *= estimatedVelocity;
	}

	// Set that as the gotopoint
	ball->mGoto = ball->mNewPos + gotoPoint;

	// We want the velocity to be at maximum along the trajectory + whatever velocity the target is moving at
	gotoVelocity += other->mNewVel;

	// Use the gotothrust to get there, equal weights to velocity and point
	return GotoThrustFollow( ball, ball->mGoto, gotoVelocity, 1.0 );
}


// -------------------------------------------------------------
//  Description:
//      Calculates the acceleration vector for a ball in an "orbit" mode using some insane math.
//      Updates the goto position of the ball each time it is called.
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Acceleration vector for the ball.
// -------------------------------------------------------------
Vector3d Ballpark::EvolveOldStyleOrbit(Ball* ball, long currentTime)
{
    // The maximum thrust of this ball, given its mass and current cruise velocity
    double cruiseVelocity = (double)(ball->mSpeedFraction) * (double)(ball->mMaxVel);
    double k = mFriction;
    double maxThrust = k*cruiseVelocity/(ball->mMass * ball->mAgility);

    // Orbit is exactly like follow, except that the target point can be anywhere
    // on the sphere defined by the follow range, and it changes continuously according
    // to some random sequence

    // This is the guy we are following
    Ball *other = ball->mFollowPtr;

    // This is where he currently is
    Vector3d otherPos = other->mNewPos;

    // This is the desired distance between the balls
    double r = (double)ball->mFollowRange + (double)ball->mRadius + (double)other->mRadius;

    // This is the current distance between the two
    Vector3d toVector = (otherPos - ball->mNewPos);
    double dist = toVector.Length();
    // Make it unit
    toVector.Normalize();

    double phi1 = currentTime * ORBITAL_PRECESSION;
    // only use the last 16 bits of the mId
    // this ensures that different ships orbiting the same target will have different orbital planes
    double phi2 = (ball->mId&0x000000000000ffff) + currentTime * ORBITAL_PRECESSION;
    Vector3d radialVector = Vector3d(cos(phi1) * cos(phi2), sin(phi2), sin(phi1) * cos(phi2));
    // shitround the components to 7 decimal digits
    radialVector.x =  (double)((int64_t)(radialVector.x*10000000))/10000000;
    radialVector.y =  (double)((int64_t)(radialVector.y*10000000))/10000000;
    radialVector.z =  (double)((int64_t)(radialVector.z*10000000))/10000000;
                
    radialVector.Cross(toVector);
    radialVector.Normalize();
    // radial is now transverse to the toVector

    // Now I am going to choose a goto point that is tangent to the orbit
    double toComp = (dist*dist - r*r);
    if(toComp >= 0.0)
    {
        double radComp =  r*sqrt(toComp)/dist;

        toVector = (toComp/dist)*toVector + radComp*radialVector;
        toVector.Normalize();
    }

    // This is the ratio of thrust to put into radial component
    double radialFactor = exp(-(r-dist)*(r-dist)/40000.0);

    // Shitfix, exp() returns slightly different results depending on target platform (32bit or 64bit) so 
    // we round to 7 significant decimal digits.
    radialFactor =  (double)((int64_t)(radialFactor*10000000))/10000000;

    // Now use the law of cosine rule to get the other factor
    // First dot product of these guys is equal to the cosine of the angle between them,
    // but we are interested in the pi-angle, whence the minus sign
    phi1 =  -toVector*radialVector;

    // Now this is the determinant for the solutions
    double transverseFactor = 1.0+radialFactor*radialFactor *(phi1*phi1-1.0);

    if(transverseFactor > 0.0)
    {
        // Choose the plus solution
        transverseFactor = radialFactor*phi1 + sqrt(transverseFactor);
    }
    else
    {
        // Imaginary component in solution, just use the real part.
        transverseFactor = radialFactor*phi1;
    }

    transverseFactor *=SIGNUM(dist-r);

    // Now calculate the actual acceleration
    Vector3d a = (radialFactor*radialVector + transverseFactor*toVector)*maxThrust;
    ball->mGoto = ball->mNewPos + 10.0*AU*a;

    return a;
}

// -------------------------------------------------------------
//  Description:
//      Updates the velocity of a ball in order to make it stop.
//  Arguments:
//      ball - the ball to evolve
//  Returns:
//      Nothing
// -------------------------------------------------------------
void Ballpark::EvolveStop(Ball* ball)
{
	if( !g_useDynamicalOrientation )
	{
		ball->mNewVel[1] = ( ball->mNewVel[1] - 0.07 * ball->mNewVel[1] ) * 0.9345794392523364485981308411215;
	}
}
#pragma endregion


#pragma region Common evolve functions for all modes
void Ballpark::CapAcceleration(const Ball *ball, Vector3d& a)
{
    // Disabling turn angle for now
    return;
    // Basically we want to limit the angle between the acceleration and the current speed vector
    double velLen2 = ball->mNewVel.LengthSq();
    if(velLen2==0.0)
        return; // No velocity vector to compare with

    double accLen2 = a.LengthSq();
    if(accLen2==0.0)
        return; // No acceleration vector to cap

    Vector3d oldA = a;
    Vector3d unitVel = ball->mNewVel/sqrt(velLen2);
    Vector3d unitAcc = a/sqrt(accLen2);

    double avAngle = unitVel*unitAcc;

    // Make sure we are within the domain of the acos function otherwise evil will happen
    if(avAngle > 1.0)
        avAngle = 1.0;
    else if(avAngle < -1.0)
        avAngle = -1.0;

    avAngle = acos(avAngle);

    if(avAngle <= ball->mAgility)
        return; // No need to cap

    // Need to rotate the acceleration vector towards the velocity vector
    double angle = ball->mAgility - avAngle;
    // 'v' will be the axis around which we turn
    Vector3d axis = ball->mNewVel;
    axis.Cross(a);
    axis.Normalize();
    QuaternionD q = QuaternionD(axis, angle);
    a = q.RotateVector(a);
    if(isMaster && !_finite(a.x)){
        CCP_LOGERR_CH( s_chPark,"OldAcceleration for %I64d: [%f, %f, %f]", ball->mId, oldA.x, oldA.y, oldA.z);
        CCP_LOGERR_CH( s_chPark,"NewAcceleration for %I64d: [%f, %f, %f]", ball->mId, a.x, a.y, a.z);
        CCP_LOGERR_CH( s_chPark,"NewVel for %I64d: [%f, %f, %f]", ball->mId,ball->mNewVel.x,ball->mNewVel.y, ball->mNewVel.z);
        CCP_LOGERR_CH( s_chPark,"Axis for %I64d: [%f, %f, %f]", ball->mId, axis.x, axis.y, axis.z);
        CCP_LOGERR_CH( s_chPark,"Angles for %I64d: [%f, %f]", ball->mId, avAngle, angle);
    }

}

Vector3d Ballpark::GotoThrust(const Ball *ball, const Vector3d& target, bool missile)
{
    // This is the basic direction of the thrust: difference between the target and current point
    Vector3d a = (target - ball->mNewPos);
    double length2 = a.LengthSq();

    // This is the distance that the ball can travel during one time step
    double dist = (double)ball->mSpeedFraction * ball->mMaxVel * dt;

    // This is the maximum thrust of this ball, as defined by its mass, agility and max speed
    double maxThrust = mFriction*ball->mSpeedFraction*(double)ball->mMaxVel/(ball->mMass * ball->mAgility);

    if(!missile && length2 < dist*dist )
    {
        // if we are within 'homing' distance, then essentially use maxThrust
        // scaled with the length of the delta
        a.Normalize();
        double coff = length2/(dist*dist);
        a *= maxThrust*coff*coff;
    }
    else
    {
        // Use maxThrust
        a.Normalize();
        a *= maxThrust;
    }

    return a;
}

Vector3d Ballpark::OrbitThrust( const Ball* ball, const Vector3d& target )
{
	// The maximum acceleration depends on the maximum velocity, friction, mass, and agility.
	const auto frictionOverMass = mFriction / ( ball->mMass * ball->mAgility );
	const auto maxAccel = frictionOverMass * ball->mSpeedFraction * (double)ball->mMaxVel;

	// maxAccel should be > 0 for the rest to work.  This should not happen, but who knows.
	if( maxAccel <= 0 )
	{
		return Vector3d( 0.0, 0.0, 0.0 );
	}

	// The direction for the new acceleration
	Vector3d a = ( target - ball->mNewPos );

	// Regular ships strive to stop at their destination
	// k/m(target - current) - velocity
	a *= frictionOverMass;
	a -= ball->mNewVel;

	// Now we can calculate the time required for the current acceleration
	const auto tacc = a.Length() / maxAccel;

	// If tacc is greater than 1 we need to cap at maximum acceleration
	// Otherwise we slow down gracefully
	if( tacc > 1 )
		a /= tacc;

	// Simply stop if tacc is small enough.  This will reduce crazy wobbling
	if( tacc < 0.1 )
		return Vector3d(0.0, 0.0, 0.0);

	return a;
}

Vector3d Ballpark::GotoThrustFollow( const Ball* b, const Vector3d& target, const Vector3d& targetVelocity, double weight )
{
	// If target velocity is small, use gotothrust
	const double speed = targetVelocity.Length();

	if( speed < 5e-2 * b->mMaxVel * b->mSpeedFraction )
	{
		if( g_useNewOrbit && b->mMode == DSTBALL_ORBIT )
		{
			return OrbitThrust( b, target );
		}
		return GotoThrust( b, target );
	}

	// The maximum acceleration depends on the maximum velocity, friction, mass, and agility.
	const auto frictionOverMass = mFriction / ( b->mMass * b->mAgility );
	const auto maxAccel = frictionOverMass * b->mSpeedFraction * (double)b->mMaxVel;

	// maxAccel should be > 0 for the rest to work.  This should not happen, but who knows.
	if( maxAccel <= 0 )
	{
		return Vector3d( 0.0, 0.0, 0.0 );
	}

	// This is the acceleration required to get to target in a single timestep
	Vector3d a = ( ( target - b->mNewPos ) * frictionOverMass - b->mNewVel * ( 1 - b->mTimeFactor ) ) * frictionOverMass /
		( dt * frictionOverMass - ( 1 - b->mTimeFactor ) );

	// Here is the acceleration required to have the required velocity
	const auto avel = ( targetVelocity - b->mNewVel * b->mTimeFactor ) * frictionOverMass / ( 1.0 - b->mTimeFactor );

	// Use a weighted average of the two to regulate fluctuations
	a = ( a + weight * avel ) / ( 1.0 + weight );

	// Make sure we are within limits
	double accel = a.LengthSq();

	if( accel > maxAccel * maxAccel )
	{
		a *= maxAccel / std::sqrt( accel );
	}

	return a;
}

#pragma endregion
#pragma endregion

Vector3 Ballpark::GotoTorque( const Ball* ball, const Vector3& direction )
{
	// Start with some cheap tests before we start the calculations
	const float tScale = float( mFriction ) / ( float( ball->mMass ) * ball->mAgility * ball->mRotAgility );
	const auto maxAlpha = ball->mMaxAngVel * tScale;

	// It has to be positive
	if( maxAlpha <= 0.0f )
		return Vector3( 0.0f, 0.0f, 0.0f );

	// We need the direction to point somewhere
	const auto lenDir = Length( direction );
	if( lenDir == 0 )
		return Vector3( 0.0f, 0.0f, 0.0f );

	// Get the rotation from ship to world frame for the heading
	const Matrix mat = RotationMatrix( ball->mNewRot );

	const auto heading = Normalize( TransformCoord( Vector3( 0.0f, 0.0f, 1.0f ), mat ) );

	// Calculate required rotation
	// The coordinates are designed such the z is forward, y is up, and x is right.
	// pitch is around x-axis, yaw around y axis, roll around z-axis
	// The coordinates are left handed???  Lets find out.
	// The rotation is perpendicular to heading and the new direction
	auto rot = Cross( heading, direction ) / lenDir;

	// The magnitude is the sin(angle) between the two, need to normalize to the actual angle
	auto rotLength = Length( rot );

	// Much simpler to use the dot product to calculate the direction
	const auto rotAngle = std::acos( std::max( float( -1. ), std::min( float( 1. ), Dot( direction, heading ) / lenDir ) ) );

	// The cross product gives the correct orientation, but need to scale it
	// If we flip 180 degrees, we have no preferred rotation vector, use ships -x axis
	// If rotLength is very small and the rotation is very small, the cross product gives the correct answer.
	if( rotLength > 1e-12 )
	{
		rot *= rotAngle / rotLength;
	}
	else if( rotAngle > 1 )
	{
		rot = Vector3( -rotAngle, 0.0f, 0.0f );
	}

	// Roll is calculated separately and completely ignored here
	// Need to subtract the roll from the angular velocity, roll is treated separately
	const auto omega0 = ball->mNewAngVel - Dot( heading, ball->mNewAngVel ) * heading;

	// Calculate both positive and negative rotation and pick the faster one
	// The positive is faster in almost all cases
	rot *= tScale;
	auto tAlpha = rot - omega0;
	// The negative turns the opposite direction
	auto tAlphaNeg = ( Vector3( 1.0f, 1.0f, 1.0f ) * ( 2.0f * float( _PI_ ) * tScale ) - rot ) - omega0;
	for( int32_t i( 0 ); i < 3; ++i )
		if( fabs( tAlpha[i] ) > fabs( tAlphaNeg[i] ) )
			tAlpha[i] = tAlphaNeg[i];


	// Do nothing if tAcc is < 1, otherwise cap at maxAlpha
	const auto tAcc = Length( tAlpha ) / maxAlpha;

	if( tAcc > dt )
		tAlpha /= tAcc;

	return tAlpha;
}

static inline void CalculateN2Alpha2( int& n2, float& kappa2, Vector3& alpha2, const Vector3& omega0, const float lenOmega0, const Ball* ball, const float tf, const float dt, const float tScale )
{
	if( lenOmega0 > 1e-10 )
	{
		const auto factor = ball->mMaxAngVel / lenOmega0;
		n2 = int( std::ceil( std::log( factor / ( 1.f + factor ) ) / ( -dt * tScale ) ) );
		kappa2 = std::pow( tf, n2 );
		alpha2 = -omega0 * ( tScale * kappa2 / ( 1 - kappa2 ) );
	}
	else
	{
		n2 = 0;
		//alpha2 is never used if n2=0
		alpha2 = Vector3( 0.f, 0.f, 0.f );
		kappa2 = 1.f;
	}
}

static inline void CalculateN2Alpha2( int& n2, float& kappa2, float& alpha2, const float omega0, const Ball* ball, const float tf, const float dt, const float tScale )
{
	if( std::fabs( omega0 ) > 1e-10 )
	{
		const auto factor = ball->mMaxAngVel / std::fabs( omega0 );
		n2 = int( std::ceil( std::log( factor / ( 1.f + factor ) ) / ( -dt * tScale ) ) );
		kappa2 = std::pow( tf, n2 );
		alpha2 = -omega0 * ( tScale * kappa2 / ( 1 - kappa2 ) );
	}
	else
	{
		n2 = 0;
		//alpha2 is never used if n2=0
		alpha2 = 0.f;
		kappa2 = 1.f;
	}
}

Vector3 Ballpark::GotoTorqueBreaks( const Ball* ball, const Vector3& direction )
{
	// Use a fraction of the timescale as is used for linear motion.
	// We can adjust this with a tuning factor
	const float tScale = float( mFriction ) / ( float( ball->mMass ) * ball->mAgility * ball->mRotAgility );

	const float maxAlpha = ball->mMaxAngVel * tScale;

	// It has to be positive
	if( maxAlpha <= 0.0f )
		return Vector3( 0.0f, 0.0f, 0.0f );

	// We need the direction to point somewhere
	const auto lenDir = Length( direction );
	if( lenDir == 0 )
		return Vector3( 0.0f, 0.0f, 0.0f );

	// Get the rotation from ship to world frame for the heading
	const Matrix mat = RotationMatrix( ball->mNewRot );

	const auto heading = Normalize( TransformCoord( Vector3( 0.0f, 0.0f, 1.0f ), mat ) );

	// Calculate required rotation
	// The coordinates are designed such the z is forward, y is up, and x is right.
	// The rotation is perpendicular to heading and the new direction
	auto rot = Cross( heading, direction ) / lenDir;

	// The magnitude is the sin(angle) between the two, need to normalize to the actual angle
	auto rotLength = Length( rot );

	// Much simpler to use the dot product to calculate the direction
	// Need to clip the value to be within -1 and 1.
	const auto rotAngle = std::acos( std::max( float( -1. ), std::min( float( 1. ), Dot( direction, heading ) / lenDir ) ) );

	// The cross product gives the correct orientation, but need to scale it
	// If we flip 180 degrees, we have no preferred rotation vector, use ships -x axis
	// If rotLength is very small and the rotation is very small, the cross product gives the correct answer.
	if( rotLength > 1e-12 )
	{
		rot *= rotAngle / rotLength;
	}
	else if( rotAngle > 1 )
	{
		rot = Vector3( -rotAngle, 0.0f, 0.0f );
	}

	// Roll is calculated separately and completely ignored here
	// Need to subtract the roll from the angular velocity, roll is treated separately
	const auto omega0 = ball->mNewAngVel - Dot( heading, ball->mNewAngVel ) * heading;

	//This is where we differ from the other version
	//Apply an iterative algorithm to find the timesteps required for breaking and accelerating

	//No torque if both rotation and omega are small
	const auto lenOmega0 = Length( omega0 );
	if( lenOmega0 * dt < 1e-4 && rotAngle < 1e-4 )
		return Vector3( 0.f, 0.f, 0.f );

	//Find two pairs of timesteps and alpha
	//1 is acceleration, 2 is deceleration
	int n1( 0 ), n2( 0 );
	Vector3 alpha1, alpha2;
	float kappa1( 1.f ), kappa2( 1.f );
	const float tf = std::exp( -float( dt ) * tScale );
	const float tau = 1.f / tScale;

	//Start by finding n2 and alpha2 to stop the current velocity, if any
	CalculateN2Alpha2( n2, kappa2, alpha2, omega0, lenOmega0, ball, tf, float( dt ), tScale );

	//Now we can start the iterations.  There is a possibility of an ifinite loop, because the solution is not unique.
	//If many iterations are needed, it means n1 is large
	constexpr int NITER = 5;
	for( int count = 0; count < NITER; ++count )
	{
		//First check to see if n1=0 is a match.
		//Separate that out because it is different
		if( n2 > 0 )
		{
			const auto theta2 = tau * alpha2 * ( float( n2 * dt ) - tau * ( 1.f - kappa2 ) ) + tau * omega0 * ( 1 - kappa2 );
			if( Length( rot - theta2 ) < 1e-3 )
			{
				n1 = 0;
				break;
			}
		}

		//Then we find a proper value for n1 and alpha1
		kappa1 = 1.f;
		for( n1 = 1; n1 < 100; ++n1 )
		{
			kappa1 *= tf;
			if( n2 == 0 )
			{
				alpha1 = ( rot - omega0 * ( tau * ( 1.f - kappa1 ) ) ) / ( tau * ( float( n1 * dt ) - tau * ( 1.f - kappa1 ) ) );
			}
			else
			{
				alpha1 = ( rot - omega0 * ( tau - kappa1 * kappa2 * float( n2 * dt ) / ( 1 - kappa2 ) ) ) / ( tau * ( float( n1 * dt ) - float( n2 * dt ) * kappa2 * ( 1 - kappa1 ) / ( 1 - kappa2 ) ) );
			}

			const auto lenAlpha1 = Length( alpha1 );
			if( lenAlpha1 < maxAlpha )
			{
				break;
			}
		}

		//Calculate a new value for n2 and see if it differs
		const int n2Old = n2;
		const auto omega1 = alpha1 * ( tau * ( 1 - kappa1 ) ) + omega0 * kappa1;
		CalculateN2Alpha2( n2, kappa2, alpha2, omega1, Length( omega1 ), ball, tf, float( dt ), tScale );

		//Stop if n2 does not change
		if( n2Old == n2 )
			break;
	}

	if( n1 > 0 )
	{
		if( n1 > 1 )
		{
			//Accelerate full speed in the beginning
			alpha1 = Normalize( alpha1 );
			alpha1 *= maxAlpha;
		}
		return alpha1;
	}

	if( n2 > 0 )
		return alpha2;

	//This should never happen, but keep it here for safety
	return Vector3( 0.f, 0.f, 0.f );
}

float Ballpark::GotoRollBreaks( const Ball* ball, const Vector3& headingTorque )
{
	// Use a fraction of the timescale as is used for linear motion.
	// We can adjust this with a tuning factor
	const auto tScale = float( mFriction ) / ( float( ball->mMass ) * ball->mAgility * ball->mRotAgility );
	const auto maxAlpha = tScale * ball->mMaxAngVel;

	// It has to be positive
	if( maxAlpha <= 0 )
		return 0.;

	// This matrix transforms from world frame to ship frame
	const auto mat = RotationMatrix( Conjugate( ball->mNewRot ) );

	// Transform the angular velocity to the ship frame to get the roll velocity
	auto shipOmega = TransformCoord( ball->mNewAngVel, mat );
	const auto omegaRoll = shipOmega.z;

	// Roll into turns
	// We want the heading torque to be completely aligned with the negative x-axis
	// for very sharp turns only.
	// This will look like an airplane
	const auto lenTorque = Length( headingTorque );

	// To avoid rolling into the breaking torque, make sure the current
	// angular velocity aligns with the torque.
	// Use maximum angular velocity to scale the turnRoll with upRoll
	// Only roll into significant turns
	float turnRoll = 0.f;
	float weight = 0.f;
	if( lenTorque > 0.05 * maxAlpha )
	{
		// Need to rotate the torque into the ships frame
		auto shipTorque = TransformCoord( headingTorque, mat );
		// Need to avoid the break roll turns, only roll into the torque when the angular velocity
		// aligns with the turn
		shipOmega.z = 0.f;
		auto cosAngle = Dot( shipOmega, shipTorque );
		if( cosAngle >= 0 || Length( shipOmega ) < 0.01f * ball->mMaxAngVel )
		{
			turnRoll = std::atan2( -shipTorque.y, -shipTorque.x );
			// Use an absolute scale to roll into turns, depending on the maximum
			// possible velocity and the fractional torque
			weight = lenTorque / tScale;
			if( weight > 1.f )
				weight = 1.f;
		}
	}

	// Also roll upwards if we are not turning strongly and not moving straight up
	const auto up = TransformCoord( Vector3( 0.0f, 1.0f, 0.0f ), mat );
	const auto upRoll = std::fabs( up.z < 0.95f ) ? std::atan2( -up.x, up.y ) : 0.0f;

	// Use the upRoll when heading rotation is small
	const auto desiredRoll = weight < 0.05f ? upRoll : weight * turnRoll;

	// Short circuit if no rotation is required
	if( std::fabs( omegaRoll ) < 1e-4 && std::fabs( desiredRoll ) < 1e-4 )
		return 0.0f;

	//Find two pairs of timesteps and alpha
	//1 is acceleration, 2 is deceleration
	int n1( 0 ), n2( 0 );
	float alpha1, alpha2;
	float kappa1( 1.f ), kappa2( 1.f );
	const float timeFactor = std::exp( -float( dt ) * tScale );
	const float tau = 1.f / tScale;

	//Start by finding n2 and alpha2 to stop the current velocity, if any
	CalculateN2Alpha2( n2, kappa2, alpha2, omegaRoll, ball, timeFactor, float( dt ), tScale );

	//Now we can start the iterations.  There is a possibility of an ifinite loop, because the solution is not unique.
	//If many iterations are needed, it means n1 is large
	constexpr int NITER = 5;
	for( int count = 0; count < NITER; ++count )
	{
		//First check to see if n1=0 is a match.
		//Separate that out because it is different
		if( n2 > 0 )
		{
			const auto theta2 = tau * alpha2 * ( float( n2 * dt ) - tau * ( 1.f - kappa2 ) ) + tau * omegaRoll * ( 1 - kappa2 );
			if( std::fabs( desiredRoll - theta2 ) < 1e-3 )
			{
				n1 = 0;
				break;
			}
		}

		//Then we find a proper value for n1 and alpha1
		kappa1 = 1.f;
		for( n1 = 1; n1 < 100; ++n1 )
		{
			kappa1 *= timeFactor;
			if( n2 == 0 )
			{
				alpha1 = ( desiredRoll - omegaRoll * ( tau * ( 1.f - kappa1 ) ) ) / ( tau * ( float( n1 * dt ) - tau * ( 1.f - kappa1 ) ) );
			}
			else
			{
				alpha1 = ( desiredRoll - omegaRoll * ( tau - kappa1 * kappa2 * float( n2 * dt ) / ( 1 - kappa2 ) ) ) / ( tau * ( float( n1 * dt ) - float( n2 * dt ) * kappa2 * ( 1 - kappa1 ) / ( 1 - kappa2 ) ) );
			}

			if( std::fabs( alpha1 ) < maxAlpha )
			{
				break;
			}
		}

		//Calculate a new value for n2 and see if it differs
		const int n2Old = n2;
		const auto omega1 = alpha1 * tau * ( 1 - kappa1 ) + omegaRoll * kappa1;
		CalculateN2Alpha2( n2, kappa2, alpha2, omega1, ball, timeFactor, float( dt ), tScale );

		//Stop if n2 does not change
		if( n2Old == n2 )
			break;
	}

	if( n1 > 0 )
	{
		if( n1 > 1 )
		{
			//We can safely accelerate full speed one more time.  The code adjusts later
			alpha1 = SIGNUMF( alpha1 ) * maxAlpha;
		}
		return alpha1;
	}

	if( n2 > 0 )
		return alpha2;

	//This should never happen, but keep it here for safety
	return 0.f;
}

float Ballpark::GotoRoll( const Ball* ball, const Vector3& headingTorque )
{
	// Use a fraction of the timescale as is used for linear motion.
	// We can adjust this with a tuning factor
	const auto tScale = float( mFriction ) / ( float( ball->mMass ) * ball->mAgility * ball->mRotAgility );
	const auto maxAlpha = tScale * ball->mMaxAngVel;

	if( maxAlpha <= 0 )
		return 0.;

	// Roll into turns, but only if there is some heading torque
	// We want the heading torque to be as much in the negative x-axis as possible
	// This will look like an airplane
	const auto lenTorque = Length( headingTorque );

	// This matrix transforms from world frame to ship frame
	const auto mat = RotationMatrix( Conjugate( ball->mNewRot ) );
	float turnRoll = 0.;
	if( lenTorque > 1e-10 )
	{
		// Need to rotate the torque into the ships frame
		auto shipTorque = TransformCoord( headingTorque, mat );
		turnRoll = std::atan2( -shipTorque.y, -shipTorque.x );
	}

	// Also roll upwards if we are not turning strongly and not moving straight up
	const auto up = TransformCoord( Vector3( 0.0f, 1.0f, 0.0f ), mat );
	const auto upRoll = std::fabs( up.z < 0.95f ) ? std::atan2( -up.x, up.y ) : 0.0f;

	// Use a simple cut between the two
	const auto weight = lenTorque / maxAlpha;
	const auto desiredRoll = weight < 0.1 ? upRoll : turnRoll;

	// The current rotation along the z-axis is the starting angular speed
	// Use that to estimate the desired roll_torque direction
	const auto omegaRoll = TransformCoord( ball->mNewAngVel, mat ).z;
	auto tauRoll = desiredRoll * tScale - omegaRoll;
	const auto tauRollNeg = ( 2.0f * float( _PI_ ) - desiredRoll ) * tScale - omegaRoll;
	if( std::fabs( tauRollNeg ) < std::fabs( tauRoll ) )
		tauRoll = tauRollNeg;

	// Cap the roll at maximum alpha
	const auto tAcc = std::fabs( tauRoll ) / maxAlpha;

	if( tAcc > 1 )
		tauRoll /= tAcc;

	return tauRoll;
}

void Ballpark::ApplyTorque( Quaternion& rotation, Vector3& angvel, const Vector3& torque, const float roll, const float tau, float timeStepFraction )
{
	const float timeFactor = std::exp( -float( dt ) * timeStepFraction / tau );

	//Need to separate the rotational velocity into heading and roll
	const auto mat = RotationMatrix( rotation );
	auto heading = TransformCoord( Vector3( 0.0f, 0.0f, 1.0f ), mat );
	const auto omegaRoll = Dot( angvel, heading );
	const auto omegaHeading = angvel - heading * omegaRoll;

	// Use the heading vector for the roll rotation
	const auto rotRoll = tau * ( roll * ( timeStepFraction * float( dt ) - tau * ( 1.0f - timeFactor ) ) + omegaRoll * ( 1.0f - timeFactor ) );
	const auto rotHeading = tau * ( torque * ( timeStepFraction * float( dt ) - tau * ( 1.0f - timeFactor ) ) + omegaHeading * ( 1.0f - timeFactor ) );

	// To apply the roll, we need to rotate the heading to the new position
	// This code is rather inefficient, because RotationQuaternion normalizes rotHeading.
	// TODO: Make this more efficient if needed
	const auto angleHeading = Length( rotHeading );

	// There are issues with the quaternion creation if the heading rotation is small enough
	const bool applyHeading = angleHeading > 1e-10;

	// Need to roll after we have changed the heading
	const auto quatHeading = applyHeading ? RotationQuaternion( rotHeading, Length( rotHeading ) ) : Quaternion( 0.0f, 0.0f, 0.0f, 1.0f );

	const auto headingRotated = applyHeading ? TransformCoord( heading, RotationMatrix( quatHeading ) ) : heading;

	// Now we can update the angular velocity
	angvel = torque * ( tau * ( 1.0f - timeFactor ) ) + omegaHeading * timeFactor;
	angvel += headingRotated * ( tau * roll - ( tau * roll - omegaRoll ) * timeFactor );

	// And the rotation quaternion
	// The convention for Quaternions is opposite to most, a *= b applies b after a.
	// The a *= b operator is equivalent to a = a * b, which in most conventions applies a after b
	if( applyHeading )
		rotation *= quatHeading;
	rotation *= RotationQuaternion( headingRotated, rotRoll );

	// Normalize to be safe
	const auto norm = Length( rotation );
	if( norm > 0 )
		rotation /= norm;

	if( std::isnan( norm ) )
	{
		CCP_LOGERR_CH( s_chPark, "NaNs in rotation" );
	}
}

#pragma region Collisions


bool Ballpark::CalculateCollisionResponse(Ball* ball, size_t collisionIteration)
{
	if (!ball)
	{
		CCP_LOGWARN_CH(s_chPark, "CalculateCollisionResponses called with a null pointer for ball");
		return false;
	}

	// Find or create our collision responses
	auto collIter = mCollisionLocations.find(ball->mId);

	if (collIter == mCollisionLocations.end())
	{
		// Insert it and initialize
		collIter = mCollisionLocations.insert(collIter, std::pair<ID, CollisionBallProperties>(ball->mId, CollisionBallProperties()));
		collIter->second.Initialize(ball, dt);
	}

	auto& ballProperties = collIter->second;

	// Only need to call this on the first iteration
	if (collisionIteration == 0)
		mPartition->GetCollisionCandidates(ball, mPotentialCollisionBalls[ball->mId], mPotentialCollidables[ball->mId], isMaster);

	// Nothing to do if we already have a collision as early as possible
	if (ballProperties.m_tStart >= ballProperties.m_tEnd)
	{
		// Need to update the end
		ballProperties.CalculateEnd();
		return false;
	}

	// Also ignore any collision if we happen to have a collision at t=0
	// This time we don't have to update the end, because that happens above or below when the collision was added.
	if (ballProperties.m_locations.front().collider != -1)
		return false;

	auto& balls = mPotentialCollisionBalls[ball->mId];
	auto& collidables = mPotentialCollidables[ball->mId];

	// Only calculate if needed
	if (balls.size() == 0 && collidables.size() == 0)
		return false;

	// The collision for us and a possible neighbor
	CollisionItem collisionItem, neighborCollision;

	// Start with the static collisions, they are cheaper so better to have them calculated sooner
	GetNextCollisionStatic(ballProperties, collisionItem, balls, collidables);

	// Finish with the neighbor collisions, if needed
	if (collisionItem.timeOfImpact != ballProperties.m_tStart)
		GetNextCollisionFree(ballProperties, collisionItem, balls, collisionIteration);

	// Add the collisions to the collision structure, making sure we keep things in order.
	// In particular, we may need to delete collisions from prior free ball collisions

	// Nothing to do if there is no collision
	if (collisionItem.collider != nullptr)
	{
		// Static collisions have a different property
		if (collisionItem.mbIndex2 == -2)
		{
			CalculateCollisionImpactStatic(ballProperties, collisionItem);
		}
		else
		{
			// The other ball properties are already calculated, so safe to use [] access.
			neighborCollision = CalculateCollisionImpactFree(ballProperties, mCollisionLocations[collisionItem.collider->mId], collisionItem);
		}

		// Need to check for neighbor collisions first so we can return in the next if statement
		if (neighborCollision.impact != nullptr && neighborCollision.impact->collider != -1)
			AddImpact(*neighborCollision.impact, collisionIteration, dynamic_cast<Ball*>(neighborCollision.collider));

		// In rare cases we may have a positive collision calculation but no impact.
		if (collisionItem.impact != nullptr && collisionItem.impact->collider != -1)
		{
			AddImpact(*collisionItem.impact, collisionIteration, ball);

			// We need to update the end here, this ball has already calculated everything.
			ballProperties.CalculateEnd();
			return true;
		}
	}

	// We need to update the end here, this ball has already calculated everything.
	ballProperties.CalculateEnd();

	return false;
}

void Ballpark::GetNextCollisionStatic(CollisionBallProperties& ballProp, CollisionItem& collisionItem, VectorOfBalls& balls, VectorOfStaticCollidables& collidables)
{
	// A convenience shorthand
	auto* ball = ballProp.m_ball;

	// Store a static version of the miniballs, to reduce allocations
	static std::vector<Vector3d> mb_p0, mb_p1;
	ON_BLOCK_EXIT([]() { mb_p0.clear(); mb_p1.clear(); });

	Vector3d p0, p1;
	Vector3d vjunk;
	ballProp.CalculatePosition(p0, vjunk, ballProp.m_tStart);
	ballProp.CalculatePosition(p1, vjunk, ballProp.m_tEnd);

	// Start looping over static collideables, which apparently are not seen by missiles and mushrooms
	// Which is somewhat strange, because MiniBalls are seen by defender missiles, unless owned by same owner.
	// Not sure this matters in the game, but it is inconsistent
	if (ball->mMode != DSTBALL_MISSILE && ball->mMode != DSTBALL_MUSHROOM)
	{
		for (auto collidable : collidables)
		{
			Vector3d normal;
			double timeOfImpact(0.0);

			// Check for collision with the main ball
			bool collides = collidable->CheckCollision( p0, p1, ball->mRadius, normal, timeOfImpact );

			if (collides)
			{
				// Ignore if last collision was with same item and we have not moved.
				if (ballProp.IgnoreCollision(collidable->mId, timeOfImpact))
					continue;

				//Calculate the actual time of impact
				timeOfImpact = ballProp.m_tStart + (ballProp.m_tEnd - ballProp.m_tStart) * timeOfImpact;

				//Only add if sooner than previous
				if( collisionItem.timeOfImpact < 0.0 || timeOfImpact < collisionItem.timeOfImpact )
				{
					collisionItem.normal = normal;
					collisionItem.collider = collidable;
					collisionItem.timeOfImpact = timeOfImpact;
					collisionItem.mbIndex1 = -1;
					collisionItem.impact = nullptr;
				}
			}

			// Can break if we already found the first collision
			if( collisionItem.timeOfImpact == ballProp.m_tStart )
				break;

		}
	}

	// Now all the fixed balls, if needed
	if( collisionItem.timeOfImpact != ballProp.m_tStart )
	{
		for (auto neighbor : balls)
		{
			if (!neighbor->isFree)
			{
				// Don't include missile launchers and mushrooms.  Not sure if static objects can be owners of missiles.
				if (CollisionBallNeighborIgnore(ball, neighbor))
					continue;

				// Check for collision with the main ball
				double timeOfImpact = CollideTwoSpheres(p0, p1, neighbor->mNewPos, neighbor->mNewPos, static_cast<double>(ball->mRadius) + static_cast<double>(neighbor->mRadius));

				if (timeOfImpact >= 0.0)
				{
					// Ignore if last collision was with same item and we have not moved.
					if (ballProp.IgnoreCollision(neighbor->mId, timeOfImpact))
						continue;

					//Calculate the actual time of impact
					const double toi = ballProp.m_tStart + timeOfImpact * (ballProp.m_tEnd - ballProp.m_tStart);

					//Overwrite only if more recent or first
					if( collisionItem.timeOfImpact < 0.0 || timeOfImpact < collisionItem.timeOfImpact )
					{
						// Use linear approximation for the positions to get the normal
						const auto position = p0 + (p1 - p0) * timeOfImpact;
						collisionItem.normal = position - neighbor->mNewPos;
						collisionItem.normal.Normalize();

						collisionItem.collider = neighbor;
						collisionItem.timeOfImpact = toi;
						collisionItem.mbIndex1 = -1;
						collisionItem.impact = nullptr;
					}
				}
			}

			// Can break if we already found the first collision
			if( collisionItem.timeOfImpact == ballProp.m_tStart )
				break;
		}
	}
}

void Ballpark::GetNextCollisionFree(CollisionBallProperties& ballProp, CollisionItem& collisionItem, VectorOfBalls& balls, size_t collisionIteration)
{
	// For convenience
	auto* ball = ballProp.m_ball;

	// Store these vectors statically to reduce memory allocations
	static std::vector<double> times;
	static std::vector<Vector3d> ballMbP0, ballMbP1;
	static std::vector<Vector3d> otherMbP0, otherMbP1;

	// Positions for the main balls
	Vector3d ballP0, ballP1, otherP0, otherP1;
	Vector3d vjunk;

	ON_BLOCK_EXIT([]() {
		times.clear();
		ballMbP0.clear();
		ballMbP1.clear();
		otherMbP0.clear();
		otherMbP1.clear();
	});


	for (auto other : balls)
	{
		// Only do free balls with higher Id, unless we are a missile
		if (other->isFree && (other->mId > ball->mId || ball->mMode == DSTBALL_MISSILE))
		{
			// Don't include missile launchers and mushrooms.
			if (CollisionBallNeighborIgnore(ball, other))
				continue;

			// Get, or Initialize, the other balls properties
			auto iter = mCollisionLocations.find(other->mId);
			if (iter == mCollisionLocations.end())
			{
				iter = mCollisionLocations.insert(iter, std::pair<ID, CollisionBallProperties>(other->mId, CollisionBallProperties()));
				iter->second.Initialize(other, dt);
			}
			auto& otherProp = iter->second;

			// The end time here is the smaller of the end times of both balls
			double endTime = std::min(ballProp.m_tEnd, otherProp.m_tEnd);

			// We can continue if that is later than start time, which can happen
			if (ballProp.m_tStart >= endTime)
				continue;

			// We have to be extra careful that the other ball does not have later collisions already assigned.
			times.clear();
			otherMbP0.clear();
			otherMbP1.clear();

			// Loop through the locations and add the times that come after m_tStart, but before m_tEnd
			times.push_back(ballProp.m_tStart);
			for (auto& positions : otherProp.m_locations)
			{
				if (positions.time > ballProp.m_tStart && positions.time < endTime)
					times.push_back(positions.time);
			}
			times.push_back(endTime);

			// If we have more than just the start and the end, we need to invalidate the miniballs
			if (times.size() > 2 || endTime != ballProp.m_tEnd)
			{
				ballMbP0.clear();
				ballMbP1.clear();
			}

			// To save time, we only calculate one position in the loop
			ballProp.CalculatePosition(ballP0, vjunk, times[0]);
			otherProp.CalculatePosition(otherP0, vjunk, times[0]);

			for (size_t ii = 0; ii < times.size() - 1; ++ii)
			{
				double tstart = times[ii];
				double tend = times[ii + 1];

				ballProp.CalculatePosition(ballP1, vjunk, tend);
				otherProp.CalculatePosition(otherP1, vjunk, tend);

				double comparisonRadius = static_cast<double>(ball->mRadius) + static_cast<double>(other->mRadius);
				double timeOfImpact = CollideTwoSpheres(ballP0, ballP1, otherP0, otherP1, comparisonRadius);

				if (timeOfImpact >= 0.0)
				{
					// Keep track of added collision
					bool added = false;

					// Make sure the actual time of impact is relevant
					double toi = tstart + (tend - tstart) * timeOfImpact;

					if( collisionItem.timeOfImpact < 0 || toi < collisionItem.timeOfImpact )
					{
						// Ignore if we already have a collision with this ball at this time
						if (ballProp.IgnoreCollision(other->mId, timeOfImpact))
							continue;

						collisionItem.timeOfImpact = toi;
						collisionItem.collider = other;
						collisionItem.mbIndex1 = -1;
						collisionItem.mbIndex2 = -1;
						collisionItem.impact = nullptr;

						added = true;

						if (added)
						{
							// No need to search further with this ball
							break;
						}
					}
				}

				// Need to update the initial positions of both balls if there are substeps.
				if (times.size() > 2)
				{
					ballP0 = ballP1;
					otherP0 = otherP1;

					if (ballMbP1.size() > 0)
						ballMbP0 = ballMbP1;
					if (otherMbP1.size() > 0)
						otherMbP0 = otherMbP1;
				}

			}

			// Can break if we already found the first collision
			if( collisionItem.timeOfImpact == ballProp.m_tStart )
				break;

			// Need to invalidate the miniballs for our ball if there were substeps
			if (times.size() > 2 || endTime != ballProp.m_tEnd)
			{
				ballMbP0.clear();
				ballMbP1.clear();
			}

		}
	}
}

Vector3d Ballpark::ApproximateEllipsePointOfImpact(Ball* ball, BallLocation& ballLoc, const CollisionItem& collision)
{
	if( g_useDynamicalOrientation )
	{
		constexpr double fakeEllipticity = 0.3;

		// We need to find the angle between the outward normal and the rotation of the ship at
		// the point of impact

		// The cosine of the angle is the dot product of outward normal and ship direction.
		const auto toWorld = RotationMatrix( ballLoc.rot );
		auto shipDirection = Vector3d( TransformCoord( Vector3( 0.0f, 0.0f, 1.0 ), toWorld ) );
		const auto cosTheta = -collision.normal * shipDirection; // The normal points into ship, hence the minus sign.

		// This is the normal distance to point of impact, need to rotate it to fake ellipticity
		auto pointOfImpact = -collision.normal;

		if( std::fabs( cosTheta ) < 1.0 && std::fabs( cosTheta ) > 0.0 )
		{
			// Need also sinTheta
			const auto sinTheta = std::fabs( cosTheta ) < 1.0 ? std::sqrt( 1 - cosTheta * cosTheta ) : 0;

			// Calculate the sine of the angle of rotation
			const auto sinAlpha = 2 * fakeEllipticity * cosTheta * sinTheta /
				std::sqrt( ( std::pow( cosTheta, 2 ) + std::pow( ( 1 - fakeEllipticity ) * sinTheta, 2 ) ) *
						   ( std::pow( cosTheta, 2 ) + std::pow( ( 1 + fakeEllipticity ) * sinTheta, 2 ) ) );

			// Need to rotate the outward normal by alpha around the vector defined by the negative cross product
			// of outward normal and shipDirection.  Store it in shipDirection
			shipDirection.Cross( collision.normal ); // The normal actually points inwards.

			const auto alpha = std::asinf( static_cast<float>(sinAlpha) );
			const auto rotMat = RotationMatrix( shipDirection.AsVector3(), alpha );

			pointOfImpact = TransformCoord( pointOfImpact.AsVector3(), rotMat ) * ball->mRadius;

			// Reduce the radius to match the ellipse to reduce the impact a bit
			pointOfImpact *= ( 1 - fakeEllipticity ) * std::sqrt( 1. / ( sinTheta * sinTheta + ( 1 - fakeEllipticity ) * cosTheta * cosTheta ) );
		}

		return pointOfImpact;
	}

	// Cannot do anything special if we do not have the orientation, just use the collision normal times the radius
	return -collision.normal * ball->mRadius;
}

void Ballpark::CalculateCollisionImpactStatic(CollisionBallProperties& ballProp, CollisionItem& collision)
{

	// We only need to calculate the impact once and this only works for static collisions
	if (collision.impact == nullptr && collision.mbIndex2 == -2)
	{
		collision.impact.reset(new CollisionImpact());
		collision.impact->timeOfImpact = collision.timeOfImpact;

		// If we are already in collision at the start of the time step then turn on acceleration away from the object.
		if( ballProp.m_tStart == collision.timeOfImpact )
		{
			const auto anormal = ballProp.m_ball->mLastG * collision.normal;
			const auto maxAcceleration = ballProp.m_ball->mMaxVel * mFriction / (ballProp.m_ball->mMass * ballProp.m_ball->mAgility) - anormal;
			ballProp.m_ball->mLastC = maxAcceleration * collision.normal;
		}

		// Calculate the ball location to use for the impact
		BallLocation bloc;
		ballProp.CalculatePosition( bloc.p, bloc.v, collision.timeOfImpact );

		if( g_useDynamicalOrientation )
		{
			ballProp.CalculateRotation( bloc.rot, bloc.omega, collision.timeOfImpact );
		}

		// We have the normal, time of impact, and pre-calculated ball properties
		// Calculate the point of impact
		auto pointOfImpact = ApproximateEllipsePointOfImpact(ballProp.m_ball, bloc, collision);

		// We need the relative velocity at the point of impact, which is just our velocity, including rotation
		auto relVel = bloc.v;

		Vector3d vrot( bloc.omega );
		if( g_useDynamicalOrientation )
		{
			vrot.Cross( pointOfImpact );
			relVel += vrot;
		}

		// Calculate the relative velocity change at the point of impact, assuming perfectly elastic collision
		const double vImpulse = -2.0 * relVel * collision.normal;

		// If the impulse is negative, we have no collision.  This can happen if we are already colliding with the object
		// or if this is not the first collision in a series of simultaneous collisions.
		if (vImpulse < 0.0)
		{
			// The response in the impact is at 0, just return.
			return;
		}

		double impulse = vImpulse;
		if( g_useDynamicalOrientation )
		{
			Vector3d rcrossn = pointOfImpact;
			rcrossn.Cross( collision.normal );
			vrot = rcrossn;
			vrot.Cross( pointOfImpact );

			// Calculate impulse over mass (j/m), assuming I = 0.5*m*radius**2
			impulse = vImpulse / ( 1.0 + ( vrot * collision.normal ) / ( 0.5 * ballProp.m_ball->mRadius * ballProp.m_ball->mRadius ) );
			collision.impact->angularVelocityChange = rcrossn.AsVector3() * float( impulse / ( 0.5 * ballProp.m_ball->mRadius * ballProp.m_ball->mRadius ) );
		}

		// Now we can calculate the impact
		collision.impact->velocityChange = impulse * collision.normal;

		// Need to add the actual collider
		collision.impact->collider = collision.collider->mId;
	}
}

CollisionItem Ballpark::CalculateCollisionImpactFree(CollisionBallProperties& ballProp, CollisionBallProperties& otherProp, CollisionItem& collision)
{

	if( collision.impact != nullptr || collision.mbIndex2 == -2 )
	{
		// This should never be called with an impact already calculated
		return CollisionItem();
	}

	// Create the collision item for the other ball
	CollisionItem otherCollision;

	otherCollision.timeOfImpact = collision.timeOfImpact;
	// Need to store a pointer to the other ball here
	otherCollision.collider = collision.collider;
	otherCollision.mbIndex1 = collision.mbIndex2;
	otherCollision.mbIndex2 = collision.mbIndex1;

	// And the collision impacts
	otherCollision.impact.reset(new CollisionImpact());
	otherCollision.impact->timeOfImpact = collision.timeOfImpact;
	collision.impact.reset(new CollisionImpact());
	collision.impact->timeOfImpact = collision.timeOfImpact;

	// Calculate all the location information needed for both balls
	BallLocation ballLoc, otherLoc;

	ballProp.CalculatePosition( ballLoc.p, ballLoc.v, collision.timeOfImpact );
	otherProp.CalculatePosition( otherLoc.p, otherLoc.v, collision.timeOfImpact );

	if( g_useDynamicalOrientation )
	{
		ballProp.CalculateRotation( ballLoc.rot, ballLoc.omega, collision.timeOfImpact );
		otherProp.CalculateRotation( otherLoc.rot, otherLoc.omega, collision.timeOfImpact );
	}

	// Need to find the normal of the collision.  It points from the collided balls
	collision.normal = ballLoc.p - otherLoc.p;

	// Normalize the normal, making sure it actually is a direction
	const auto length = collision.normal.Length();
	if (length > 0.0)
		collision.normal /= length;
	else
		collision.normal.z = 1.0;  // This is a random direction

	otherCollision.normal = -collision.normal;

	// If we are already in collision, the timeOfImpact will be 0 and we turn on acceleration away from each other
	if( 0.0 == collision.timeOfImpact )
	{
		auto anormal = ballProp.m_ball->mLastG * collision.normal;
		auto maxAcceleration = ballProp.m_ball->mMaxVel * mFriction / (ballProp.m_ball->mMass * ballProp.m_ball->mAgility) - anormal;
		ballProp.m_ball->mLastC = maxAcceleration * collision.normal;

		anormal = otherProp.m_ball->mLastG * otherCollision.normal;
		maxAcceleration = otherProp.m_ball->mMaxVel * mFriction / (otherProp.m_ball->mMass * otherProp.m_ball->mAgility) - anormal;
		otherProp.m_ball->mLastC = maxAcceleration * otherCollision.normal;

	}

	// Now get the point of impact for both balls
	auto poi1 = ApproximateEllipsePointOfImpact(ballProp.m_ball, ballLoc, collision);
	auto poi2 = ApproximateEllipsePointOfImpact(otherProp.m_ball, otherLoc, otherCollision);

	Vector3d relVel;
	Vector3d vRot1( ballLoc.omega ), vRot2( otherLoc.omega );
	if( g_useDynamicalOrientation )
	{
		// The relative velocity, use 2-1 as in the wikipedia page
		vRot1.Cross( poi1 );
		vRot2.Cross( poi2 );
		relVel = ( otherLoc.v - ballLoc.v ) + ( vRot2 - vRot1 );
	}
	else
	{
		relVel = otherLoc.v - ballLoc.v;
	}

	// To get the sign correct, use the otherBall normal
	const double vImpulse = -2.0 * (relVel * otherCollision.normal);

	// A positive vImpulse is required for an actual collision
	if (vImpulse < 0.0)
	{
		// The impacts are both set at 0, just return
		return otherCollision;
	}

	Vector3d rCrossN1 = poi1;
	Vector3d rCrossN2 = poi2;
	if( g_useDynamicalOrientation )
	{
		rCrossN1.Cross( collision.normal );
		vRot1 = rCrossN1;
		vRot1.Cross( poi1 );

		rCrossN2.Cross( otherCollision.normal );
		vRot2 = rCrossN2;
		vRot2.Cross( poi2 );
	}

	// Calculate impulse, multiply with m1m2/m1m2 to make things pretty.
	// Assume I = 0.5 m R^2
	const double m1 = ballProp.m_ball->mMass;
	const double m2 = otherProp.m_ball->mMass;
	const double r1 = ballProp.m_ball->mRadius;
	const double r2 = otherProp.m_ball->mRadius;

	double impulse = 0;
	if( g_useDynamicalOrientation )
	{
		impulse = m1 * m2 * vImpulse / ( m1 + m2 + 2.0 * ( vRot1 * m2 / ( r1 * r1 ) - vRot2 * m1 / ( r2 * r2 ) ) * collision.normal );
	}
	else
	{
		impulse = m1 * m2 * vImpulse / ( m1 + m2 );
	}

	// Now we can calculate the impact
	collision.impact->velocityChange = (impulse / m1) * collision.normal;
	otherCollision.impact->velocityChange = (impulse / m2) * otherCollision.normal;

	if( g_useDynamicalOrientation )
	{
		collision.impact->angularVelocityChange = rCrossN1.AsVector3() * float( impulse / ( 0.5 * m1 * r1 * r1 ) );
		otherCollision.impact->angularVelocityChange = rCrossN2.AsVector3() * float( impulse / ( 0.5 * m2 * r2 * r2 ) );
	}

	collision.impact->collider = otherProp.m_ball->mId;
	otherCollision.impact->collider = ballProp.m_ball->mId;

	return otherCollision;
}

void Ballpark::AddImpact(CollisionImpact& impact, size_t collIter, Ball* ball)
{
	bool addImpact = false;

	// It is an error at this point not to have the item in the storage
	auto cIter = mCollisionLocations.find(ball->mId);

	if (cIter == mCollisionLocations.end())
	{
		CCP_LOGERR_CH(s_chPark, "Collision structure not found for %I64d when adding impacts", ball->mId);
		return;
	}

	// We should not add the impact unless it comes sooner than m_tEnd
	if (cIter->second.m_tEnd <= impact.timeOfImpact)
		return;

	static std::vector<std::pair<double, ID>> collisionsInvalidated;
	ON_BLOCK_EXIT([]() { collisionsInvalidated.clear(); });

	// Limit the impacts velocity and angular velocity change
	// 5 times the maximum for the velocity change
	const auto speed2 = impact.velocityChange.LengthSq();
	const auto fraction = 25 * ball->mMaxVel * ball->mMaxVel / speed2;
	if (fraction < 1)
	{
		impact.velocityChange *= fraction;
	}

	if(g_useDynamicalOrientation)
	{
		// Only twice for the angular velocity so ships don't spin many circles
		const auto omega2 = LengthSq( impact.angularVelocityChange );
		const auto frac = 4 * ball->mMaxAngVel * ball->mMaxAngVel / omega2;
		if( frac < 1 )
		{
			impact.angularVelocityChange *= frac;
		}
	}

	// Use the impact adder of the collision structure which returns all the impacts this action invalidates
	// It does not invalidate collsions in current iteration unless this one happens sooner.
	collisionsInvalidated = cIter->second.AddImpact(impact);

	// Use the recursive function remove from free to remove all relevant collisions
	for (const auto& pair : collisionsInvalidated)
	{
		auto ballIter = mFreeBalls.find(pair.second);
		if (ballIter != mFreeBalls.end())
		{
			RemoveImpactFromFree(pair.first, ball->mId, ballIter->second);
		}
	}

	// Loop over the impacts and flag out of order impacts.  THIS IS FOR DEBUGGING AND SHOULD BE REMOVED ONCE FIXED
	double prev(0), curr(0);
	for (const auto& loc : cIter->second.m_locations)
	{
		prev = curr;
		curr = loc.time;
		if (curr < prev)
		{
			CCP_LOGWARN_CH(s_chPark, "Adding impacts out of order! This should not happen.");
		}
	}
}


// Remove an impact from another free ball if necessary
// Returns true if iterators to mCollisions are invalidated
void Ballpark::RemoveImpactFromFree(double time, ID collider, Ball* ball)
{
	// Make sure we are a free ball
	if (!ball || !ball->isFree)
		return;

	// Find our structure, it is an error if it is not here
	auto c_iter = mCollisionLocations.find(ball->mId);

	if (c_iter == mCollisionLocations.end())
	{
		CCP_LOGERR_CH(s_chPark, "Collision structure not found for %I64d when removing impacts", ball->mId);
		return;
	}

	// Remove it, and all that are possibly invalidated
	auto invalidatedCollisions = c_iter->second.RemoveImpact(time, collider);

	for (const auto& pair : invalidatedCollisions)
	{
		auto ball_iter = mFreeBalls.find(pair.second);
		if (ball_iter != mFreeBalls.end())
		{
			RemoveImpactFromFree(pair.first, ball->mId, ball_iter->second);
		}
	}
}

inline bool Ballpark::CollisionBallNeighborIgnore(Ball* b, Ball* neighbor)
{
	// Capture the logic for ignoring collisions.  We loop twice to have fixed balls come before free balls.
	return (b->mMode == DSTBALL_MISSILE && (neighbor->mId == b->mOwnerId || (neighbor->mMode == DSTBALL_MINIBALL && neighbor->mOwnerId == b->mOwnerId))) // I am a missile, and the other guy is my daddy
		|| ((neighbor->mMode == DSTBALL_MISSILE || neighbor->mMode == DSTBALL_MUSHROOM) && b->mId == neighbor->mOwnerId) // The other guy is a missile and I am his daddy
		|| (neighbor->mMode == DSTBALL_MISSILE && b->mId == neighbor->mFollowId) // The other guy is a missile and I am his target
		|| ((neighbor->isSpaceJunk && !b->isSpaceJunk) || (!neighbor->isMassive));
}

//This function is not physically accurate, but it serves its purpose nicely
void Ballpark::CalculateBallRotationVelocity( Ball* ball, double timeStepFraction, Quaternion& rotation, Vector3& omega )
{
	if( !ball )
	{
		CCP_LOGERR_CH( s_chPark, "Null pointer for ball in CalculateBallRotationVelocity" );
		return;
	}

	// A quick solution if timeStepFraction is zero
	if( timeStepFraction == 0.0 )
		return;

	// The time factor
	const float tau = ( float( ball->mMass ) * ball->mAgility * ball->mRotAgility ) / float( mFriction );

	// Stuff is pre-calculated if the ball was already in last evolve function
	auto collisionIter = mCollisionLocations.find( ball->mId );

	if( collisionIter == mCollisionLocations.end() )
	{
		// No collisions, a single pass to the integrator will do
		ApplyTorque( rotation, omega, ball->mTorque, ball->mRoll, tau, static_cast<float>( timeStepFraction ) );
	}
	else
	{
		// Use the already pre-calculated points
		collisionIter->second.CalculateRotation( rotation, omega, timeStepFraction );
	}
}

void Ballpark::CalculateBallPositionVelocity(Ball* ball, double timeStepFraction, Vector3d& position, Vector3d& velocity)
{
	if (!ball)
	{
		CCP_LOGERR_CH(s_chPark, "Null pointer for ball in CalculateBallPositionVelocity");
		return;
	}

	// A quick solution if timeStepFraction is zero
	if (timeStepFraction == 0.0)
		return;

	// Stuff is pre-calculated if the ball was already in last evolve function
	auto collisionIter = mCollisionLocations.find(ball->mId);

	if (collisionIter == mCollisionLocations.end())
	{
		// No collisions, a single pass to the integrator will do
		Integrate(position, velocity, ball->mLastG + ball->mLastC, ball->mMass * ball->mAgility, mFriction, ball->mTimeFactor, timeStepFraction);
	}
	else
	{
		// Use the already pre-calculated points
		collisionIter->second.CalculatePosition(position, velocity, timeStepFraction);
	}
}

//---------------------------------------------------------------------------------------
// Gradient sums up influence of nearby balls on a single ball
//---------------------------------------------------------------------------------------
void Ballpark::Gradient(Ball *ball)
{
	if (!ball)
		return;

	// Keep static vectors for accumulating balls and collidables and clear it after we're done.
	// These vector will grow to the largest set of balls/collidables ever needed, which should
	// happen fairly quickly. After that, there won't be any memory allocations needed
	// and that is a worthwhile optimization.
	static VectorOfBalls s_uni;
	static VectorOfStaticCollidables s_staticCollidables;

	ON_BLOCK_EXIT( [=] { s_uni.clear(); s_staticCollidables.clear(); } );

	VectorOfBalls::iterator kt;
	VectorOfStaticCollidables::iterator st;
	Ball *neighbor;

	mPartition->GetCollisionCandidates(ball, s_uni, s_staticCollidables, isMaster);

	for (kt = s_uni.begin(); kt != s_uni.end(); ++kt)
	{
		neighbor = *kt;

		if ((ball->mMode == DSTBALL_MISSILE && (neighbor->mId == ball->mOwnerId || (neighbor->mMode == DSTBALL_MINIBALL && neighbor->mOwnerId == ball->mOwnerId))) // I am a missile, and the other guy is my daddy
			|| ((neighbor->mMode == DSTBALL_MISSILE || neighbor->mMode == DSTBALL_MUSHROOM) && ball->mId == neighbor->mOwnerId) // The other guy is a missile and I am his daddy
			|| (neighbor->mMode == DSTBALL_MISSILE && ball->mId == neighbor->mFollowId) // The other guy is a missile and I am his target
			|| (neighbor->isSpaceJunk && !ball->isSpaceJunk))
			continue; // Don't include missile launchers and mushrooms

		Potential(ball, neighbor);
	}

	for (st = s_staticCollidables.begin(); st != s_staticCollidables.end(); ++st)
	{
		if (ball->mMode == DSTBALL_MISSILE || ball->mMode == DSTBALL_MUSHROOM)
			continue; // Don't include missile launchers and mushrooms
		(*st)->CollideWithBall(ball);
	}
}

//---------------------------------------------------------------------------------------
// Potential calculates actual collision response between two balls
//---------------------------------------------------------------------------------------
void Ballpark::Potential(Ball *me, Ball *other, int recursionDepth)
{
	// First task is to estimate where I will be at next dt
	Vector3d p0, p1, q0, q1, vp1, vq1;
	double m1, m2;
	double collRadius = (double)me->mRadius + (double)other->mRadius;

	if (me->isFree)
		m1 = me->mMass * me->mAgility;
	else
		m1 = 1.0e34;

	if (other->isFree)
		m2 = other->mMass * other->mAgility;
	else
		m2 = 1.0e34;

	p0 = p1 = me->mNewPos;
	vp1 = me->mNewVel;

	q0 = q1 = other->mNewPos;
	vq1 = other->mNewVel;

	// Get the new point for me
	Integrate(p1, vp1, me->mLastG, m1, mFriction, me->mTimeFactor, dt);
	// Get the new point for the other guy
	Integrate(q1, vq1, other->mLastG, m2, mFriction, other->mTimeFactor, dt);

	// Calculate when collision occurs
	double s = CollideTwoSpheres(p0, p1, q0, q1, collRadius);
	if (s == -1.0)
	{
		return;
	}

	// Now 's' contains the collision time...

	//CCP_LOG_CH( s_chPark,"Collision in %f seconds between %I64d and %I64d\n",s*dt,me->mId,other->mId);

	// Estimate impact point
	p1 = me->mNewPos;
	vp1 = me->mNewVel;

	q1 = other->mNewPos;
	vq1 = other->mNewVel;

	//if(me->mId == mEgo)
	//    CCP_LOGWARN_CH( s_chPark,"[ %d ] Collision between ship %I64d and %I64d", mCurrentTime, me->mId, other->mId);
	// Now p1 and q1 are at the supposed impact point

	Vector3d a1, a2;

	if (s > 0.0)
	{
		// Get the new point for me
		Integrate(p1, vp1, me->mLastG, m1, mFriction, me->mTimeFactor, s*dt);
		// Get the new point for the other guy
		Integrate(q1, vq1, other->mLastG, m2, mFriction, other->mTimeFactor, s*dt);

		// Now we have the exact point of collision, and the velocity vectors there.
		Vector3d normal = (q1 - p1);
		normal.Normalize();

		double v1, v2, v1p, v2p;
		double eps = 1.0;

		// These are the radial components of velocity
		v1 = vp1 * normal;
		v2 = vq1 * normal;

		if (!other->isFree)
		{
			//CCP_LOG_CH( s_chPark,"[ %d ]Other ball %I64d is not free, i.e. %I64d hitting a fixed object", mCurrentTime, other->mId, me->mId);
			// This is the simple case of someone hitting a fixed object. Speed is thus inverted along the radial direction
			// Now given that. I would be situated at:
			vp1 -= 2.0*v1*normal;
			Integrate(p1, vp1, me->mLastG, m1, mFriction, me->mTimeFactor, (1.0 - s)*dt);

			// In order to get there, my acceleration needs to be:
			double k = mFriction;
			// This is really (me->mNewVel - vp1)*(k/m1/(me-mTimeFactor-1.0)) - me->mLastG. 
			a1 = -(-m1 * me->mLastG + me->mTimeFactor*m1*me->mLastG - me->mTimeFactor*me->mNewVel*k + vp1 * k) / m1 / (me->mTimeFactor - 1.0);
		}
		else
		{
			//CCP_LOG_CH( s_chPark,"[ %d ] Other ball %I64d is free",other->mId);
			double mm1 = me->mMass;
			double mm2 = other->mMass;
			// Use base mass when calculating collision response.
			v1p = (mm1*v1 - mm2 * v1 + 2.0*mm2*v2) / (mm1 + mm2);
			v2p = (mm2*v2 - mm1 * v2 + 2.0*mm1*v1) / (mm1 + mm2);

			vp1 += (v1p - v1)*normal;
			Integrate(p1, vp1, me->mLastG, m1, mFriction, me->mTimeFactor, (1.0 - s)*dt);

			double k = mFriction;
			a1 = -(-m1 * me->mLastG + me->mTimeFactor*m1*me->mLastG - me->mTimeFactor*me->mNewVel*k + vp1 * k) / m1 / (me->mTimeFactor - 1.0);
			//CCP_LOG_CH( s_chPark,"[ %d ] Free ball %I64d has calculated vp1 (%f, %f, %f) and a1 (%f, %f, %f) at time %f", mCurrentTime, me->mId, vp1.x, vp1.y, vp1.z, a1.x, a1.y, a1.z, s);
		}
	}
	else
	{
		//CCP_LOG_CH( s_chPark,"[ %d ] %I64d already in collision, need to get out of it, time is %f",mCurrentTime, me->mId,s);

		// This is a situtation where the balls are already in collision
		Vector3d normal;
		Vector3d p0q0 = q0 - p0;
		double p0q0_2 = p0q0 * p0q0;
		if (p0q0_2 == 0.0)
		{// I'm right into the other guy...choose an arbitary normal
			if (me->mId > other->mId)
				normal = Vector3d(1.0, 0.0, 0.0);
			else
				normal = Vector3d(-1.0, 0.0, 0.0);
		}
		else
		{
			normal = (q1 - p1);
			normal.Normalize();
		}
		// Calculate the distance I need to move to be in the clear
		// + 1 below is forcing extra distance to make sure that the recursed-with-values
		// actually takes us out of the collision, otherwise, we may end in infinite
		// recursion where the displacement doesn't progress at all over a tiny distance.
		double dist = collRadius - sqrt(p0q0_2) + 1;
		//CCP_LOG_CH( s_chPark,"[ %d ] %I64d need dist %.32f ",mCurrentTime, me->mId, dist);

		// distribute the distance amongst the two of us pro-rata of our mass
		double d1, d2;
		d1 = m2 / (m1 + m2)*dist;
		d2 = m1 / (m1 + m2)*dist;

		//CCP_LOG_CH( s_chPark,"[ %d ] %I64d distance distributed is d1 %.32f and d2 %.32f",mCurrentTime, me->mId, d1, d2);

		// if one of us is a fixed object, we just forcefully move them out of collision regardless of mass and other stuffz
		// However, if both of us are moveable, we use the recursive method.
		if (me->isFree && other->isFree)
		{
			//CCP_LOG_CH( s_chPark," %I64d both me and other are free; Using fake impact point ",me->mId);
			if (recursionDepth < 2) // Should never recurse more then once, but we allow two times :P
			{
				Vector3d tmpMe, tmpOther;

				// Save the position values, as we will restore them
				tmpMe = me->mNewPos;
				tmpOther = other->mNewPos;

				// forcibly move the balls out of collision along their normal
				me->mNewPos = me->mNewPos - d1 * normal;
				other->mNewPos = other->mNewPos + d2 * normal;

				// Now pretend that the collision actually occurs there, and get the collision response from there
				//CCP_LOG_CH( s_chPark,"[ %d ] Before recursing; me->LastC: (%f,%f,%f)\n", mCurrentTime, me->mLastC.x, me->mLastC.y, me->mLastC.z);
				Potential(me, other, recursionDepth + 1);
				//CCP_LOG_CH( s_chPark,"[ %d ] After recursing; me->LastC: (%f,%f,%f)\n", mCurrentTime, me->mLastC.x, me->mLastC.y, me->mLastC.z);
				// Restore the old positions
				me->mNewPos = tmpMe;
				other->mNewPos = tmpOther;

				if (m2 / m1 > 25.0)
				{    /* I.e. I'm the lighter party/will need to move much more then the other guy.
					We exagerate the acceleration of this guy to get him out quickly.  Otherwise the collision response
					 may fail to get us out of the other guy and by clicking a couple of times in space to change directions
					 we canfly right through him. This requires effort and will and is only possible with light agile ships.
					 To counter this I scale the acceleration vector to the distance we need to get out. This works
					 well for light small ships but is hillarious to watch if two titans are added to park at the same spot. */
					 //CCP_LOGWARN_CH( s_chPark,"[ %d ] Scaling collision response vector of %I64d: (%f,%f,%f)\n", mCurrentTime, me->mId, me->mLastC.x, me->mLastC.y, me->mLastC.z);
					me->mLastC = me->mLastC.Normalize() * dist;
					//CCP_LOGWARN_CH( s_chPark,"[ %d ] After scaling lastC we effectively made it: (%f,%f,%f)\n", mCurrentTime, me->mLastC.x, me->mLastC.y, me->mLastC.z);
				}

				if ((me->mNewVel - other->mNewVel).LengthSq() > 0.0001)
					return;
			}
			else
			{
				CCP_LOGERR_CH(s_chPark, "[ %d ] EEEEK! ball %I64d heading for infinite recursion into ball %I64d", mCurrentTime, me->mId, other->mId);
				// will fall through to use old forceful method.
			}

			CCP_LOG_CH(s_chPark, "[ %d ] %I64d did not return early.  acceleration for me (%f, %f, %f)", mCurrentTime, me->mId, a1.x, a1.y, a1.z);
			// ---------------------------------------------
		}

		double tmp;

		double normalComp = me->mLastG*normal;
		// Calculate the acceleration
		tmp = 1.0 / (m1 + dt * mFriction);
		a1 = ((-d1 / (dt*tmp*m1))*normal - me->mNewVel) / dt - normalComp * normal;
		//CCP_LOG_CH( s_chPark,"[ %d ] %I64d acceleration for me (%f, %f, %f)",mCurrentTime, me->mId, a1.x, a1.y, a1.z);
	}

	// Don't disrupt a missile's path if it is colliding with its target
	if ((me->mMode != DSTBALL_MISSILE || other->mId != me->mFollowId) && s >= me->mLastCollision)
	{
		Vector3d lastC = 0.85 * a1;
		if (s == me->mLastCollision)
		{
			//CCP_LOG_CH( s_chPark,"[ %d ] %I64d s %f equals last collision",mCurrentTime, me->mId, s);
			// Use the stronger collision of the two
			if (lastC.LengthSq() > me->mLastC.LengthSq())
			{
				me->mLastC = lastC;
				//CCP_LOG_CH( s_chPark,"[ %d ] %I64d Setting my lastC to (%f, %f, %f)",mCurrentTime, me->mId, me->mLastC.x, me->mLastC.y, me->mLastC.z);
			}
			//CCP_LOG_CH( s_chPark,"[ %d ] %I64d NOT resetting my lastC", mCurrentTime, me->mId);
		}
		else
		{
			me->mLastC = lastC;
			me->mLastCollision = s;
			//CCP_LOG_CH( s_chPark,"[ %d ] %I64d s is %f and we have set lastC to (%f, %f, %f)",mCurrentTime, me->mId, s, me->mLastC.x, me->mLastC.y, me->mLastC.z);
		}
	}

	me->mCollisions.push_back(other->mId);
}

#pragma endregion
#pragma endregion


#pragma region Ballpark start and pause
void Ballpark::Start()
{
    // If this is the first time we are called
    // then register for evolution ticks.
    if (!mHaveTicks)
    {
        BeOS->RegisterForTicks(this, (void*)TICK_EVOLVE);

        mHaveTicks = true;
    }
}

void Ballpark::Pause()
{
    if(mHaveTicks)
    {
        BeOS->UnregisterForTicks(this,(void*)TICK_EVOLVE);
        mHaveTicks = false;
    }
}
#pragma endregion


#pragma region Ego ball calculations
//---------------------------------------------------------------------------------------
// GetReferencePoint returns the absolute position of the ego ball
//---------------------------------------------------------------------------------------

Vector3d* Ballpark::GetReferencePoint(Vector3d* out, Be::Time time)
{
    if(time==mLastReferenceTime)
    {
        *out = mLastReference;
        return out;
    }

    ClientBall *ball = GetEgo();

    if(!ball)
    {
        *out = Vector3d(0.0,0.0,0.0);
        return out;
    }

    ball->InterpolatedPosition(&mLastReference,time);

    mLastDelta = mOldReference;
    mOldReference = mLastReference;
    mLastDelta = mLastReference - mLastDelta;

    mLastReferenceTime = time;
    *out = mLastReference;
    return out;
}

//---------------------------------------------------------------------------------------
// Delta returns the change of reference point since last time it was called
//---------------------------------------------------------------------------------------

void Ballpark::Delta(
    Vector3* ref,
    Vector3* sref,
    Be::Time time
    )
{
    ClientBall *ball = GetEgo();
    
    if(!ball)
    {
        Vector3 zero( 0.0f, 0.0f, 0.0f );
        *ref = *sref = zero;
        return;
    }
    Vector3d yada(0.0,0.0,0.0);
    ball->InterpolatedPosition(&yada, time);

    *sref = ball->mDeltaPos.AsVector3();
    *ref = ball->mDeltaPos.AsVector3();
}

void Ballpark::DeltaVel(
    Vector3* vel,
    Be::Time time
    )
{
    ClientBall *ball = GetEgo();

    if(!ball)
    {
        *vel = Vector3(0.0f,0.0f,0.0f);
        return;
    }

    Vector3d yada(0.0,0.0,0.0);
    ball->InterpolatedPosition(&yada, time);

    *vel = ball->mLastVel.AsVector3();
}
#pragma endregion


#pragma region Ball add/remove/insert
void Ballpark::InsertInBoxes(Partitionable *p)
{
	Box *box1, *top = nullptr;
	long newBubbleId = -1;

	// first determine in what box the center of the ball is. Use the distance it can travel in one time-step
	// as an inflated radius. Note than an inflated radius will always result in the same bubble as a non-inflated one.
	double inflatedRadius = p->GetInflatedRadius(dt);
	box1 = mPartition->GetBox(inflatedRadius, p->mNewPos);

	if(mProbe==p->mId)
		CCP_LOGWARN_CH( s_chPark,"[%d] Going into insert with [%d, %d]", mCurrentTime, p->mOldBubble, p->mNewBubble);

	if(isMaster)
	{
		top = mPartition->GetTopmost(box1);
		newBubbleId = top->mBubble;

		if(newBubbleId == -1)
		{
			if(mProbe==p->mId)
				CCP_LOGWARN_CH( s_chPark,"[%d] No bubble at new insert", mCurrentTime);

			// At this point, no matter what, this box will get assigned a bubble
			// In this case I need to remove the ball from its current box as it might be the only ball there
			// and would trigger an empty neighbor partition box.
			p->DeleteFromBoxes();
			p->mMyBox = 0;

			newBubbleId = top->NearbyBubbles();

			if(newBubbleId == -1)
			{
				// Still no bubble. Create a new one
				if(mProbe==p->mId)
					CCP_LOGWARN_CH( s_chPark,"[%d] No nearby one. Must create one: %d", mCurrentTime, mBubbleId);

				newBubbleId = mBubbleId++;
			}

			top->mBubble = newBubbleId;
		}

		if(mProbe==p->mId)
			CCP_LOGWARN_CH( s_chPark,"[%d] Insertion will be in: %d", mCurrentTime, newBubbleId);

		// The new box belongs to a bubble
		if(p->mNewBubble == newBubbleId)
		{
			// This is the same bubble that I am currently in, so shift bubble history
			if(mProbe==p->mId)
				CCP_LOGWARN_CH( s_chPark,"[%d] Ball already in that bubble. Shifting.", mCurrentTime);

			//ball->mOldBubble = ball->mNewBubble;
		}
		else
		{
			// This is a new bubble
			p->mOldBubble = p->mNewBubble;
			p->mNewBubble = newBubbleId;
			AddToBubble(p);
			// FLAG BUBBLE SWITCH ONLY ONCE PER BALL PER EVOLVE //
		}
	}

	// Let's now find which neighbors it intersects (if any)
	// Note that we inflate the radius of the ball by
	// the distance the ball could conceivably move within one time step and we only take boxes
	// in the direction of the velocity
	// This is to be sure to include nearby boxes when close to boundaries.
	p->InsertInBoxes(box1, top, newBubbleId);
	
}

void Ballpark::InsertBallInBoxes(Ball *ball)
{
	if(!ball)
		return;
	
	TASKLET(Timer_InsertBallInBoxes);

	
	Partitionable* p  = (Partitionable*) ball;
    InsertInBoxes(ball);

    // Add the ball to the map of balls
    if(!inEvolve){
        mBalls.Add(ball->mId, ball);
        if(ball->isGlobal)
            mGlobals.Add(ball->mId, ball);
    }
}


//---------------------------------------------------------------------------------------
// Ballpark::InsertBallInBox
//
// The given ball is added to the given box and inversely.
//
//---------------------------------------------------------------------------------------

void Ballpark::InsertInBox(Box *box,Partitionable *p)
{
    if(!p)
        return;

    p->InsertInBox(box);
}

void Ballpark::InsertStaticCollidableInBoxes(StaticCollidable* collidable)
{
	InsertInBoxes(collidable);

	if(!inEvolve){
		mStaticCollidables[collidable->mId] = collidable;
	}
}

//---------------------------------------------------------------------------------------
// Ballpark::GetMaxRadius
//
// Returns the radius of the largest ball, including correction
// for velocity.
//
//---------------------------------------------------------------------------------------

double Ballpark::GetMaxRadius(double start)
{
    Ball *b;
    double radius = start;
    BallIterator it(&mBalls);

    while((b = it++))
    {
        if((double)b->mRadius + dt*b->mMaxVel > radius)
            radius = (double)b->mRadius+dt*b->mMaxVel;
    }

    return radius;
}

//---------------------------------------------------------------------------------------
// BallExists returns pointer to ball with given id iff it exists
//---------------------------------------------------------------------------------------

Ball *Ballpark::BallExists(const ID& srcId)
{
    return mBalls[srcId];
}

ID Ballpark::GetIDForCollisionObject(ID objectId)
{
	ID theID;

	theID = objectId;
	if(isMaster)
	{
		// I am on a server, so the ID is mine to generate
		if(objectId <= 0)
		{
			// Generate a local ball
			if(objectId < 0)
			{
				// This is a pure local ball ID request
				theID = --mLocalHiCnt;
			}
			else
			{
				// This is supposed to be a client/server non-item ball
				theID = --mLocalCnt;
			}
		}
	}
	else
	{
		// I am on the client
		if(objectId == -1)
		{
			theID = --mLocalHiCnt;
		}
	}
	return theID;
}

//---------------------------------------------------------------------------------------
// AddBall adds a ball with the given parameters to the current simulation.
// Do not use this to add miniballs - use AddMiniball for that.
//---------------------------------------------------------------------------------------
Ball * Ballpark::AddBall(
    const ID& objectId,
    double mass,
    float radius,
    float maxVel,
    bool isFree,
    bool isGlobal,
    bool isMassive,
    bool isInteractive,
    bool isSpaceJunk,
    double x,
    double y,
    double z,
    double vx,
    double vy,
    double vz,
    float agility,
    float speedFraction
    )
{
    bool created = false;
    ID theID = GetIDForCollisionObject(objectId);

    //CCP_LOG_CH( s_chPark,"[ %d ] Issueing ball with ID: %I64d",mCurrentTime, theID);
    //CCP_LOG_CH( s_chPark,"[ %d ]  supplied arguments: mass %f, radius %f, maxVel %f", mCurrentTime, mass, radius, maxVel);
    //CCP_LOG_CH( s_chPark,"[ %d ]  isFree %d, isGlobal %d, isMassive %d, isInteractive%d, isSpaceJunk %d", mCurrentTime, isFree, isGlobal, isMassive, isInteractive, isSpaceJunk);
    //CCP_LOG_CH( s_chPark,"[ %d ]   x %f, y %f, z %f, vx %f, vy %f, vz %f, agility %f, speedFraction %.14f", mCurrentTime, x, y, z, vx, vy, vz, agility, speedFraction);
    // Check if we already have a ball with the given ID
    Ball *ball = mBalls[theID];

    if(!ball)
    {
        // This ball doesn't exist. Need to create a new one.
        if(isMaster)
        {
            ball = new OBall();
        }
        else
        {
            ball = new OClientBall();
        }

        ball->mId = theID;
        //CCP_LOG_CH( s_chPark, "Ball: %I64d created\n", mLocalHiCnt );
        created = true;
    }else{
        // This ball already exists.  Clear its boxes to be sure
        ball->DeleteFromBoxes();
        //CCP_LOG_CH( s_chPark, "Ball: %I64d updated\n", objectId );
    }

    // I now have a valid ball pointer. Put in values.

    ball->mNewPos.x      = x;
    ball->mNewPos.y      = y;
    ball->mNewPos.z      = z;

    ball->mNewVel.x      = vx;
    ball->mNewVel.y      = vy;
    ball->mNewVel.z      = vz;

    if(created)
    {
        ball->mOldPos = ball->mNewPos - dt*ball->mNewVel;

        ball->mOldVel.x      = vx;
        ball->mOldVel.y      = vy;
        ball->mOldVel.z      = vz;
    }
    else
    {
        // Some initializations
        ball->mFormationID   = -1;
        ball->mEffectStamp   = 0;
        ball->mOwnerId       = 0;
        ball->mFollowId      = 0;
        ball->mFollowRange   = 10.0;
        ball->mCorporationID = -1;
        ball->mAllianceID    = -1;
        ball->mHarmonic      = -1;
		if( ball->isMoribund )
		{
			ball->isMoribund = false;
			moribundBalls.erase( ball );
			CCP_LOGWARN_CH( s_chPark, "AddBall: Existing ball %I64d was moribund", objectId );
		}
    }

    ball->mRadius        = (radius < 0.0f ?  0.0f : radius);
    ball->mMass          = (mass   < 0.0f ?  0.0f : mass);

    ball->isFree         = isFree;

    ball->isGlobal       = isGlobal;
    ball->isMassive      = isMassive;
    ball->isInteractive  = isInteractive;
    ball->isSpaceJunk    = isSpaceJunk;
    ball->mMaxVel        = (maxVel < 0.0f ? 0.0f : maxVel);
    ball->mAgility       = (agility <= 0.0f ?  1.0f : agility);
    ball->mSpeedFraction = (speedFraction < 0.0f ? 0.0f : speedFraction);

    SetBallTimeFactor(ball);

    if (!created && !isFree)
    {
        // If we're replacing an existing ball, we need to ensure we don't leave the ball in the free list
        mFreeBalls.erase(theID);
    }
    else if(ball->isFree)
    {
        mFreeBalls.insert(DictEntry(ball->mId, ball));
    }

    // Stop is the default mode.
    ball->SetMode(DSTBALL_STOP);

    // If we're replacing an existing ball, we want to ensure that mFollowers/mFollowPtr are consistent
    if(!created && ball->mFollowers.size() > 0)
    {
        auto followersCopy = ball->mFollowers;
        auto follower = followersCopy.begin();
        auto end = followersCopy.end();

        for(;follower != end;++follower)
        {
			auto id = *follower;
			auto f = mBalls[id];
			if( !f )
			{
				CCP_LOGWARN_CH( s_chPark, "AddBall:%I64d has non-existing follower %I64d, removing", theID, id );
				ball->mFollowers.erase( id );
				continue;
			}

			if( f->mFollowPtr != ball )
            {
                CCP_LOGWARN_CH( s_chPark,"AddBall:%I64d has inconsistent follower %I64d, removing", theID, id );
                ball->mFollowers.erase( id );
            }
        }
    }

    // Give the ball a pointer to the ballpark
    ball->mPark = this;

    // Now insert ball into the space partition
    InsertBallInBoxes(ball);

    // Register for ticks

    if(created)
        ((IBall*)ball)->Unlock();

    //CCP_LOG_CH( s_chPark, "Ball: %I64d added at [%f, %f, %f] with mass %f & radius %f",ball->mId, ball->mNewPos.x, ball->mNewPos.y, ball->mNewPos.z, ball->mMass, ball->mRadius);
    //CCP_LOG_CH( s_chPark, "  isFree %d, isGlobal %d, isMassive %d, isInteractive %d, isSpaceJunk %d", ball->isFree, ball->isGlobal, ball->isMassive, ball->isInteractive, ball->isSpaceJunk);
    //CCP_LOG_CH( s_chPark, "  isCloaked %d, isMoribund %d, MaxVel, Agility, SpeedFraction", ball->isCloaked, ball->isMoribund, ball->mMaxVel, ball->mAgility, ball->mSpeedFraction);

    return ball;
}

//---------------------------------------------------------------------------------------
// AddUnOrientedBall adds a ball with the given parameters to the current simulation.
// Call this to add balls when Dynamical Orientation is disabled.
//---------------------------------------------------------------------------------------
Ball* Ballpark::AddOldStyleOrientedBall(
	const ID& objectId,
	double mass,
	float radius,
	float maxVel,
	bool isFree,
	bool isGlobal,
	bool isMassive,
	bool isInteractive,
	bool isSpaceJunk,
	double x,
	double y,
	double z,
	double vx,
	double vy,
	double vz,
	float agility,
	float speedFraction
)
{
	Ball* ball = AddBall( objectId, mass, radius, maxVel, isFree, isGlobal, isMassive, isInteractive, isSpaceJunk, x, y, z, vx, vy, vz, agility, speedFraction );
	if( ball->isFree )
	{
		// Snap ball orientation to velocity vector
		ball->CalculateYawPitchRoll( true );
	}
	return ball;
}

//---------------------------------------------------------------------------------------
// AddOrientedBall adds a ball with the given parameters to the current simulation.
// Call this to add balls when Dynamical Orientation is enabled.
//---------------------------------------------------------------------------------------
Ball* Ballpark::AddDynamicallyOrientedBall(
	const ID& objectId,
	double mass,
	float radius,
	float maxVel,
	float maxAngVel,
	bool isFree,
	bool isGlobal,
	bool isMassive,
	bool isInteractive,
	bool isSpaceJunk,
	double x,
	double y,
	double z,
	double vx,
	double vy,
	double vz,
	float rx,
	float ry,
	float rz,
	float rw,
	float wx,
	float wy,
	float wz,
	float agility,
	float rotAgility,
	float speedFraction )
{
	ID theID = GetIDForCollisionObject( objectId );
	const bool isNewBall = mBalls[theID] == nullptr;

	BallPtr ball = AddBall( objectId, mass, radius, maxVel, isFree, isGlobal, isMassive, isInteractive, isSpaceJunk, x, y, z, vx, vy, vz, agility, speedFraction );

	ball->mNewRot.x = rx;
	ball->mNewRot.y = ry;
	ball->mNewRot.z = rz;
	ball->mNewRot.w = rw;

	ball->mNewAngVel.x = wx;
	ball->mNewAngVel.y = wy;
	ball->mNewAngVel.z = wz;

	if( isNewBall )
	{
		ball->mOldRot = ball->mNewRot * RotationQuaternion( ball->mNewAngVel, -float( dt ) * Length( ball->mNewAngVel ) );

		ball->mOldAngVel.x = wx;
		ball->mOldAngVel.y = wy;
		ball->mOldAngVel.z = wz;
	}

	ball->mMaxAngVel = ( maxAngVel < 0.0f ? 0.0f : maxAngVel );
	ball->mRotAgility = ( rotAgility <= 0.0f ? 1.0f : rotAgility );
	return ball;
}

//---------------------------------------------------------------------------------------
// AddMiniball adds a miniball with the given parameters to the current simulation
//---------------------------------------------------------------------------------------
Ball * Ballpark::AddMiniball(
	const ID& parentObjectId,
	double mass,
	float radius,
	double x,
	double y,
	double z
	)
{
	ID theID = GetIDForCollisionObject(-1);
	Ball *ball = mBalls[theID];

	if (ball)
	{
		CCP_LOGERR_CH(s_chPark, "Ballpark.AddMiniball: Ball already exists for new ID %I64d. This should not happen!", theID);
	}

	if (isMaster)
	{
		ball = new OBall();
	}
	else
	{
		ball = new OClientBall();
	}
	ball->mId = theID;
    ball->mPark = this;

	ball->mNewPos.x = x;
	ball->mNewPos.y = y;
	ball->mNewPos.z = z;
	ball->mOldPos = ball->mNewPos;
    ball->mRadius = (radius < 0.0f ? 0.0f : radius);
    ball->mMass = (mass < 0.0f ? 0.0f : mass);
    ball->mOwnerId = parentObjectId;  // It is important this is set BEFORE calling InsertBallInBoxes (used for box-membership sorting)

	// Miniballs don't move!
	ball->mNewVel.x = 0.0;
	ball->mNewVel.y = 0.0;
	ball->mNewVel.z = 0.0;
	ball->mOldVel.x = 0.0;
	ball->mOldVel.y = 0.0;
	ball->mOldVel.z = 0.0;
	ball->mMaxVel = 0.0f;
	ball->mAgility = 0.0f;
	ball->mSpeedFraction = 0.0f;
    ball->isFree = 0;
    ball->isGlobal = 0;
    ball->isMassive = 1;
    ball->isInteractive = 0;
    ball->isSpaceJunk = 0;

	ball->mNewRot.x = 0.0f;
	ball->mNewRot.y = 0.0f;
	ball->mNewRot.z = 0.0f;
	ball->mNewRot.w = 1.0f;
	ball->mOldRot.x = 0.0f;
	ball->mOldRot.y = 0.0f;
	ball->mOldRot.z = 0.0f;
	ball->mOldRot.w = 1.0f;

	ball->mNewAngVel.x = 0.0f;
	ball->mNewAngVel.y = 0.0f;
	ball->mNewAngVel.z = 0.0f;
	ball->mOldAngVel.x = 0.0f;
	ball->mOldAngVel.y = 0.0f;
	ball->mOldAngVel.z = 0.0f;

	SetBallTimeFactor(ball);
    ball->SetMode(DSTBALL_MINIBALL);

	// Now insert ball into the space partition
	InsertBallInBoxes(ball);

	// Register for ticks
	((IBall*)ball)->Unlock();

	return ball;
}

Capsule * Ballpark::AddCapsule(
	const ID& objectId,
	const ID& parentObjectId,
	double ax,
	double ay,
	double az,
	double bx,
	double by,
	double bz,
	float radius)
{
	ID theID;
	bool created = false;

	theID = GetIDForCollisionObject(objectId);
	Capsule * capsule = nullptr;
	auto it = mCapsules.find(theID);
	if( it != mCapsules.end() )
	{
		capsule = it->second;
	}
	else
	{
		capsule = new OCapsule();
		capsule->Initialize(theID, parentObjectId, ax, ay, az, bx, by, bz, radius);
		mCapsules[theID] = capsule;
	}
	
	capsule->mPark = this;

	// Insert the capsule into the space partition
	InsertStaticCollidableInBoxes(capsule);

	return capsule;
}

OrientedBox * Ballpark::AddOrientedBox(
	const ID& objectId,
	const ID& parentObjectId,
	double corner_0, double corner_1, double corner_2,
	double x0, double x1, double x2,
	double y0, double y1, double y2,
	double z0, double z1, double z2 )
{
	ID theID;
	bool created = false;

	theID = GetIDForCollisionObject(objectId);
	OrientedBox * obb = nullptr;
	auto it = mOrientedBoxes.find(theID);
	if( it != mOrientedBoxes.end() )
	{
		obb = it->second;
	}
	else
	{
		obb = new OOrientedBox();
		obb->Initialize(
		    theID,
		    parentObjectId,
			corner_0, corner_1, corner_2,
			x0, x1, x2,
			y0, y1, y2,
			z0, z1, z2);
		mOrientedBoxes[theID] = obb;
	}

	obb->mPark = this;

	// Insert the capsule into the space partition
	InsertStaticCollidableInBoxes(obb);

	return obb;
}


//---------------------------------------------------------------------------------------
// AddMushroom adds an expanding ball in the ballpark, avoiding it parent
//---------------------------------------------------------------------------------------

void Ballpark::AddMushroom(
    const ID& ownerId,
    float range,
    double time
    )
{

    if(range <= 0.0 || time <= 0)
    {
        CCP_LOGWARN_CH( s_chPark,"Inconsistent parameters for mushroom. Canceled.\n");
        return;
    }

    Ball *owner = mBalls[ownerId];

    if(!owner)
    {
        CCP_LOGWARN_CH( s_chPark,"Non existing parent for mushroom. Canceled.\n");
        return;
    }
	Ball *mushroom = nullptr;

	if( g_useDynamicalOrientation )
	{
		mushroom = AddDynamicallyOrientedBall(
			0,
			1.0e20f,
			0.0f,
			0.0f,
			0.0f,
			false,
			false,
			true,
			false,
			false,
			owner->mNewPos.x,
			owner->mNewPos.y,
			owner->mNewPos.z,
			0.0, 0.0, 0.0,
			0.0f, 0.0f, 0.0f, 1.0f,
			0.0f, 0.0f, 0.0f,
			1.0f,
			1.0f,
			1.0f );
	}
	else
	{
		mushroom = AddOldStyleOrientedBall(
			0,
			1.0e20f,
			0.0f,
			0.0f,
			false,
			false,
			true,
			false,
			false,
			owner->mNewPos.x,
			owner->mNewPos.y,
			owner->mNewPos.z,
			0.0, 0.0, 0.0,
			1.0f,
			1.0f );
	}

    mushroom->mFollowRange = range;
    mushroom->mGoto[0] = time;
    mushroom->mOwnerId = ownerId;
    mushroom->mEffectStamp = mCurrentTime;
    mushroom->SetMode(DSTBALL_MUSHROOM);
}
#pragma endregion


#pragma region Ball mode-changing methods
//---------------------------------------------------------------------------------------
// MissileFollow make src ball follow dst ball without mutual collisions
//---------------------------------------------------------------------------------------

void Ballpark::MissileFollow(
    const ID& srcId,
    const ID& dstId,
    const ID& ownerId
    )
{
    if(srcId==dstId || srcId==ownerId || dstId==ownerId)
    {
        CCP_LOGWARN_CH( s_chPark,"Inconsistent ids for missiles. Canceled\n");
        return;
    }


    ID ownerIdCheck = ownerId;
    if (ownerIdCheck < 0)
        ownerIdCheck = -ownerIdCheck;
    Ball *owner = mBalls[ownerIdCheck];
    Ball *missile = mBalls[srcId];
    Ball *target = mBalls[dstId];

    if(!missile)
    {
        CCP_LOGWARN_CH( s_chPark,"Non existing missile: %I64d (target: %I64d). Canceled\n", srcId, dstId);
        return;
    }

    if (!target)
    {
        CCP_LOGWARN_CH( s_chPark,"Non existing target (%I64d) for missile: %I64d. Canceled\n", dstId, srcId);
        return;
    }

    if(target->isMoribund)
    {
        CCP_LOG_CH( s_chPark,"Ball:%I64d trying to missile-follow moribund ball:%I64d. Cancelled.\n", srcId, dstId);
        return; // Can't target dead balls
    }

    if(!owner)
    {
        CCP_LOGWARN_CH( s_chPark,"Non existing missile owner. Tolerated.\n");
    }

    // stop whatever the missile was doing
    Stop(missile);

    if(missile->mNewBubble != target->mNewBubble)
    {
        CCP_LOGWARN_CH( s_chPark,"Missile:%I64d and target:%I64d in different bubbles. Canceled.\n",srcId, dstId);
        return;
    }

    if(target->isCloaked)
    {
        CCP_LOGWARN_CH( s_chPark,"Missile:%I64d targeting cloaked target:%I64d. Canceled.\n",srcId, dstId);
        return;
    }

    // Now start following the dstId ball
    missile->mFollowId = dstId;
    missile->mFollowPtr = target;

    // The follow range is made negative so the missile actually attempts to go to the center of the target, not its surface
    missile->mFollowRange = -(missile->mRadius + target->mRadius);
    missile->mOwnerId = ownerId;

    // The effect stamp is used to make the missile go in a straight launch direction for a few timesteps before actually following
    missile->mEffectStamp = mCurrentTime;
    Vector3d velocity = missile->mNewVel;
    missile->mGoto = velocity.Normalize()*1.e16;
    missile->mSpeedFraction = 1.0;
    missile->SetMode(DSTBALL_MISSILE);

    // Declare this ball a follower
    target->mFollowers.insert(missile->mId);
}

//---------------------------------------------------------------------------------------
// FollowBall make src ball follow dst ball within a given range
//---------------------------------------------------------------------------------------

void Ballpark::FollowBall(
    const ID& srcId,
    const ID& dstId,
    float range
    )
{
    if(!_finite(range) )
    {
        CCP_LOGWARN_CH( s_chPark,"NaN in FollowBall. Ignored");
        return;
    }

    Ball *ball = mBalls[srcId];

    if(!ball)
        return;


    if(srcId == dstId)
    {
        return;
    }

    Ball *dest = mBalls[dstId];
    if(!dest)
	{
		CCP_LOGWARN_CH(s_chPark, "Ball:%I64d trying to follow non-existent ball:%I64d. Cancelled.\n", srcId, dstId);
		return;
	}

    if(dest->isMoribund)
    {
        CCP_LOG_CH( s_chPark,"Ball:%I64d trying to follow moribund ball:%I64d. Cancelled.\n", srcId, dstId);
        return; // Can't follow dead balls
    }

    if(ball->mNewBubble != dest->mNewBubble)
    {
        CCP_LOGWARN_CH( s_chPark,"Ball:%I64d trying to follow ball:%I64d in another bubble. Cancelled.\n",srcId, dstId);
        return;
    }

    if(dest->isCloaked)
    {
        CCP_LOGWARN_CH( s_chPark,"Ball:%I64d trying to follow cloaked ball:%I64d. Cancelled.\n",srcId, dstId);
        return;
    }

	// the source ball exists
	// stop whatever it was doing
	Stop(ball);

	// Now start following the dstId ball
    ball->mFollowId = dstId;
    ball->mFollowPtr = dest;
    ball->mFollowRange = range;
    ball->SetMode(DSTBALL_FOLLOW);

    // Declare this ball a follower
    auto res = dest->mFollowers.insert(ball->mId);
    if(!res.second)
        CCP_LOGERR_CH( s_chPark,"ball %I64d already flagged as follower of %I64d", ball->mId, dest->mId);
}

//---------------------------------------------------------------------------------------
// FollowBall make src ball follow dst ball within a given range
//---------------------------------------------------------------------------------------

void Ballpark::FormationFollow(
    const ID& srcId,
    const ID& dstId,
    char slot
    )
{

    Ball *ball = mBalls[srcId];

    if(!ball)
        return;


    if(srcId == dstId)
    {
        return;
    }

    Ball *dest = mBalls[dstId];
    if(!dest)
        return;

    if(dest->mFormationID < 0)
        return; // Destination is not a formation leader

    if(dest->mFormationID > int(mFormations.size())-1)
        return; // Illegal formation ID

    if(slot > int(mFormations[dest->mFormationID].size())-1)
        return; // Illegal slot

    // stop whatever it was doing
    Stop(ball);

    if(ball->mNewBubble != dest->mNewBubble)
    {
        CCP_LOGWARN_CH( s_chPark,"Ball:%I64d trying to formation follow ball:%I64d in another bubble. Cancelled.\n",srcId, dstId);
        return;
    }

    if(dest->isCloaked)
    {
        CCP_LOGWARN_CH( s_chPark,"Ball:%I64d trying to formation follow cloaked ball:%I64d. Cancelled.\n",srcId, dstId);
        return;
    }

    // Now start following the dstId ball
    ball->mFollowId = dstId;
    ball->mFollowPtr = dest;
    ball->mEffectStamp = slot;
    ball->SetMode(DSTBALL_FORMATION);

    // Declare this ball a follower
    dest->mFollowers.insert(ball->mId);
}

//---------------------------------------------------------------------------------------
// Orbit makes src ball orbit dst ball within some given range
//---------------------------------------------------------------------------------------

void Ballpark::Orbit(
    const ID& srcId,
    const ID& dstId,
    float range
    )
{
    if(!_finite(range) )
    {
        CCP_LOGWARN_CH( s_chPark,"NaN in Orbit. Ignored");
        return;
    }

    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    if(srcId == dstId)
    {
        return;
    }

    Ball *dest = mBalls[dstId];
    if(!dest)
	{
		CCP_LOGWARN_CH(s_chPark, "Ball:%I64d trying to orbit non-existent ball:%I64d. Cancelled.\n", srcId, dstId);
		return;
	}

    if(dest->isMoribund)
    {
        CCP_LOG_CH( s_chPark,"Ball:%I64d trying to orbit moribund ball:%I64d. Cancelled.\n", srcId, dstId);
        return; // Can't orbit dead balls
    }

    if(ball->mNewBubble != dest->mNewBubble)
    {
		CCP_LOGWARN_CH(s_chPark, "Ball:%I64d trying to orbit Ball:%I64d in another bubble. Cancelled.\n", srcId, dstId);
        return;
    }

    if(dest->isCloaked)
    {
		CCP_LOGWARN_CH(s_chPark, "Ball:%I64d trying to orbit cloaked Ball:%I64d. Cancelled.\n", srcId, dstId);
        return;
    }

	// the source ball exists
	// stop whatever it was doing
	Stop(ball);

	// Now start following the dstId ball
    ball->mFollowId = dstId;
    ball->mFollowPtr = dest;
    ball->mFollowRange = range;
    ball->SetMode(DSTBALL_ORBIT);

    // Declare this ball a follower
    dest->mFollowers.insert(ball->mId);
}

//---------------------------------------------------------------------------------------
// WarpTo make ball src go to specified coordinate very fast
//---------------------------------------------------------------------------------------

void Ballpark::WarpTo(
    const ID& srcId,
    double x,
    double y,
    double z,
    double minRange,
    int warpFactor
    )
{
    if(!_finite(x) || !_finite(y) || !_finite(z))
    {
        CCP_LOGWARN_CH( s_chPark,"NaN in WarpTo. Ignored");
        return;
    }

    if(minRange < 0.0)
        minRange = 0.0;

    if(warpFactor <= 0)
        warpFactor = 1;

    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    Vector3d dst(x,y,z);

    // Actually offset the goto point on a 20 km radius from the point

    Vector3d delta = (ball->mNewPos-dst);

    if(delta.LengthSq() < 10000000000.0)
    {
        // Distance too small. Do a goto point instead
        GotoPoint(ball,Vector3d(x,y,z));
        if (!PyOS->PostEvent((IEveBallpark*)this, "Destiny::OnExitWarp", "OnExitWarp", "Li", ball->mId, 1))
        {
            PyOS->PyError();
        }
        return;
    }

    if (ball->IsWarpish() && dst == ball->mGoto)
    {
        //CCP_LOG_CH(s_chPark, "WarpTo: redundant command; already trying to warp to the specified spot.");
        return; // no-op really, and since we don't want to reset the align counter, let's do nothing
    }

    // Stop ball from doing whatever it was doing
    Stop(ball);
    // Use persistent info about the warp
    ball->mGoto = dst;
    ball->mEffectStamp = -1;
    ball->mFollowRange =  0.0;

    // Note that we shanghai the followId member to keep a double (casting it to 64bit int)
    double tmp = minRange;
    ball->mFollowId = *((int64_t *)&tmp);

    // And we also shanghai the ownerId member to keep the desired warp speed
    ball->mOwnerId = warpFactor;

    ball->SetMode(DSTBALL_WARP);
    //CCP_LOGWARN_CH( s_chPark,"Starting warp for %f", dst.Length());
}

#pragma endregion


#pragma region Warp
void Ballpark::RealWarp(Ball *ball)
{
    if(!ball)
        return;

    StopAllFollowers(ball);
    Vector3d dst = ball->mGoto;

    // Actually offset the goto point on a 20 km radius from the point

    Vector3d delta = (ball->mNewPos-dst);
    delta.Normalize();
    dst = dst + *((double *)&ball->mFollowId)*delta;
    delta = (dst - ball->mNewPos);

    // Use persistent info about the warp
    ball->mGoto = dst;
    ball->mEffectStamp = mCurrentTime;
    ball->mLastCollision = delta.Length(); // mLastCollision repurposed for the total warp length (you're not collidable whilst in warp)
    ball->isMassive = false;
    ball->mSensor.active = false;
    //CCP_LOGWARN_CH( s_chPark,"Real warp started at: %d and for distance:%f", mCurrentTime, dist);
}

void Ballpark::SetupWarpConstants(
        double warpFactor,
        double warpDistance,
        double &accelDuration,
        double &cruiseDuration,
        double &decelDuration,
        double &accelDistance,
        double &cruiseDistance,
        double &decelDistance,
        double &accelRate,
        double &decelRate,
        double &warpSpeed
        )
{
    // For the sake of future-me (or present-you), here's a guide to the equations used below
    // ACC: accel rate
    // DEC: decel rate
    // S: max warp speed
    // D: total distance to warp

    // ACCEL PHASE:
    // Exponential growth starting at time 0 until time tA
    // Start with x=0 and v=0 at t=0
    //     x = exp(ACC * t)                   [1]  distance across time during acceleration phase
    //     v = d/dt [x]
    //     v = ACC * exp(ACC * t)             [2]  speed across time during acceleration phase
    // Acceleration phase ends when v = s at time tA, so substitute S=v, t=tA in to [2]:
    //     S = ACC * exp(ACC * tA)
    // Solve for tA:
    //     tA = ln(S / ACC) / ACC             [3]  duration of acceleration phase
    // Acceleration phase ends when t = tA at distance dA, so substitute t=tA, x=dA and [3] in to [1]:
    //     dA = S / ACC                       [4]  distance travelled in acceleration

    // DECEL PHASE: (This is mostly the reverse of ACCEL)
    // Exponential decay starting at time 0 until time tD
    // Start with x=0 and v=S at t=0
    //     v = DEC * exp(DEC * (tD - t))      [5]
    // Deceleration phase beging with v=S, so substitute S=v, t=0 in to [5]
    //     S = DEC * exp(DEC * tD)
    // Solve for tD
    //     tD = ln(S / DEC) / DEC             [6]  duration of deceleration phase
    // Substitute [2] in to [1]
    //     v = S * exp(-DEC * t)              [7]  speed across time during deceleration phase
    // Integrate [3] wrt t
    //     x = -S / DEC * exp(-DEC * t) + c
    // When t0, x=0, so C = S/DEC
    //     x = S / DEC - S / DEC * exp(-DEC * t)   [8] distance across time during deceleration phase
    // Deceleration phase ends when t = tD at distance dD, so substitute t=tD, x=dD and [6] in to [8]:
    //     dD = S / DEC - 1                   [8]  distance travelled in deceleration

    // CRUISE PHASE
    // This phase is movement at constant speed, to cover whatever distance is not covered during acceleration and deceleration
    //     v = S                              [9]  speed across time during cruise phase
    //     x = St                             [10] distance across time during cruise phase
    //     D = dA + dC + dD
    // Substitute [4] and [8], and solve for dC
    //     dC = D - S / ACC - S / DEC + 1     [11] distance travelled during cruise
    //     tC = dC / v
    //     tC = D / S - 1 / ACC - 1 / DEC + 1 / S    [12] duration of cruise phase

    // We have a constraint that tC can never be negative (otherwise the ship would be begin decelerating before it finished accelerating)
    //     tC > 0
    //     D / S - 1 / ACC - 1 / DEC + 1 / S > 0
    // Solving for S gives
    //     S < (D + 1) * ACC * DEC / (ACC + DEC)
    // We enforce this constraint by lowering S if it would ever be violated, until the cruise phase duration becomes 0


    // WarpFactor controls how quickly the ship can warp.
    // It affects three attributes of the warp:
    //  The maximum speed attainable (See warpSpeedToAUPerSecond in appConst.py)
    warpSpeed = warpFactor * WARP_FACTOR_TO_AU_PER_SECOND * AU; // Convert to m/s units
    //  The rate of acceleration
    accelRate = warpFactor * WARP_FACTOR_TO_ACCELERATION;
    //  The rate of deceleration
    //  (Constrain this to maximum of 2 to prevent fast ships from insta-decelerating)
    decelRate = std::min(warpFactor * WARP_FACTOR_TO_DECELERATION, 2.0);

    // A warp is split in to three phases: Acceleration, Cruise, Deceleration
    // If the warp is short enough that the ship would overshoot the destination before it reached top speed,
    // then we reduce the top speed accordingly so that the acceleration+deceleration phases alone are exactly enough to complete the distance
    // (and so the cruise phase effectively takes 0 seconds)
    warpSpeed = std::min(warpSpeed, (warpDistance + 1) * accelRate * decelRate / (accelRate + decelRate));

    // Calculate the time spent and distance travelled in each phase
    accelDuration = log(warpSpeed / accelRate) / accelRate;
    cruiseDuration = (warpDistance / warpSpeed ) - (1.0 / accelRate) - (1.0 / decelRate) + (1.0 / warpSpeed);
    decelDuration = log(warpSpeed / decelRate) / decelRate;
    accelDistance = warpSpeed / accelRate;
    cruiseDistance = warpSpeed * cruiseDuration;
    decelDistance = warpSpeed / decelRate - 1;
}

double Ballpark::WarpDistance(Ball *ball, Vector3d& p, Vector3d& v, double t, bool interpolating)
{
    TASKLET(Timer_WarpDistance);

    double warpFactor = (double)ball->mOwnerId;
    double warpDistance = ball->mLastCollision;
    
    double accelDuration;
    double cruiseDuration;
    double decelDuration;
    double accelDistance;
    double cruiseDistance;
    double decelDistance;
    double accelRate;
    double decelRate;
    double warpSpeed;
    SetupWarpConstants(warpFactor, warpDistance, accelDuration, cruiseDuration, decelDuration, accelDistance, cruiseDistance, decelDistance, accelRate, decelRate, warpSpeed);
    // TODO: (Possibly) Calculate these constants once at entry to warp, and re-use for the duration.
    //       BUT this will require special handling for new balls that the client encounters whilst they are mid-warp.

    // Depending on which phase of the warp this ball is in, calculate the 1D distance & speed,
    // then map these back on to the 3D position and velocity ball attributes (p and v).
    double speed;
    double distance;
    Vector3d dir = (ball->mGoto - p);
    dir.Normalize();
    if (t < accelDuration)
    {
        // Acceleration phase
        speed = accelRate * exp(accelRate * t);
		double currSpeed = v.Length();
		if (currSpeed > speed)
			// Faking the speed to be continuous even though it actually isn't
			speed = currSpeed;

        v = speed * dir;

        distance = exp(accelRate * t);
        p = ball->mGoto - (accelDistance + cruiseDistance + decelDistance - distance) * dir;
    }
    else if ((t - accelDuration) < cruiseDuration)
    {
        // Cruise phase
        speed = warpSpeed;
        v = speed * dir;

        distance = warpSpeed * (t - accelDuration) + accelDistance;
        p = ball->mGoto - (accelDistance + cruiseDistance + decelDistance - distance) * dir;
    }
    else if ((t - cruiseDuration - accelDuration) < (decelDuration + 1))
    // Add 1 to ensure we always allow the final tick at the end of the warp to complete before hitting the warning in the final 'else'
    {
        // Deceleration phase
        speed = warpSpeed * exp(-decelRate * (t - cruiseDuration - accelDuration));
        v = speed * dir;

        distance = warpSpeed / decelRate - (warpSpeed / decelRate) * exp(-decelRate * (t - cruiseDuration - accelDuration)) + accelDistance + cruiseDistance;
        p = ball->mGoto - (accelDistance + cruiseDistance + decelDistance - distance) * dir;

        // Once the speed drops below 50% of regular max-velocity and 100m/s, the ship drops out of warp
        // (This ensures that very fast ships still spend a bit of time in the end-warp phase before they become active/targetable)
        if (speed < std::min(ball->mMaxVel / 2.0, 100.0) && !interpolating)
        {
            if (!isMaster && !PyOS->PostEvent(
                (IEveBallpark*)this, "Destiny::OnDeactivatingWarp",
                "OnDeactivatingWarp", "Li", ball->mId, mCurrentTime
                ))
            {
                PyOS->PyError();
            }
            ball->isMassive = true;
            ball->mSensor.active = true;
            Stop(ball);
        }
    }
    else
    {
        // Ending up in this state implies the warp has been going on for too long, and so shouldn't be possible if the math above is correct.
        // If this ever happens, log some error info, and put the ship at the exit point with 0 velocity
        CCP_LOGERR_CH(s_chPark, "Ship stuck in extended warp %I64d: warpFactor:%f warpDistance:%f time:%f interpolating:%d", ball->mId, warpFactor, warpDistance, t, interpolating);
        CCP_LOGERR_CH(s_chPark, "Goto: %f %f %f", ball->mGoto.x, ball->mGoto.y, ball->mGoto.z);

        p = ball->mGoto;
        v = Vector3d(0, 0, 0);
        distance = 0;
        
        if (!interpolating)
        {
            if (!isMaster && !PyOS->PostEvent(
                (IEveBallpark*)this, "Destiny::OnDeactivatingWarp",
                "OnDeactivatingWarp", "Li", ball->mId, mCurrentTime
                ))
            {
                PyOS->PyError();
            }
            ball->isMassive = true;
            ball->mSensor.active = true;
            Stop(ball);
        }
    }

    return distance;
}

/*
    Pretty much sets srcId at max velocity, in effect faking that it has
    already done all aligning and acceleration stuff and can go directly
    into doing warpDistance.
*/

void Ballpark::EntityWarpIn(const ID& srcId, double x, double y, double z, int warpFactor)
{
    CCP_LOG_CH( s_chPark,"EntityWarpIn getting called for %I64d to arrive at %.12f, %.12f, %.12f", srcId, x,y,z);

    // Allow WarpTo to do it's thang.
    WarpTo(srcId, x,y,z, 0.0, warpFactor);

    Ball *ball = mBalls[srcId];
    if(!ball)
        return;

    if( ball->mMode == DSTBALL_GOTO)
        return ; // should never happen really, but...

    // Allow RealWarp to do it's thing.
    RealWarp(ball);

    //CCP_LOG_CH( s_chPark,"EntityWarpIn: newPos of ball is at %.12f, %.12f, %.12f", ball->mNewPos.x, ball->mNewPos.y, ball->mNewPos.z);
    
    // Fake us already having aligned (don't care about rotation/ypr)
    Vector3d dst(x,y,z);
    ball->mNewVel = dst.Normalize() * AU ; // start at ~1AU velocity
	if( g_useDynamicalOrientation )
	{
		ball->SnapOrientation();
	}
    ball->mEffectStamp = std::max(mCurrentTime-5, 0l); // fake that we started warping some ticks ago
    dst = ball->mNewPos - ball->mGoto; // reusing dst
    ball->mLastCollision = dst.Length();  // mLastCollision repurposed for the total warp length (you're not collidable whilst in warp)
    //CCP_LOG_CH( s_chPark,"EntityWarpIn: The length between goto and pos is %.12f km", ball->mLastCollision/1000.0);

    //CCP_LOG_CH( s_chPark,"About to activate warp %I64d",ball->mId);
    if (isMaster && !PyOS->PostEvent(
        (IEveBallpark*)this, "Destiny::OnActivatingWarp",
        "OnActivatingWarp", "Li", ball->mId, mCurrentTime))
    {
        PyOS->PyError();
    }
    
    CCP_LOG_CH( s_chPark,"EntityWarpIn: All done, balls velocity is %.12f, %.12f, %.12f, length %.5f", ball->mNewVel.x, ball->mNewVel.y, ball->mNewVel.z, ball->mNewVel.Length());
}

// This function is used on the client to adjust for sim clock adjustments
// No idea what else you'd ever use it for, but if you know what you're doing, have a grand time I guess.
//
void Ballpark::AdjustTimes(Be::Time timeDelta)
{
    // First up, my own time
    mTime += timeDelta;

    // Now the same for all of my balls
    BallIterator it(&mBalls);
    Ball* iterBall;
    while((iterBall = it++))
    {
        if (iterBall->ClassType() == ClientBall::ClassType_())
        {
            ClientBall* cliBall = (ClientBall *) iterBall;
            cliBall->mPosUpdateTime += timeDelta;
            cliBall->mRotUpdateTime += timeDelta;
        }
        if (iterBall->mNewTime != 0)
        {
            iterBall->mNewTime += timeDelta;
        }
        if (iterBall->mOldTime != 0)
        {
            iterBall->mOldTime += timeDelta;
        }
    }
}


void Ballpark::ShiftFromPlanets(Vector3d& p)
{
    BallIterator it(&mBalls);
    Ball* sphere;

    while((sphere = it++))
    {
        if(!sphere->isGlobal)
            continue;
        double r = sphere->mRadius;
        if( (p-sphere->mNewPos).LengthSq() < 1.1*1.1*r*r)
        {
            // We are within a planet. Shift point gracefully outside
             p -= sphere->mNewPos;
             p.Normalize();
             p = 1.1*r*p + sphere->mNewPos;
             break;
        }
    }
}
#pragma endregion


#pragma region More ball mode-changing methods
//---------------------------------------------------------------------------------------
// GotoDirection makes ball src go in the specified direction
//---------------------------------------------------------------------------------------

void Ballpark::GotoDirection(
    const ID& srcId,
    double x,
    double y,
    double z
    )
{
    Ball *ball = mBalls[srcId];
    GotoDirection(ball,Vector3d(x,y,z));
}


void Ballpark::GotoDirection(
    Ball *ball,
    Vector3d dir
    )
{
    if(!ball)
    {
        CCP_LOGWARN_CH( s_chPark,"GotoDirection on non-existing ball. Ignored.\n");
        return;
    }

    if(!_finite(dir.x) || !_finite(dir.y) || !_finite(dir.z))
    {
        CCP_LOGWARN_CH( s_chPark,"NaN in GotoDirection. Ignored");
        return;
    }

    dir.Normalize();
    dir = ball->mNewPos + dir*1.0e17;

    //CCP_LOG_CH( s_chPark,"Ball:%I64d GotoDirection received at %d", ball->mId, mCurrentTime);
    GotoPoint(ball,dir);
}

//---------------------------------------------------------------------------------------
// GotoPoint makes ball src go to the specified point
//---------------------------------------------------------------------------------------

void Ballpark::GotoPoint(
    const ID& srcId,
    double x,
    double y,
    double z
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    GotoPoint(ball,Vector3d(x,y,z));
}
//---------------------------------------------------------------------------------------
// GotoPoint takes a pointer argument
//---------------------------------------------------------------------------------------

void Ballpark::GotoPoint(
    Ball *ball,
    const Vector3d& p
    )
{
    if(!_finite(p.x) || !_finite(p.y) || !_finite(p.z) )
    {
        CCP_LOGWARN_CH( s_chPark,"NaN in GotoPoint. Ignored");
        return;
    }

    // Stop whatever it was doing
    Stop(ball);

    // give it a point to go to
    ball->mGoto = p;

    if(ball->mSpeedFraction == 0.0)
        ball->mSpeedFraction = 1.0;

    ball->SetMode(DSTBALL_GOTO);
}

//---------------------------------------------------------------------------------------
// Stop makes ball src stop whatever it was doing and come to a rest
//---------------------------------------------------------------------------------------

void Ballpark::Stop(
    const ID& srcId
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
    {
        CCP_LOG_CH( s_chPark,"Ballpark::Stop() Ball does not exist!\n");
        return;
    }

    if(ball->mMode == DSTBALL_STOP)
        return;

    Stop(ball);
}

//---------------------------------------------------------------------------------------
// Stop takes a ball pointer as argument
//---------------------------------------------------------------------------------------

void Ballpark::Stop(
    Ball *ball
    )
{
    CCP_ASSERT(ball);
    // stop following or orbiting others
    uint8_t ballMode = ball->mMode;
	if(std::any_of(followModes.begin(), followModes.end(), [ballMode](int mode){return ballMode == mode;}) )
    {
        CCP_ASSERT(ball->mFollowPtr);

        size_t found = 0;
        if(isMaster)
        {
            found = ball->mFollowPtr->mFollowers.erase(ball->mId);
        }
        else
        {
            // Something is crashing clients in ball->mFollowPtr->mFollowers.erase(ball) but I'm not sure what yet.
            // http://defects/issue.asp?ISID=56085
            // Current theory is that ball->mFollowPtr or ball->mFollowPtr->mFollowers somehow ends up pointing to garbage.
            // I'm going to check that ball->mFollowPtr matches mBalls[ball->mFollowId] (it always should) and log out anything that is wrong.
            Ball *followById = mBalls[ball->mFollowId];
            if(!followById)
            {
                // That's odd - we were supposed to be following a ball that isn't in our park
                CCP_LOGERR_CH( s_chPark,"Ball Stop[%I64d], followId [%I64d] does not refer to a known ball (mode:%d)", ball->mId, ball->mFollowId, ball->mMode);
            }
            else
            {
                if(ball->mFollowPtr != followById)
                {
                    // That's odd - followPtr and followId don't agree
                    CCP_LOGERR_CH( s_chPark,"Ball Stop[%I64d], followPtr does not match followId [%I64d] (mode:%d)", ball->mId, ball->mFollowId, ball->mMode);
                }
                else
                {
                    // Everything is good
                    found = ball->mFollowPtr->mFollowers.erase(ball->mId);
                }
            }
        }

        if(!found)
            CCP_LOGWARN_CH( s_chPark,"[%I64d] Was not found in the other ball follow vector", ball->mId);
#if 0
        else
            CCP_LOGWARN_CH( s_chPark,"[%I64d] Removed from the other ball follow vector", ball->mId);
#endif


        if(isMaster && ball->mMode == DSTBALL_FORMATION)
        {
            // Free up the formation slot
            ball->mFollowPtr->FreeFormationSlot(ball->mEffectStamp);
        }

        ball->mEffectStamp = 0;
        ball->mFollowPtr = 0;
        ball->mFollowId = 0;
        ball->mOwnerId = 0;
        ball->mFollowRange = 0.0;
    }

    ball->SetMode(DSTBALL_STOP);
}
#pragma endregion


#pragma region Ball property getters and setters
//---------------------------------------------------------------------------------------
// SetBallMass sets the mass of the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetBallMass(
    const ID& srcId,
    double mass
    )
{
    if(mass <= 0.0f)
        return;

    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    ball->mMass = mass;
    SetBallTimeFactor(ball);
}

//---------------------------------------------------------------------------------------
// SetBallRadius sets the radius of the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetBallRadius(
    const ID& srcId,
    float radius
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    SetBallRadius(ball,radius);
}

void Ballpark::SetBallRadius(
    Ball *ball,
    float radius
    )
{
    if(radius < 0.0 || !ball)
        return;

    ball->mRadius = radius;

    InsertBallInBoxes(ball);
}

//---------------------------------------------------------------------------------------
// SetMaxSpeed sets the maximum speed for the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetMaxSpeed(
    const ID& srcId,
    float speed
    )
{
    if(speed < 0.0)
        return;

    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    ball->mMaxVel = speed;

    InsertBallInBoxes(ball);
}

void Ballpark::SetMaxAngularSpeed( const ID& srcId, float maxAngVel )
{
	if( maxAngVel < 0.0f )
		return;

	Ball* ball = mBalls[srcId];

	if( !ball )
		return;

	ball->mMaxAngVel = maxAngVel;
}

//---------------------------------------------------------------------------------------
// SetBallPosition sets the position of the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetBallPosition(
    const ID& srcId,
    double x,
    double y,
    double z
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    ball->mNewPos.x = x;
    ball->mNewPos.y = y;
    ball->mNewPos.z = z;

    // Set old position as well to avoid strange warping effects in interpolation
    ball->mOldPos.x = x;
    ball->mOldPos.y = y;
    ball->mOldPos.z = z;

    // changed position can affect placement in space partition.
    if(mProbe==ball->mId)
        CCP_LOGWARN_CH( s_chPark,"[%d] Insert from SetBallPosition", mCurrentTime);

    InsertBallInBoxes(ball);
}

//---------------------------------------------------------------------------------------
// SetBallVelocity sets the velocity vector for the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetBallVelocity(
    const ID& srcId,
    double vx,
    double vy,
    double vz
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    ball->mNewVel.x = vx;
    ball->mNewVel.y = vy;
    ball->mNewVel.z = vz;

    // Set old velocity as well to avoid strange warping effects
    ball->mOldVel.x = vx;
    ball->mOldVel.y = vy;
    ball->mOldVel.z = vz;

    // Snap the rotation to the new speed
	if( g_useDynamicalOrientation )
	{
		ball->SnapOrientation();
	}
	else
	{
		ball->CalculateYawPitchRoll( true );
	}
}

void Ballpark::SetBallAngularVelocity( const ID& srcId, double wx, double wy, double wz )
{
	Ball* ball = mBalls[srcId];

	if( !ball )
		return;

	ball->mNewAngVel.x = float( wx );
	ball->mNewAngVel.y = float( wy );
	ball->mNewAngVel.z = float( wz );

	//Set old value as well to avoid warping in interpolation
	ball->mOldAngVel.x = float( wx );
	ball->mOldAngVel.y = float( wy );
	ball->mOldAngVel.z = float( wz );
}

void Ballpark::SetBallRotation( const ID& srcId, double rx, double ry, double rz, double rw )
{
	Ball* ball = mBalls[srcId];

	if( !ball )
		return;

	// Make sure we have a valid rotation
	Quaternion rotation = Quaternion( static_cast<float>( rx ), static_cast<float>( ry ), static_cast<float>( rz ), static_cast<float>( rw ) );

	// Normalize to make sure this is rotation
	const auto length = Length( rotation );
	if( length > 1e-20 )
	{
		rotation /= length;
		ball->mNewRot = rotation;

		// Set the old rotation as well to avoid bad interpolation
		ball->mOldRot = rotation;

		// Stop the rotation as well
		ball->mNewAngVel.x = 0.0f;
		ball->mNewAngVel.y = 0.0f;
		ball->mNewAngVel.z = 0.0f;

		ball->mOldAngVel = ball->mNewAngVel;
	}
}

//---------------------------------------------------------------------------------------
// SetBallHarmonic
//---------------------------------------------------------------------------------------

void Ballpark::SetBallHarmonic(
    const ID& srcId,
    int64_t harmonic,
    int corporationID,
    int allianceID,
    bool field
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    ball->mHarmonic = harmonic;
    ball->mCorporationID = corporationID;
    ball->mAllianceID = allianceID;

    if(field)
    {
        Stop(ball);
        ball->SetMode(DSTBALL_FIELD);
    }
    else
    {
        if(ball->mMode==DSTBALL_FIELD)
            Stop(ball);
    }

}


//---------------------------------------------------------------------------------------
// SetBallFree sets the 'isFree' flag for the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetBallFree(
    const ID& srcId,
    bool flag
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    if(flag==ball->isFree)
        return;

    ball->isFree = flag;

    if(ball->isFree)
    {
        // Ball had been fixed, need to add it to the map of free balls
        mFreeBalls.insert(DictEntry(ball->mId, ball));

        // Remove miniballs if necessary
        ball->RemoveMiniBalls();
		ball->RemoveMiniCapsules();
		ball->RemoveMiniBoxes();
        
        // Set ball time to a tick a go so that we don't snap back and forth 
        // during first tick of client interpolation
        ball->mNewTime = mTime-mTickInterval*10000;
    }
    else
    {
        // Ball had been free. Put it in a sane state
        Stop(ball);
        SetBallVelocity(ball->mId, 0.0, 0.0, 0.0);
        mFreeBalls.erase(ball->mId);
        InsertBallInBoxes(ball);

        // Clear out accumulated forces (otherwise missile target-prediction can get confused)
        ball->mLastG = Vector3d(0.0, 0.0, 0.0);

        // Add miniballs if necessary
        ball->AddMiniBalls();
		ball->AddMiniCapsules();
		ball->AddMiniBoxes();
    }


}

//---------------------------------------------------------------------------------------
// SetBallAgility sets the agility modifier for the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetBallAgility(
    const ID& srcId,
    float agility
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball || agility <= 0.0)
        return;

    // disabling
    //if(agility > 3.14)
    //    agility = 3.14;

    ball->mAgility = agility;
    SetBallTimeFactor(ball);
}

void Ballpark::SetBallAngularAgility( const ID& srcId, float rotAgility )
{
	Ball* ball = mBalls[srcId];

	if( !ball || rotAgility <= 0.0 )
		return;

	ball->mRotAgility = rotAgility;
}

void Ballpark::SetBallTimeFactor(
    Ball* ball
    )
{
    if(!ball)
        return;

    double mass = ball->mMass*ball->mAgility;

    if(mass <= 0.0)
        ball->mTimeFactor = 0.0;
    else
    {
        ball->mTimeFactor = exp(-mFriction*dt/mass);
    }
}


//---------------------------------------------------------------------------------------
// SetSpeedFraction sets the speed fraction for the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetSpeedFraction(
    const ID& srcId,
    float fraction
    )
{
    if(!_finite(fraction) )
    {
        CCP_LOGWARN_CH( s_chPark,"NaN in SetSpeedFraction. Ignored");
        return;
    }

    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    if(fraction < 0.0)
        fraction = 0.0;
    else if(fraction > 1.0)
        fraction = 1.0;

    /*
    if(ball->mMode == DSTBALL_STOP && fraction > 0.0)
    {
        // I was not moving, so start in some direction
        Quaternion q;
        ball->Update(&q,ball->mRotUpdateTime);
        Vector3 v = Vector3(0.0f,0.0f,-1.0f);
        TriVectorRotateQuaternion(&v,&v,&q);
        GotoDirection(srcId,v.x,v.y,v.z);
    }
    */

    ball->mSpeedFraction = fraction;
}

void Ballpark::SetNotificationRange(
    const ID& srcId,
    double notificationRange
    )
{
    Ball *ball = mBalls[srcId];
    if(!ball)
        return;

    if(notificationRange < 0.0f)
        return;

    ball->mNotificationRange = notificationRange;
    ball->UpdateProximityStatus();
    ball->mWithinRange = false;
}

void Ballpark::SetTargetTracking(const ID& srcID, const ID& targetID, double trackingRange)
{
    Ball *ball = mBalls[srcID];
    if(!ball)
    {
        return;
    }

    if ((trackingRange <= 0.0f) || (targetID == 0))
    {
        ball->mTrackingPtr = NULL;
        ball->mTrackingRange = 0.0;
        return;
    }

    Ball *target = mBalls[targetID];
    if(!target)
    {
        return;
    }

    ball->mTrackingPtr = target;
    ball->mTrackingRange = trackingRange;
    ball->UpdateProximityStatus();
    ball->mWithinTrackingRange = false;
}

//---------------------------------------------------------------------------------------
// SetBallMassive sets the 'isMassive' flag for the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetBallMassive(
    const ID& srcId,
    bool flag
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    ball->isMassive = flag;
}

//---------------------------------------------------------------------------------------
// SetBallGlobal sets the 'isGlobal' flag for the given ball
//---------------------------------------------------------------------------------------

void Ballpark::SetBallGlobal(
    const ID& srcId,
    bool flag
    )
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    if(flag)
    {
        // Ball should be global
        mGlobals.Add(ball->mId, ball);
    }else
    {
        // Not global
        if(ball->isGlobal)
            mGlobals.RemoveBall(ball->mId);
    }

    ball->isGlobal = flag;

}

//---------------------------------------------------------------------------------------
// AddProximitySensor attaches a proximity sensor to the given ball
//---------------------------------------------------------------------------------------

void Ballpark::AddProximitySensor(
    const ID& srcId,
    float range,
    double period,
    int shuffle,
    bool onlyInteractives
)
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    ball->AddProximitySensor(range, period, shuffle, onlyInteractives);
}

//---------------------------------------------------------------------------------------
// RemoveProximitySensor removes a proximity sensor from the given ball
//---------------------------------------------------------------------------------------

void Ballpark::RemoveProximitySensor(
    const ID& srcId
)
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    ball->RemoveProximitySensor();
}

//---------------------------------------------------------------------------------------
// GetAccuracy returns accuracy factor
//---------------------------------------------------------------------------------------

double Ballpark::GetAccuracy(
    const ID& srcId,
    const ID& dstId,
    double& optimalRange,
    double falloff,
    double& trackingSpeed,
    double signatureRadius,
    double optimalSignatureRadius
)
{
    if(srcId==dstId)
    {
        CCP_LOGWARN_CH( s_chPark,"Accuracy requested between same balls. Ignored.");
        return 0.0;
    }

    // Now check that these guys exist
    Ball *src = mBalls[srcId];
    Ball *dst = mBalls[dstId];

    if(!src || !dst)
    {
        CCP_LOGWARN_CH( s_chPark,"Accuracy requested for non-existing balls. Ignored.");
        return 0.0;
    }

    if(optimalRange < 0.0f )
    {
        CCP_LOGWARN_CH( s_chPark,"Incorrect accuracy parameters. Ignored.");
        return 0.0;
    }

    if(trackingSpeed==0.0f || signatureRadius==0.0f || falloff==0.0f)
        return 0.0;

    // Find the short dist
    Vector3d radial = dst->mNewPos - src->mNewPos;
    double dist = radial.Length();
    double shortDist = (dist - dst->mRadius) - src->mRadius;
    if(shortDist < 0.0)
        shortDist = 0.0;

    if(dist==0.0)
        return 0.0;

    // Get the tranverse speed
    Vector3d combined = dst->mNewVel - src->mNewVel;

    // dot the combined speed to the radial component
    double radialSpeed = combined*radial/dist;

    double speed2 = (combined.LengthSq() - radialSpeed*radialSpeed)/(dist*dist);

    // Now all the possible singularites have been averted. Go boldly.
    double exponent = optimalSignatureRadius/(trackingSpeed*signatureRadius);
    exponent *= -exponent*speed2;
    if(shortDist > optimalRange)
        exponent -= (shortDist-optimalRange)*(shortDist-optimalRange)/(falloff*falloff);

    optimalRange = shortDist;
    trackingSpeed = sqrt(speed2);
    return pow(2.0,exponent);
}
//---------------------------------------------------------------------------------------
// CloakBall cloaks a ball
//---------------------------------------------------------------------------------------

void Ballpark::CloakBall(
    const ID& srcId,
    char cloakMode,
    float range
)
{
    Ball *ball = mBalls[srcId];

    if(!ball)
    {
        CCP_LOGWARN_CH( s_chPark,"CloakBall on non-existing ball. Ignored.");
        return;
    }

    if(cloakMode <= 0)
    {
        CCP_LOGWARN_CH( s_chPark,"CloakBall called with mode <= 0. Ignored.");
        return;
    }

    if(isMaster && cloakMode==DSTNORMALCLOAK)
    {
        // Add a proximity sensor
        ball->AddProximitySensor(range, 2.0, 0);
    }
    StopAllFollowers(ball);
    ball->isCloaked = cloakMode;
    ball->isMassive = false;
}

//---------------------------------------------------------------------------------------
// UncloakBall uncloaks a ball
//---------------------------------------------------------------------------------------

void Ballpark::UncloakBall(
    const ID& srcId
)
{
    Ball *ball = mBalls[srcId];

    if(!ball )
        return;

    if(isMaster && ball->isCloaked==DSTNORMALCLOAK)
    {
        // Remove a proximity sensor
        ball->RemoveProximitySensor();
    }

    ball->isCloaked = 0;

    if( ! ball->IsWarping() )
    {
        ball->isMassive = true; // Only set the ball massive if it's not in proper warp.  
        // clear this ball from all proximity sensors in this bubble
        DictOfBalls::iterator sit;
        for(sit = mProximityBalls.begin(); sit != mProximityBalls.end(); ++sit)
        {
            Ball* sensorBall = sit->second;
            if(sensorBall->mNewBubble != ball->mNewBubble)
                continue; // different bubble, don't care.
            if(sensorBall->isMoribund)
                continue; // dead ball, don't care
            if(!sensorBall->mHasProximity)
                continue; // sensor not active (probably don't need this check)
            CCP_LOG_CH( s_chPark,"Decloaking Ball %I64d: Removing ball from sensor %I64d", srcId, sensorBall->mId);
            sensorBall->mSensor.balls.erase(srcId);
        }
    }
}
#pragma endregion

//---------------------------------------------------------------------------------------
// CheckVisibility checks visilibity between two balls
//---------------------------------------------------------------------------------------

ID Ballpark::CheckVisibility(
    const ID& srcId,
    const ID& dstId
)
{
    TASKLET(Timer_CheckVisibility);

    Ball *src = mBalls[srcId];

    if(!src)
    {
        CCP_LOGWARN_CH( s_chPark,"Non existing source Ball:%I64d checked for visibility\n",srcId);
        return -2;
    }

    Ball *dst = mBalls[dstId];

    if(!dst)
    {
        CCP_LOGWARN_CH( s_chPark,"Non existing destination Ball:%I64d checked for visibility\n",dstId);
        return -2;
    }

    // If balls are in different bubbles there is no visibility
    if(src->mNewBubble != dst->mNewBubble)
    {
        CCP_LOGWARN_CH( s_chPark,"Visibility between Ball:%I64d and Ball:%I64d in different bubbles\n",srcId, dstId);
        return -2;
    }

    // Now go through all balls in same bubble.
    // If one of them intersects the ray between
    // the two balls, then there no visibility
    ID visible = 0;

    // ray from origin in 'src' to 'dst'
    Vector3d origin = src->mNewPos;
    Vector3d ray = dst->mNewPos - src->mNewPos;
    // The distance between the points
    double distance = ray.Length();
    ray.Normalize();

    BallIterator it(&mBalls);
    Ball* occluder;
    while((occluder = it++))
    {
        if( occluder->mNewBubble != src->mNewBubble || occluder == src || occluder == dst || !occluder->isMassive || occluder->isCloaked)
            continue;

        Vector3d toCenter = occluder->mNewPos - src->mNewPos;
        double v = ray * toCenter;
        double r = occluder->mRadius;
        double dsq = r*r - (toCenter*toCenter-v*v);
        if(dsq < 0.0)
            continue; // no intersection

        double d = sqrt(dsq);
        if(v-d > 0.0 && v-d < distance)
        {
            visible = occluder->mId;
            break;
        }
    }

    //CCP_LOG_CH( s_chPark,"Ball:%I64d and Ball:%I64d have visibility %I64d\n",srcId, dstId, visible);
    return visible;
}


int Ballpark::GetIntersection(Vector3d &origin, Vector3d &ray, Vector3d &sphere, double radius, double *first, double *second)
{
    Vector3d delta = origin - sphere;
    double b = 2.0*ray*delta;
    double c = delta.LengthSq() - radius*radius;

    double det = b*b - 4.0*c;

    if(det < 0.0)
        return 0; // No intersection

    if(det==0.0) // One intersection
    {
        if(first)
            *first = -b*0.5;

        return 1;
    }

    // Two intersections
    det = sqrt(det);

    if(first || second)
    {
        double t = (-b - det)*0.5;
        if(t > 0.0){
            // This is the closer of the two
            if(first)
                *first = t;

            t = (-b + det)*0.5;
            if(second)
                *second = t;
        }
        else
        {
            if(second)
                *second = t;

            t = (-b + det)*0.5;
            if(first)
                *first = t;
        }
    }

    return 2;
}

void Ballpark::StopAllFollowers(Ball *ball)
{
    if(!ball)
        return;

    if(ball->mFollowers.size()==0)
        return;

    auto followersCopy = ball->mFollowers;
    auto follower = followersCopy.begin();
    auto end = followersCopy.end();

    for(;follower != end;++follower)
    {
		auto id = *follower;
		auto f = mBalls[id];

		if( !f )
		{
			CCP_LOGERR_CH( s_chPark, "Ball::StopAllFollowers: ball %I64d has an invalid follower %I64d (invalid id)", ball->mId, id );
			ball->mFollowers.erase( id );
			continue;
		}

		if( f->mFollowId != ball->mId )
		{
			CCP_LOGERR_CH( s_chPark, "Ball::StopAllFollowers: ball %I64d has an invalid follower %I64d (mismatched followId %I64d)", ball->mId, id, f->mFollowId );
			ball->mFollowers.erase( id );
			continue;
		}

		if( f->mFollowPtr != ball )
		{
			CCP_LOGERR_CH( s_chPark, "Ball::StopAllFollowers: ball %I64d has an invalid follower %I64d (followPtr doesn't match followId)", ball->mId, id );
			ball->mFollowers.erase( id );
			continue;
		}

		if( f->mMode == DSTBALL_MISSILE || (f->isInteractive && f->mMode == DSTBALL_ORBIT) )
        {
            if(f->mMode==DSTBALL_MISSILE)
            {
                // Once their target is gone, missiles should just fly off into the sunset without collisions
                // (They will be removed once they time-out, as normal)
                f->isMassive = false;
            }
            GotoDirection(f ,f->mNewVel);
        }
        else
        {
            Stop(f);
        }
	}
}

void Ballpark::RemoveBall(
    const ID& srcId,
    int delay
    )
{
    Ball *ball = mBalls[srcId];
    if(!ball)
    {
        CCP_LOGERR_CH( s_chPark,"Ballpark::RemoveBall trying to remove non-existing ball %I64d.", srcId);
        return;
    }

    // Stop whatever it was doing
    Stop(ball);
    // Stop any followers as well
    StopAllFollowers(ball);

	if( !ball->mFollowers.empty() )
	{
		CCP_LOGERR_CH( s_chPark, "Ballpark::RemoveBall[%I64d], ball still has followers after stopping", ball->mId );
	}

	ball->isMoribund = true;

    // miniballs should be removed regardless of delay
    ssize_t miniballSize = ball->mMiniBalls.GetSize();
    if(miniballSize > 0 && !ball->isFree)
    {
        CCP_LOG_CH( s_chPark,"Deleting %d miniballs for ball %I64d", miniballSize, srcId);
        //ball->RemoveMiniBalls();
        for(ssize_t i=miniballSize-1 ; i >= 0; i--)
        {
            MiniBall *mb = (MiniBall *)(void *)(ball->mMiniBalls.GetAt(i));
            RemoveBall(mb->mId);
        }
        ball->mMiniBalls.Remove(-1); // special cased in BlueListC to clear the list.
    }
	ssize_t minicapsuleSize = ball->mMiniCapsules.GetSize();
	if(minicapsuleSize > 0 && !ball->isFree)
	{
		CCP_LOG_CH( s_chPark,"Deleting %d minicapsules for ball %I64d", minicapsuleSize, srcId);
		for(ssize_t i=minicapsuleSize-1 ; i >= 0; i--)
		{
			MiniCapsule *mc = (MiniCapsule *)(void *)(ball->mMiniCapsules.GetAt(i));
			RemoveCapsule(mc->mId);
		}
		ball->mMiniCapsules.Remove(-1); // special cased in BlueListC to clear the list.
	}
	ssize_t miniboxSize = ball->mMiniBoxes.GetSize();
	if(miniboxSize > 0 && !ball->isFree)
	{
		CCP_LOG_CH( s_chPark,"Deleting %d miniboxes for ball %I64d", miniboxSize, srcId);
		for(ssize_t i=miniboxSize-1 ; i >= 0; i--)
		{
			MiniBox *mc = (MiniBox *)(void *)(ball->mMiniBoxes.GetAt(i));
			RemoveOrientedBox(mc->mId);
		}
		ball->mMiniBoxes.Remove(-1); // special cased in BlueListC to clear the list.
	}
    if(inEvolve || delay > 0)
    {
        // If trying to remove ball in the middle of an Evolve loop
        // don't do it, but mark it as soon to be dead...        
        // BringOutTheDeads only calls us with delay==0 and inEvolve==false,
        // so we never end up here from that call
        ball->isMassive = false;
        ball->mEffectStamp = mCurrentTime + delay;
        moribundBalls.insert(ball);
		return;
    } 

    // Last chance to kill all references etc associated with this ball
    // We add this so that we can be more event based, rather than relying on tasklet timings
    if( !PyOS->SendEvent( ball->GetRawRoot(), 
                          "Destiny::Ball::DoFinalCleanup",
                          "DoFinalCleanup", 
                          NULL,
                          "()"))
    {
        PyOS->PyError();
    }

    //remove from moribund balls. For safety, it should have been already removed.
    moribundBalls.erase(ball);

	if( g_useIterativeCollision )
	{
		//Remove from collision calculations
		mCollisionLocations.erase(ball->mId);
		mPotentialCollidables.erase(ball->mId);
		mPotentialCollisionBalls.erase(ball->mId);
	}

    // Get the ball out of all boxes
    ball->DeleteFromBoxes();

    ball->mPark = 0;
    // remove it from the map of balls
    ball->mOldBubble = ball->mNewBubble;
    ball->mNewBubble = -1;
    AddToBubble(ball);

    if(ball->isFree)
        mFreeBalls.erase(srcId);

    if(ball->mHasProximity)
    {
        mProximityBalls.erase(srcId);
    }


    if(srcId==mEgo)
    {
        mLastEgo = 0,
        mLastEgoBall = 0;
    }

    if(ball->isGlobal)
        mGlobals.RemoveBall(srcId);

    mBalls.RemoveBall(srcId);
}

void Ballpark::RemoveCapsule(
	const ID& srcId,
	int delay
	)
{
	Capsule *capsule = mCapsules[srcId];
	if(!capsule)
	{
		CCP_LOGERR_CH( s_chPark,"Ballpark::RemoveCapsule trying to remove non-existing capsule %I64d.", srcId);
		return;
	}

	capsule->isMoribund = true;

	if(inEvolve || delay > 0)
	{
		// If trying to remove ball in the middle of an Evolve loop
		// don't do it, but mark it as soon to be dead...        
		// BringOutTheDeads only calls us with delay==0 and inEvolve==false,
		// so we never end up here from that call
		capsule->mEffectStamp = mCurrentTime + delay;
		moribundCapsules.insert(capsule);
		return;
	} 

	// Last chance to kill all references etc associated with this ball
	// We add this so that we can be more event based, rather than relying on tasklet timings
	if( !PyOS->SendEvent( capsule->GetRawRoot(), 
		"Destiny::Capsule::DoFinalCleanup",
		"DoFinalCleanup", 
		NULL,
		"()"))
	{
		PyOS->PyError();
	}

	//remove from moribund balls. For safety, it should have been already removed.
	moribundCapsules.erase(capsule);

	// Get the ball out of all boxes
	capsule->DeleteFromBoxes();

	capsule->mPark = 0;
	// remove it from the map of balls
	capsule->mOldBubble = capsule->mNewBubble;
	capsule->mNewBubble = -1;
	AddToBubble(capsule);

	mCapsules.erase(srcId);
}

void Ballpark::RemoveOrientedBox(
	const ID& srcId,
	int delay
	)
{
	OrientedBox *obb = mOrientedBoxes[srcId];
	if(!obb)
	{
		CCP_LOGERR_CH( s_chPark,"Ballpark::RemoveOrientedBox trying to remove non-existing box %I64d.", srcId);
		return;
	}

	obb->isMoribund = true;

	if(inEvolve || delay > 0)
	{
		// If trying to remove ball in the middle of an Evolve loop
		// don't do it, but mark it as soon to be dead...        
		// BringOutTheDeads only calls us with delay==0 and inEvolve==false,
		// so we never end up here from that call
		obb->mEffectStamp = mCurrentTime + delay;
		moribundOrientedBoxes.insert(obb);
		return;
	} 

	// Last chance to kill all references etc associated with this ball
	// We add this so that we can be more event based, rather than relying on tasklet timings
	if( !PyOS->SendEvent( obb->GetRawRoot(), 
		"Destiny::OrientedBox::DoFinalCleanup",
		"DoFinalCleanup", 
		NULL,
		"()"))
	{
		PyOS->PyError();
	}

	//remove from moribund balls. For safety, it should have been already removed.
	moribundOrientedBoxes.erase(obb);

	// Get the ball out of all boxes
	obb->DeleteFromBoxes();

	obb->mPark = 0;
	// remove it from the map of balls
	obb->mOldBubble = obb->mNewBubble;
	obb->mNewBubble = -1;
	AddToBubble(obb);

	mOrientedBoxes.erase(srcId);
}

//---------------------------------------------------------------------------------------
// BringOutTheDeads removes all moribund collidables
//---------------------------------------------------------------------------------------
void Ballpark::BringOutTheDeads()
{
	if(inEvolve)
		return;

	BringOutDeadBalls();
	BringOutDeadCapsules();
}

void Ballpark::BringOutDeadBalls()
{
    int killCounter = 0;

    VectorOfBalls toDelete;
    if(moribundBalls.size() > 0)
    {
        SetOfBalls::iterator sit;
        for(sit=moribundBalls.begin(); sit != moribundBalls.end(); ++sit)
        {
            Ball* ball = *sit;
        
            if(!ball)
            {
                CCP_LOGERR_CH( s_chPark, "BringOutTheDeads:: NULL pointer in moribund balls.");
                toDelete.push_back(ball);
                continue;
            }
			
			if( !ball->isMoribund )
			{
				CCP_LOGERR_CH( s_chPark, "BringOutTheDeads:: Ball %I64d in moribund balls, but not flagged as such.", ball->mId );
				toDelete.push_back( ball );
				continue;
			}

            // Ball has reached the absolute maximum age or being removed with delay = 0
			if( ball->mEffectStamp - mCurrentTime < 0 )
			{
				toDelete.push_back( ball );
			}

            // Ball is within its buffer time to be removed
            // Remove it if we have not reached our maximum kill count this tick
            else if((ball->mEffectStamp - mCurrentTime - mMoribundBallRemovalBuffer  < 0) && (killCounter < mMoribundBallRemovalCount))
            {
                toDelete.push_back(ball);
                ++killCounter;
            }
        }
    
        if(toDelete.size() > 0)
        {
			CCP_LOG_CH( s_chPark, "BringOutTheDeads:: Removing %d moribund balls (out of %d).", toDelete.size(), moribundBalls.size() );
			VectorOfBalls::iterator toDelIt;
            for(toDelIt=toDelete.begin(); toDelIt != toDelete.end(); ++toDelIt)
            {
                Ball* ball = *toDelIt;
                if( ball && ball->isMoribund )
                {
                    ID id = ball->mId;
                    RemoveBall(id);
                }
                moribundBalls.erase(ball);
            }
        }
    }
}

void Ballpark::BringOutDeadCapsules()
{
	int killCounter = 0;

	VectorOfCapsules toDelete;
	if(moribundCapsules.size() > 0)
	{
		SetOfCapsules::iterator sit;
		for(sit=moribundCapsules.begin(); sit != moribundCapsules.end(); ++sit)
		{
			Capsule* capsule = *sit;

			if(!capsule)
			{
				CCP_LOGERR_CH( s_chPark, "BringOutTheDeads:: NULL pointer in moribund capsules.");
				toDelete.push_back(capsule);
				continue;
			}
			if(!capsule->isMoribund)
				CCP_LOGERR_CH( s_chPark, "BringOutTheDeads:: Capsule in moribund capsules, but not flagged as such");

			// Ball has reached the absolute maximum age or being removed with delay = 0
			if(capsule->mEffectStamp - mCurrentTime < 0)
				toDelete.push_back(capsule);

			// Ball is within its buffer time to be removed
			// Remove it if we have not reached our maximum kill count this tick
			else if((capsule->mEffectStamp - mCurrentTime - mMoribundBallRemovalBuffer  < 0) && (killCounter < mMoribundBallRemovalCount))
			{
				toDelete.push_back(capsule);
				++killCounter;
			}
		}

		if(toDelete.size() > 0)
		{
			CCP_LOG_CH( s_chPark, "BringOutTheDeads:: Removing %d moribund capsules.", toDelete.size());
			VectorOfCapsules::iterator toDelIt;
			for(toDelIt=toDelete.begin(); toDelIt != toDelete.end(); ++toDelIt)
			{
				Capsule* capsule = *toDelIt;
				if(capsule)
				{
					ID id = capsule->mId;
					RemoveCapsule(id);
				}
				moribundCapsules.erase(capsule);
			}
		}
	}
}

//---------------------------------------------------------------------------------------
// HandleProximities udpates all proximity sensors
//---------------------------------------------------------------------------------------
void Ballpark::HandleProximities()
{
    TASKLET(Timer_HandleProximities);

    DictOfBalls::iterator sit;
    Ball *ball;

    DictOfBalls aCopy = mProximityBalls;

    //CCP_LOGWARN_CH( s_chPark,"Ballpark::HandleProximites %d sensors to check",mProximityBalls.size());

    // cycle over all balls
    int cnt = 0;
    for(sit = aCopy.begin(); sit != aCopy.end(); ++sit)
    {
        ball = sit->second;

        if(ball->isMoribund)
            continue; // don't handle dead balls

        if(InDeadBubble(ball))
        {
            if(ball->mSensor.balls.size() > 0)
            {
                // Some sensors don't run every tick.
                // We need to make sure this one definitely runs the next time we call CheckForProximity()
                // This ensures that the sensor gets one last chance to handle nearby balls when its bubble has become dead
                ball->mSensor.elapsed = ball->mSensor.period;
                ((IBall*)ball)->Lock();
                ball->CheckForProximity();
                ((IBall*)ball)->Unlock();
            }
            ball->mSensor.balls.clear();
            continue;                
        }

        if(!ball->mHasProximity)
            continue;

        ((IBall*)ball)->Lock();
        ball->CheckForProximity();
        ((IBall*)ball)->Unlock();
        cnt++;
    }
    //CCP_LOGWARN_CH( s_chPark,"Ballpark::HandleProximites %d sensors actually checked",cnt);
}

//---------------------------------------------------------------------------------------
// ClearBubbles clears bubble related Python dicts and sets.
//---------------------------------------------------------------------------------------
void Ballpark::ClearBubbles()
{
	if( bubbles )
	{
		PyDict_Clear( bubbles );
	}
	if( bubbleInteractives )
	{
		PyDict_Clear( bubbleInteractives );
	}
	if( bubbleKeepAlives )
	{
		PyDict_Clear( bubbleKeepAlives );
	}
	if( bubbleKeepAliveBalls )
	{
		PySet_Clear( bubbleKeepAliveBalls );
	}
}

//---------------------------------------------------------------------------------------
// ClearAll clears the ballpark of all balls and collidables
//---------------------------------------------------------------------------------------

void Ballpark::ClearAll(
    )
{
    BallIterator it(&mBalls);
    MapOfBoxes::iterator jt;
    Ball *b;

    while((b = it++))
    {
        b->DeleteFromBoxes();
        b->mPark = 0;
    }

    mBalls.Clear();
    mGlobals.Clear();

	if( g_useIterativeCollision )
	{
		mCollisionLocations.clear();
		mPotentialCollisionBalls.clear();
		mPotentialCollidables.clear();
	}

	std::unordered_map<ID, StaticCollidable*>::iterator cit;
	StaticCollidable *c;

	for( cit = mStaticCollidables.begin(); cit != mStaticCollidables.end(); ++cit)
	{
		c = cit->second;
		c->DeleteFromBoxes();
		c->mPark = 0;
	}

	mStaticCollidables.clear();
	mCapsules.clear();
	mOrientedBoxes.clear();

    ClearBubbles();

    if(!mBubbleConflicts.empty())
    {
        SetOfBalls::iterator it;
        for(it=mBubbleConflicts.begin(); it!=mBubbleConflicts.end(); ++it)
        {
            Ball *ball = *it;
            ((IBall*)ball)->Unlock();
        }
        mBubbleConflicts.clear();
    }

    mFreeBalls.clear();
    mProximityBalls.clear();
    mLocalCnt = -1;
    mLocalHiCnt = DSTLOCALBALLS;

    mLastEgo = 0,
    mLastEgoBall = 0;
}

//---------------------------------------------------------------------------------------
// UpdateBallBubble() finds the bubble for a single new ball
//---------------------------------------------------------------------------------------

void Ballpark::UpdateBallBubble(
    const ID& srcID
    )
{
    Ball *ball = mBalls[srcID];
    if(!ball)
    {
        CCP_LOGERR_CH( s_chPark,"Ballpark::UpdateBallBubble trying to update a non-existing ball. Canceled.\n");
        return;
    }

    if(ball->mNewBubble != -1)
    {
        CCP_LOGWARN_CH( s_chPark,"Ballpark::UpdateBallBubble trying to update a ball with a bubble.\n");
    }

    BallIterator it(&mBalls);

    double maxWidth = mPartition->mLevelWidth[0];
    maxWidth = maxWidth*maxWidth;

    long hiID = -1;
    Ball *b;
    while((b = it++))
    {
        if(b->mNewBubble > hiID)
            hiID = b->mNewBubble;

        if(b==ball || b->mNewBubble == -1)
            continue;

        if((ball->mNewPos-b->mNewPos).LengthSq() < maxWidth)
        {
            ball->mNewBubble = b->mNewBubble;
            break;
        }
    }

    if(ball->mNewBubble == -1)
    {
        CCP_LOGWARN_CH( s_chPark,"Ballpark::UpdateBallBubble couldn't find a bubble. Setting it to %d.\n",hiID+1);
        ball->mNewBubble = hiID+1;
    }
}

//---------------------------------------------------------------------------------------
// AddTransitionToList creates a python object for the bubble transition of this ball,
// and appends it to the provided list
//---------------------------------------------------------------------------------------

void Ballpark::AddTransitionToList(const Ball *ball, PyObject *transitions)
{
	if (transitions == nullptr)
	{
		CCP_LOGWARN_CH( s_chPark,"Ballpark::AddTransitionToList failed, provided transitions list is invalid");
		return;
	}
	if( ball->mMode == DSTBALL_TROLL )
	{
		// Safeguard to make sure we do not register Bubble Transitions for balls in troll mode, as
		// those should not be player ships
		return;
	}

	PyObject* transition = Py_BuildValue(
		"Lii",
		ball->mId,
		ball->mOldBubble,
		ball->mNewBubble );
	if( !transition )
	{
		CCP_LOGWARN_CH( s_chPark, "[%d] Failed to create bubble transition for ball %I64d", mCurrentTime, ball->mId );
		PyErr_Clear();
		return;
	}

	bool canAppendTransition = PyList_Append( transitions, transition ) == 0; // according to PyList_Append, 0 means success
	if( !canAppendTransition )
	{
		CCP_LOGWARN_CH( s_chPark, "[%d] Failed to append bubble transition for ball %I64d", mCurrentTime, ball->mId );
		PyErr_Clear();
	}
	Py_XDECREF(transition);
}

//---------------------------------------------------------------------------------------
// NotifyOfBubbleTransitions emits event communicating that balls have changed bubbles
//---------------------------------------------------------------------------------------

void Ballpark::NotifyOfBubbleTransitions(const PyObject* transitions)
{
	if (transitions && !PyOS->SendEvent(
			(IEveBallpark*)this, "Destiny::DoBubbleTransitions",
			"DoBubbleTransitions", NULL,"(O)", transitions
			))
	{
		PyOS->PyError();
	}
}

//---------------------------------------------------------------------------------------
// ResolveBubbleConflicts recacalculates causal domains and assigns new bubble Ids
//---------------------------------------------------------------------------------------

void Ballpark::ResolveBubbleConflicts()
{
    TASKLET(Timer_ResolveBubbleConflicts);

    if(mBubbleConflicts.empty())
    {
        return;
    }

    //BeTimer timer = BeTimer();

    SetOfBalls::iterator it;
    for(it = mBubbleConflicts.begin(); it != mBubbleConflicts.end(); ++it)
    {
        Ball *ball = *it;

        if(ball->isMoribund){
            CCP_LOGWARN_CH( s_chPark,"Moribund ball %d being resolved for bubbles", ball->mId);
            ((IBall*)ball)->Unlock();
            continue;
        }

        Box *box = mPartition->GetTopmost(ball->mMyBox);

        if(mProbe==ball->mId)
            CCP_LOGWARN_CH( s_chPark,"[%d] Resolving for conflicts (%d -> %d)", mCurrentTime, ball->mOldBubble, ball->mNewBubble);
        //CCP_LOG_CH( s_chPark,"PRE %d: %d %d",i,ball->mOldBubble,ball->mNewBubble);

        if(box->mBubble != -1)
        {
            ball->mOldBubble = ball->mNewBubble;
            ball->mNewBubble = box->mBubble;
            if(mProbe==ball->mId)
                CCP_LOGWARN_CH( s_chPark,"[%d] Box had bubble %d. Adding ball to it if different.", mCurrentTime, box->mBubble);

            if(ball->mOldBubble != ball->mNewBubble)
            {
                //RemoveBallFromBubble(ball,ball->mOldBubble);
                AddToBubble(ball);
            }
            //CCP_LOG_CH( s_chPark,"PST %d: %d %d",i,ball->mOldBubble,ball->mNewBubble);

            ((IBall*)ball)->Unlock();
            continue; // aldready traversed
        }

        // Must check neighbors for ID
        long newBubbleId = box->NearbyBubbles();

        if(newBubbleId != -1)
        {
            ball->mOldBubble = ball->mNewBubble;
            ball->mNewBubble = newBubbleId;
            box->mBubble = newBubbleId;

            if(mProbe==ball->mId)
                CCP_LOGWARN_CH( s_chPark,"[%d] Neighbor had bubble %d. Adding ball to it if different.", mCurrentTime, newBubbleId);

            if(ball->mOldBubble != ball->mNewBubble)
            {
                //RemoveBallFromBubble(ball,ball->mOldBubble);
                AddToBubble(ball);
            }
            //CCP_LOG_CH( s_chPark,"PST %d: %d %d",i,ball->mOldBubble,ball->mNewBubble);
            ((IBall*)ball)->Unlock();
            continue;
        }

        // Must assign a new bubble ID
        ball->mOldBubble = ball->mNewBubble;
        ball->mNewBubble = mBubbleId;
        box->mBubble = mBubbleId;

        if(mProbe==ball->mId)
            CCP_LOGWARN_CH( s_chPark,"[%d] Created new bubble %d. Adding ball to it if different.", mCurrentTime, mBubbleId);

        if(ball->mOldBubble != ball->mNewBubble)
        {
            //RemoveBallFromBubble(ball,ball->mOldBubble);
            AddToBubble(ball);
        }
            //CCP_LOG_CH( s_chPark,"PST %d: %d %d",i,ball->mOldBubble,ball->mNewBubble);

        ((IBall*)ball)->Unlock();
        mBubbleId++;
    }

    mBubbleConflicts.clear();
    //CCP_LOG_CH( s_chPark,"Conflict Resolution: %f",timer.GetTime()/10000.0);
}

void Ballpark::CopyBubbles()
{
    TASKLET(Timer_CopyBubbles);

    if(!bubbles)
    {
        return;
    }

    PyObject *delBallList   = PyList_New(0);
    PyObject *delBubbleList = PyList_New(0);
    // Now go over all entries and make copies of them...
    PyObject *bubbleID, *bubble;
    Py_ssize_t pos1 = 0;

    while (PyDict_Next(bubbles, &pos1, &bubbleID, &bubble))
    {
        PyObject *ballID, *value;
        Py_ssize_t pos2 = 0;
        while (PyDict_Next(bubble, &pos2, &ballID, &value))
        {
            long action = PyLong_AS_LONG(value);
            if(action == 1)
            {
                PyDict_SetItem(bubble, ballID, Zero);
            }
            else if(action == -1)
            {
                // Flag this ID to be deleted from the dict
                PyList_Append(delBallList, ballID);
            }
        }
        // Now delete all the ones that are supposed to be deleted
        Py_ssize_t i;
        Py_ssize_t lim = PyList_Size(delBallList);
        for(i = 0; i < lim ; i++)
        {
            PyObject *key = PyList_GET_ITEM(delBallList, i);
            PyDict_DelItem(bubble, key);
        }


        if(PyDict_Size(bubble) == 0)
        {
            PyList_Append(delBubbleList, bubbleID);
        }

        PySequence_DelSlice(delBallList, 0, lim);
    }

    Py_ssize_t i;
    Py_ssize_t lim = PyList_Size(delBubbleList);
    for(i = 0; i < lim ; i++)
    {
        PyObject *key = PyList_GET_ITEM(delBubbleList, i);
        PyDict_DelItem(bubbles, key);
    }

    Py_DECREF(delBallList);
    Py_DECREF(delBubbleList);
}

bool Ballpark::InDeadBubble(Ball *ball)
{
    if(!isMaster)
    {
        return false;
    }

    if (ball->mMode==DSTBALL_WARP)
    {
        // A warping ball is always considered in a live bubble
        return false;
    }

    PyObject *key = PyLong_FromLong(ball->mNewBubble);

    PyObject *interactives;
    Py_ssize_t interactiveCount = 0;
    // A bubble with one or more interactive balls is considered live
    if((interactives = PyDict_GetItem(bubbleInteractives, key)))
    {
        interactiveCount = PySet_Size(interactives);
        if(interactiveCount != 0)
        {
            Py_DECREF(key);
            return false;
        }
    }

    PyObject *keepAlivesSet;
    Py_ssize_t keepAlivesCount = 0;
    // A bubble with one or more keep-alive balls is considered live
    if((keepAlivesSet = PyDict_GetItem(bubbleKeepAlives, key)))
    {
        keepAlivesCount = PySet_Size(keepAlivesSet);
        if(keepAlivesCount != 0)
        {
            Py_DECREF(key);
            return false;
        }
    }

    Py_DECREF(key);
    return true;
}

size_t Ballpark::GetInteractiveCnt(Ball *ball)
{
    if(!isMaster)
        return 0;

    Py_ssize_t cnt = 0;

    PyObject *interactives;
    PyObject *key = PyLong_FromLong(ball->mNewBubble);

    if((interactives = PyDict_GetItem(bubbleInteractives, key)))
        cnt = PySet_Size(interactives);

    Py_DECREF(key);

    return cnt;
}
void Ballpark::SetBallRigid(
    const ID& srcId
)
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    Stop(ball);
    ball->SetMode(DSTBALL_RIGID);
}
void Ballpark::SetBallTroll(
    const ID& srcId,
    int delay
)
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    if(delay < 1)
        delay = 1;

    // Start by putting ball in known state
    Stop(ball);

    // Now make ball free
    SetBallFree(srcId, true);
    // And interactive
    SetBallInteractive(srcId, true);

    ball->mEffectStamp = mCurrentTime + delay;
    ball->SetMode(DSTBALL_TROLL);
}

bool Ballpark::TrollReady(Ball *ball)
{
    if(ball->mMode != DSTBALL_TROLL)
        return false;

    // Check if troll is due for petrification
    if(ball->mEffectStamp > mCurrentTime)
        return false;

    // Yep, this troll is ready
    return true;
}
void Ballpark::PetrifyTroll(Ball *ball)
{
    if(!TrollReady(ball))
        return;

    // Make ball non-free
    SetBallFree(ball->mId, false);
    // Make ball non-interactive
    SetBallInteractive(ball->mId, false);
    ball->SetMode(DSTBALL_RIGID);
}

void Ballpark::SetBallInteractive(
    const ID& srcId,
    bool flag
)
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    if(flag && !ball->isInteractive)
    {
        // becoming interactive
        ball->isInteractive = flag;
        IncreaseInteractiveCnt(ball, ball->mNewBubble);
    }
    else if(!flag && ball->isInteractive)
    {
        // becoming non-interactive
        DecreaseInteractiveCnt(ball, ball->mNewBubble);
        ball->isInteractive = flag;
    }
    else
    {
        ball->isInteractive = flag;
    }
}

void Ballpark::DecreaseInteractiveCnt(Partitionable *p, long inBubble)
{
    if(p->isInteractive)
    {
        Py_ssize_t val = 0;
        if(inBubble != -1)
        {
            PyObject *key = PyLong_FromLong(inBubble);
            PyObject *interactives;
            if((interactives = PyDict_GetItem(bubbleInteractives, key)))
            {
                PyObject *key2 = PyLong_FromLongLong(p->mId);
                PySet_Discard(interactives, key2);
                Py_DECREF(key2);
                val = PySet_Size(interactives);
                //CCP_LOGWARN_CH( s_chPark,"[%d] Decreasing count for ball %d in bubble %d", mCurrentTime, ball->mId, inBubble);

                if(val == 0)
                    PyDict_DelItem(bubbleInteractives, key);
            }

            Py_DECREF(key);
        }
    }
}

void Ballpark::UpdateInteractiveCnt(Partitionable *p, long oldBubble, long newBubble)
{
    if(!p->isInteractive)
        return;

    IncreaseInteractiveCnt(p, newBubble);
    DecreaseInteractiveCnt(p, oldBubble);
}

void Ballpark::IncreaseInteractiveCnt(Partitionable *p, long inBubble)
{
    if(p->isInteractive)
    {
        long val = 0;

        if(inBubble != -1)
        {
            //CCP_LOGWARN_CH( s_chPark,"[%d] Increasing count for ball %d in bubble %d", mCurrentTime, ball->mId, inBubble);
            PyObject *interactives;
            PyObject *key = PyLong_FromLong(inBubble);
            if(!(interactives = PyDict_GetItem(bubbleInteractives, key)))
            {
                interactives = PySet_New(NULL);
                PyDict_SetItem(bubbleInteractives, key, interactives);
                Py_DECREF(interactives);
            }

            PyObject *key2 = PyLong_FromLongLong(p->mId);
            PySet_Add(interactives, key2);
            Py_DECREF(key2);
            Py_DECREF(key);

        }
    }
}

void Ballpark::AddToBubble(Partitionable *p)
{
	if(!bubbles)
	{ 
		// Bubbles are kept as a dictionary, containing a map to ball IDs
		bubbles = PyDict_New();
	}

	PyObject *newBubbleId = PyLong_FromLong(p->mNewBubble);
	PyObject *oldBubbleId = PyLong_FromLong(p->mOldBubble);
	PyObject *ballId      = PyLong_FromLongLong(p->mId);

	if(p->mNewBubble != -1)
	{
		PyObject *newBubble = PyDict_GetItem(bubbles, newBubbleId);

		if(!newBubble){
			// If this bubble didn't exist, create it
			newBubble = PyDict_New();
			// Add the item to it
			PyDict_SetItem(bubbles, newBubbleId, newBubble);
			Py_DECREF(newBubble);  
		}
        
		PyObject *value = PyDict_GetItem(newBubble, ballId);

		if(!value)
		{
            // Need to add me to this thing
			PyDict_SetItem(newBubble, ballId, Plus);
		}
		else
		{
			// Already in bubble. Check how.
			long val = PyLong_AS_LONG(value);

			if(val==-1)
			{
				// Someone had deleted me. Nullify that.
				if(mProbe==p->mId)
					CCP_LOGWARN_CH( s_chPark,"[%d] Nullifying add in %d", mCurrentTime, p->mNewBubble);

				PyDict_SetItem(newBubble, ballId, Zero);
			}

		}

		// I have a new bubble and I'm certainly not in it. Add myself to it.

		if(mProbe==p->mId)
			CCP_LOGWARN_CH( s_chPark,"[%d] ball added to bubble %d", mCurrentTime, p->mNewBubble);

		// Check if someone is listening for these events
		if(PyDict_Size(mBubbleSubscriptions) > 0)
		{
			if(PyDict_GetItem(mBubbleSubscriptions, newBubbleId))
			{
				// Dispatch stuff
				if (!PyOS->SendEvent(
					(IEveBallpark*)this, "Destiny::DoBubbleAdd",
					"DoBubbleAdd", NULL, "(iL)", p->mNewBubble, p->mId
					))
				{
					PyOS->PyError();
				}
			}
		}
	}



	if(p->mOldBubble != -1)
	{
		PyObject *oldBubble = PyDict_GetItem(bubbles, oldBubbleId);

		if(oldBubble)
		{
			PyObject *value = PyDict_GetItem(oldBubble, ballId);

			if(value)
			{
				// Already in bubble. Check how.
				long val = PyLong_AS_LONG(value);

				if(val==0)
				{
					// The guy is here. Delete him
					if(mProbe==p->mId)
						CCP_LOGWARN_CH( s_chPark,"[%d] Deleting in %d", mCurrentTime, p->mOldBubble);

					PyDict_SetItem(oldBubble, ballId, Minus);
				}else if(val==1)
				{
					// The guy has just been added. Zap him completely.
					if(mProbe==p->mId)
						CCP_LOGWARN_CH( s_chPark,"[%d] Zapping in %d", mCurrentTime, p->mOldBubble);

					PyDict_DelItem(oldBubble, ballId);
				}

			}

			if(mProbe==p->mId)
				CCP_LOGWARN_CH( s_chPark,"[%d] ball removed from bubble %d", mCurrentTime, p->mOldBubble);

			// Check if someone is listening for these events
			if(PyDict_Size(mBubbleSubscriptions) > 0)
			{
				if(PyDict_GetItem(mBubbleSubscriptions, oldBubbleId))
				{
					// Dispatch stuff
					if (!PyOS->SendEvent(
						(IEveBallpark*)this, "Destiny::DoBubbleDel",
						"DoBubbleDel", NULL, "(iL)", p->mOldBubble, p->mId
						))
					{
						PyOS->PyError();
					}

				}
			}
		}
	}

	UpdateInteractiveCnt(p, p->mOldBubble, p->mNewBubble);
	UpdateKeepAliveBalls(p, p->mOldBubble, p->mNewBubble);

	Py_DECREF(newBubbleId);
	Py_DECREF(oldBubbleId);
	Py_DECREF(ballId);
}

//---------------------------------------------------------------------------------------
// InitializeBubbles recacalculates causal domains and assigns new bubble Ids
//---------------------------------------------------------------------------------------

void Ballpark::InitializeBubbles()
{
    BallIterator it(&mBalls);
    Ball *ball;
    Box *box;

    // First clear the current bubble flags
    mPartition->ClearTraversalFlag();

    // If the python dict of bubbles not initialized, do so now

    mBubbleId = 0;


	// Clear bubble tracking python members
	ClearBubbles();

    // Now cycle over all balls
    while((ball = it++))
    {
        box = ball->mMyBox;

        if(!box)
        {
            CCP_LOGERR_CH( s_chPark,"Ball without a box in InitializeBubbles\n");
            ball->mOldBubble = ball->mNewBubble;
            ball->mNewBubble = -1;
            continue;
        }

        box = mPartition->GetTopmost(box);

        if(box->mBubble != -1)
        {
            ball->mOldBubble = ball->mNewBubble;
            ball->mNewBubble = box->mBubble;

            AddToBubble(ball);
            continue; // aldready traversed
        }

        // We have a box which hasn't been traversed, so it must
        // belong to a new connected region. Seed fill it.
        box->Traverse(mBubbleId);


        ball->mOldBubble = ball->mNewBubble;
        ball->mNewBubble = mBubbleId;

        AddToBubble(ball);
 
        mBubbleId++; 
    }
}

//---------------------------------------------------------------------------------------
// GetEgo returns the current ball assigned as 'ego', possibly caching the pointer
//---------------------------------------------------------------------------------------

ClientBall* Ballpark::GetEgo()
{
    // If mEgo hasn't changed and I have a pointer then return that
    if( mEgo == mLastEgo && mLastEgoBall)
        return mLastEgoBall;

    // Either mEgo changed or the pointer is invalid
    mLastEgoBall = (ClientBall *)mBalls[mEgo];

    if(!mLastEgoBall)
    {
        // didn't find this ball, just return null
        mLastEgo = mEgo;
        mLastEgoBall = 0;
        return mLastEgoBall;
    }

    // Found it. Cache it.
    mLastEgo = mEgo;

    return mLastEgoBall;
}

//---------------------------------------------------------------------------------------
// InitializeFormations initializes formations static data
//---------------------------------------------------------------------------------------

void Ballpark::AddFormation(PyObject *vectorList)
{
    VectorOfVectors vecs;

    Py_ssize_t length = PyTuple_Size(vectorList);

    for(Py_ssize_t i = 0; i < length; i++)
    {
        Vector3d vec;
        PyObject *vector = PyTuple_GetItem(vectorList, i);

        vec.x = PyFloat_AsDouble(PyTuple_GetItem(vector,0));
        vec.y = PyFloat_AsDouble(PyTuple_GetItem(vector,1));
        vec.z = PyFloat_AsDouble(PyTuple_GetItem(vector,2));

        vecs.push_back(vec);
    }

    mFormations.resize(mFormations.size()+1);
    mFormations.back().swap(vecs);
}

void Ballpark::SetBallFormation(const ID& srcId, char formationID)
{
    Ball *ball = mBalls[srcId];

    if(!ball)
        return;

    if(formationID > int(mFormations.size())-1)
        return; // Illegal formation ID

    if(ball->mFormationID >= 0)
    {
        // Already in formation, need to change formation
        if(formationID == -1)
        {
            // Must stop all followers
            StopAllFollowers(ball);
            ball->mFormationSlots.reset();
        }
        else if(formationID != ball->mFormationID)
        {
            // This is a new formation. Discard ships with slots that don't fit
            size_t newLength = mFormations[formationID].size();
            if( mFormations[ball->mFormationID].size() > newLength)
            {
                // Cycle over followers and stop those with higher slots
                size_t numberOfFollowers = ball->mFollowers.size();
                VectorOfBalls stopCandidates;
                auto foll = ball->mFollowers.begin();
                auto end = ball->mFollowers.end();
                size_t i;
                for(; foll != end; ++foll)
                {
                    Ball *follower = mBalls[*foll];
                    if(follower->mMode != DSTBALL_FORMATION)
                        continue;

                    if(follower->mEffectStamp > (int)newLength - 1)
                        stopCandidates.push_back(follower);
                }

                for(i = 0; i < stopCandidates.size() ; i++)
                {
                    Stop(stopCandidates[i]);
                }

                // reset all bits above
                for(i = mFormations[ball->mFormationID].size() ; i < newLength ; i++)
                    ball->mFormationSlots.reset(i);
            }
        }
    }

    ball->mFormationID = formationID;

}

PyObject* Ballpark::PySetBallNotInParkCallback(
     PyObject* args
     )
{
    Py_XDECREF(Ballpark::s_ballNotInParkCallback);
    if (!PyArg_ParseTuple(args, "O", &Ballpark::s_ballNotInParkCallback))
    {
        Ballpark::s_ballNotInParkCallback = NULL;
    } else
        Py_INCREF( Ballpark::s_ballNotInParkCallback );

    Py_INCREF( Py_None );
    return Py_None;
}

void Ballpark::UpdateKeepAliveBalls(Partitionable *p, long oldBubble, long newBubble)
{
	PyObject* ballId = PyLong_FromLongLong( p->mId );
	if( !ballId )
	{
		CCP_LOGWARN( "Ballpark::UpdateKeepAliveBalls failed to convert ballId to PyLong" );
		PyErr_Clear();
		return;
	}
	ON_BLOCK_EXIT( [ballId] { Py_DECREF( ballId ); } );
	int containsResult = PySet_Contains( bubbleKeepAliveBalls, ballId );
	if( containsResult < 0 )
	{
		CCP_LOGWARN( "Ballpark::UpdateKeepAliveBalls failed to check membership in bubbleKeepAliveBalls" );
		PyErr_Clear();
		return;
	}
	if( !containsResult )
	{
		return;
	}
	AddKeepAliveBall( ballId, newBubble );
	RemoveKeepAliveBall( ballId, oldBubble );
}

void Ballpark::AddKeepAliveBall(PyObject *ballId, long inBubble)
{
    if(inBubble != -1)
    {
        PyObject *keepAliveBallsInBubbleSet;
        PyObject *bubbleId = PyLong_FromLong(inBubble);
		if(!(keepAliveBallsInBubbleSet = PyDict_GetItem(bubbleKeepAlives, bubbleId)))
        {
            keepAliveBallsInBubbleSet = PySet_New(NULL);
            PyDict_SetItem(bubbleKeepAlives, bubbleId, keepAliveBallsInBubbleSet);
			Py_DECREF(keepAliveBallsInBubbleSet);
        }
        PySet_Add(keepAliveBallsInBubbleSet, ballId);
        Py_DECREF(bubbleId);
    }
}

void Ballpark::RemoveKeepAliveBall(PyObject *ballId, long inBubble)
{
    if(inBubble != -1)
    {
        PyObject *bubbleId = PyLong_FromLong(inBubble);
        PyObject *keepAliveBallsInBubbleSet;
        if((keepAliveBallsInBubbleSet = PyDict_GetItem(bubbleKeepAlives, bubbleId)))
        {
            PySet_Discard(keepAliveBallsInBubbleSet, ballId);
            if(PySet_Size(keepAliveBallsInBubbleSet) == 0)
			{
                PyDict_DelItem(bubbleKeepAlives, bubbleId);
			}
        }
        Py_DECREF(bubbleId);
    }
}
