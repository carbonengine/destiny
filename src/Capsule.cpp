// Copyright © 2015 CCP ehf.

#include "StdAfx.h"
#include "Capsule.h"
#include "Collision.h"
#include "Ball.h"
#include "Ballpark.h"
#include "Settings.h"


void Capsule::Initialize(ID theID, ID parentObjectId, double ax, double ay, double az, double bx, double by, double bz, float radius)
{
	mId = theID;
	mParentBallId = parentObjectId;
	mHemisphereA.x = ax;
	mHemisphereA.y = ay;
	mHemisphereA.z = az;
	mHemisphereB.x = bx;
	mHemisphereB.y = by;
	mHemisphereB.z = bz;
	mRadius = radius;
	mAB = mHemisphereB - mHemisphereA;
	mOldPos = mNewPos = GetCenter();
}

float Capsule::GetBoundingRadius()
{
	return (float)mAB.Length() * 0.5f + mRadius;
}

Vector3d Capsule::GetCenter()
{
	return (mHemisphereA + mHemisphereB) * 0.5;
}


bool Capsule::CheckCollision(const Vector3d& p0, const Vector3d& p1, float radius, Vector3d& normal, double& timeOfImpact)
{
	// The bounding radius + sphere radius
	const double boundingRadius = static_cast<double>(GetBoundingRadius()) + static_cast<double>(radius);

	// Make a cheap check for intersection of path and bounding sphere
	const auto path = p1 - p0;
	const double pathsq = path * path;
	const auto separation = mNewPos - p0;
	const auto separationEnd = mNewPos - p1;
	const double distanceToClosest = separation * path;


	// No intersection if distance is negative or greater than pathLength
	if ((distanceToClosest < 0 && boundingRadius*boundingRadius < separation*separation)
		|| (distanceToClosest > pathsq && boundingRadius*boundingRadius < separationEnd*separationEnd)
		)
		return false;

	// This is the closest point squared, there is a hit if it is smaller than radius squared
	// All the distances here are multiplied with pathsq to avoid divisions
	const auto distsq = pathsq * separation.LengthSq() - distanceToClosest * distanceToClosest;
	if (distsq > pathsq * boundingRadius * boundingRadius)
		return false;

	// Now check for the actual capsule
	// First we make sure there is a cylinder.
	const double sqLengthAB = mAB.LengthSq();
	if (sqLengthAB < 1e-10)
	{
		// We are colliding two spheres, bounding radius is the actual radius

		double s = CollideTwoSpheres(p0, p1, mNewPos, mNewPos, boundingRadius);

		if (s >= 0.0)
		{
			// Use the approximate position at impact to set the normal
			timeOfImpact = s;
			normal = p0 + s * path - mNewPos;

			// Normalize, making sure we have a valid normal
			const auto normalLength = normal.Length();
			if (normalLength < 1e-10)
			{
				normal = Vector3d(0.0, 0.0, 1.0);
			}
			else
			{
				normal /= normalLength;
			}

			return true;
		}

		return false;
	}

	const double comparisonRadius = static_cast<double>(radius) + static_cast<double>(mRadius);

	const auto invSqLengthAB = 1. / sqLengthAB;
	const auto lengthAB = std::sqrt(sqLengthAB);

	const auto ao = p0 - mHemisphereA;  // From A to center of ball at the beginning.
	const auto paraAO = ao * mAB / lengthAB; // The parallel part of the distance in the beginning
	const auto perpPath = path - (path * mAB) * mAB * invSqLengthAB;  // The component of the path perpendicular to the cylinder.
	const auto perpAO = ao - (ao * mAB) * mAB * invSqLengthAB;  // The perpendicular part of the start

	// Start by checking for a start within the cylinder
	const double c = perpAO * perpAO - comparisonRadius * comparisonRadius;

	// if c is negative, we are starting within the infinite cylinder
	if (c <= 0.0)
	{
		// Check if we are starting within the cylinder
		if (paraAO >= 0.0 && paraAO <= lengthAB)
		{
			timeOfImpact = 0.0;

			//Move away from the center
			normal = perpAO;
			normal.Normalize();
			return true;
		}

		// Find which hemisphere we are possibly colliding with
		Vector3d center = paraAO < 0.0 ? mHemisphereA : mHemisphereB;

		const auto s = CollideTwoSpheres(p0, p1, center, center, comparisonRadius);
		if (s >= 0.0)
		{
			// Use the approximate position at impact to set the normal
			timeOfImpact = s;
			normal = p0 + s * path - center;

			// We cannot be exactly on the center, that would be within the cylinder
			normal.Normalize();

			return true;
		}

		// We are not in collision
		return false;
	}


	// Find if the path intersects
	const double a = perpPath * perpPath;

	// We are nearly parallel to the cylinder
	if (a < 1e-10)
	{
		// We have already checked for start within the cylinder
		return false;
	}

	// Check for collision with the cylinder
	const auto b = 2.0 * (perpPath * perpAO);

	double s1, s2;

	// Check for solution
	if (!Quadratic(s1, s2, a, b, c))
	{
		return false;
	}

	// We have already checked for start within cylinder, so only need to make sure 
	// s2 is between 0 and 1
	if (s2 >= 0.0 && s2 <= 1.0)
	{
		// Find the vector from A to the point of collision
		const auto pc = p0 + s2 * path - mHemisphereA;

		// Make sure we hit the cylinder between top and bottom
		const auto pcHeight = pc * mAB / lengthAB;

		if (pcHeight >= 0.0 && pcHeight <= lengthAB)
		{
			// Collision with the cylinder
			timeOfImpact = s2;

			// Normal is in the direction away from the center
			normal = pc - (pc * mAB) * mAB * invSqLengthAB;
			normal.Normalize();

			return true;
		}

		// Check for collision with the closer sphere
		Vector3d center = pcHeight < 0 ? mHemisphereA : mHemisphereB;

		const auto s = CollideTwoSpheres(p0, p1, center, center, comparisonRadius);
		if (s >= 0.0)
		{
			// Use the approximate position at impact to set the normal
			timeOfImpact = s;
			normal = p0 + s * path - center;

			// We cannot be exactly on the center
			normal.Normalize();

			return true;
		}

	}

	return false;
}

void Capsule::HandleCollisionNonIteratively(Ball* ball, Vector3d p0, Vector3d p1, double m1, Vector3d vp1)
{
	double rad = (double)ball->mRadius + (double)mRadius;
	Vector3d d = vp1;
	Vector3d AO = p0 - mHemisphereA; // From A to the center of the ball
	double m = mAB * d / (mAB * mAB);
	double n = mAB * AO / (mAB * mAB);
	Vector3d Q = d - (mAB * m);
	Vector3d R = AO - (mAB * n);

	double a = Q * Q;
	double b = 2.0 * (Q * R);
	double c = R * R - rad * rad;
	Vector3d normal;

	double radSq = rad * rad;

	double s1, s2, s, timeOfImpact;
	s = -1.0;
	timeOfImpact = -1.0;

	if (a == 0.0)
	{
		// If a is zero, the ray is parallel to the cylinder axis (AB).
		// Check for collision with the A side cap.
		double s = CollideTwoSpheres( p0, p1, mHemisphereA, mHemisphereA, rad );
		if (s != -1.0 && (timeOfImpact == -1.0 || s < timeOfImpact))
		{
			timeOfImpact = s;
			normal = p0 - mHemisphereA;
		}


		// Check for collision with the B side cap.
		s = CollideTwoSpheres( p0, p1, mHemisphereB, mHemisphereB, rad );
		if (s != -1.0 && (timeOfImpact == -1.0 || s < timeOfImpact))
		{
			timeOfImpact = s;
			normal = p0 - mHemisphereB;
		}
		ReactToCollision( ball, p1, vp1, m1, normal, timeOfImpact );
		return;
	}

	if (!Quadratic(s1, s2, a, b, c))
	{
		return;
	}

	if (s1 >= 0.0 && s1 <= 1.0)
	{
		s = s1;
	}
	if (s2 >= 0.0 && s2 <= 1.0 && s2 < s1)
	{
		s = s2;
	}
	else if (s == -1.0)
	{
		// We don't collide with the cylinder
		double abSq = mAB.LengthSq();

		// Are we already inside  the cylinder ? Sphere checks already determine if we are already inside the spheres.
		const double t = ( p0 - mHemisphereA ) * mAB / abSq;
		if (t >= 0.0 && t <= 1.0)
		{
			const Vector3d proj = mHemisphereA + t * mAB;
			Vector3d dist = p0 - proj;
			if (dist.LengthSq() < radSq)
			{
				// p0 was inside the cylinder
				timeOfImpact = 0.0;
				normal = dist;
				vp1 = ball->mNewVel;
				ReactToCollision( ball, p0, vp1, m1, normal, timeOfImpact );
				return;
			}
		}
	}

	double tK1 = (s1 * m) + n;  // Where on the scale from 0 to 1 does the first collision occur on the cylinder from A to B
	double tK2 = (s2 * m) + n;  // Same for the second collision

	if (tK1 < 0.0)
	{
		// Check for collision with the A side cap.
		double s = CollideTwoSpheres( p0, p1, mHemisphereA, mHemisphereA, rad );
		if (s != -1.0 && (timeOfImpact == -1.0 || s < timeOfImpact))
		{
			timeOfImpact = s;
			normal = p0 - mHemisphereA;
		}
	}
	else if (tK1 > 1.0)
	{
		// Check for collision with the B side cap.
		double s = CollideTwoSpheres( p0, p1, mHemisphereB, mHemisphereB, rad );
		if (s != -1.0 && (timeOfImpact == -1.0 || s < timeOfImpact))
		{
			timeOfImpact = s;
			normal = p0 - mHemisphereB;
		}
	}
	else
	{
		if (s != -1.0 && (timeOfImpact == -1.0 || s < timeOfImpact))
		{
			timeOfImpact = s;
			Vector3d projAbAo = (AO * mAB / (mAB.LengthSq())) * mAB;
			normal = AO - projAbAo;

			if (normal.LengthSq() < radSq)
			{
				timeOfImpact = 0.0;
			}
		}
	}

	if (tK2 < 0.0)
	{
		// Check for collision with the A side cap.
		double s = CollideTwoSpheres( p0, p1, mHemisphereA, mHemisphereA, rad );
		if (s != -1.0 && (timeOfImpact == -1.0 || s < timeOfImpact))
		{
			timeOfImpact = s;
			normal = p0 - mHemisphereA;
		}

	}
	else if (tK2 > 1.0)
	{
		// Check for collision with the B side cap.
		double s = CollideTwoSpheres( p0, p1, mHemisphereB, mHemisphereB, rad );
		if (s != -1.0 && (timeOfImpact == -1.0 || s < timeOfImpact))
		{
			timeOfImpact = s;
			normal = p1 - mHemisphereB;
		}
	}
	else
	{
		if (s != -1.0 && (timeOfImpact == -1.0 || s < timeOfImpact))
		{
			timeOfImpact = s;
			Vector3d projAbAo = (AO * mAB / (mAB.LengthSq())) * mAB;
			normal = AO - projAbAo;
			if (normal.LengthSq() < radSq)
			{
				timeOfImpact = 0.0;
			}
		}
	}


	// We have impact. Now react.
	vp1 = ball->mNewVel;
	ReactToCollision( ball, p0, vp1, m1, normal, timeOfImpact );
}

void Capsule::CollideWithBall(Ball* ball)
{
	if (!ball->isFree)
	{
		return;
	}
	Vector3d p0, p1, vp1;
	double m1 = ball->mMass * ball->mAgility;

	p0 = p1 = ball->mNewPos;
	vp1 = ball->mNewVel;

	// Get the destination point for the ball
	mPark->Integrate(p1, vp1, ball->mLastG, m1, mPark->mFriction, ball->mTimeFactor, mPark->dt);

	if( g_useIterativeCollision )
	{
		// Calculate the normal
		Vector3d normal;
		double timeOfImpact;
		if (CheckCollision(p0, p1, ball->mRadius, normal, timeOfImpact))
			ReactToCollision(ball, p1, vp1, m1, normal, timeOfImpact);
	}
	else
	{
		// Calculate if and when collision occurs
		HandleCollisionNonIteratively(ball, p0, p1, m1, vp1);
	}
}

void Capsule::ReactToCollision(Ball* ball, Vector3d& ballPosition, Vector3d& ballVelocity, double m1, Vector3d& normal, double timeOfImpact)
{
	if (timeOfImpact == -1.0)
	{
		return;
	}
	double centerDistance = normal.Length();
	normal.Normalize();
	Vector3d acceleration;
	double normalComp = ball->mLastG*normal;
	if (timeOfImpact > 0.0)
	{
		if (normalComp > 0.0)
		{
			// We are colliding on our way out of the object
			return;
		}

		// This is the simple case of someone hitting a fixed object. Speed is thus inverted along the normal.
		mPark->Integrate(ballPosition, ballVelocity, ball->mLastG, m1, mPark->mFriction, ball->mTimeFactor, timeOfImpact*mPark->dt);
		double v1 = ballVelocity * normal;
		ballVelocity -= 2.0*v1*normal;

		mPark->Integrate(ballPosition, ballVelocity, ball->mLastG, m1, mPark->mFriction, ball->mTimeFactor, (1.0 - timeOfImpact)*mPark->dt);
		// In order to get there, my acceleration needs to be:
		double k = mPark->mFriction;
		acceleration = -(-m1 * ball->mLastG + ball->mTimeFactor*m1*ball->mLastG - ball->mTimeFactor*ball->mNewVel*k + ballVelocity * k) / m1 / (ball->mTimeFactor - 1.0);
	}
	else
	{
		// Calculate the acceleration
		double tmp = 1.0 / (m1 + mPark->dt*mPark->mFriction);
		double dist = (double)mRadius + (double)ball->mRadius - centerDistance + 1.0; // The distance that we need to move the ball to get it out.
		acceleration = ((dist / (mPark->dt*tmp*m1))*normal - ball->mNewVel) / mPark->dt - normalComp * normal;
	}
	Vector3d lastC = 0.85 * acceleration;

	if (timeOfImpact == ball->mLastCollision)
	{
		// Use the stronger collision of the two
		if (lastC.LengthSq() > ball->mLastC.LengthSq())
		{
			ball->mLastC = lastC;
		}
	}
	else
	{
		ball->mLastC = lastC;
		ball->mLastCollision = timeOfImpact;
	}

	ball->mCollisions.push_back(mId);
}

void Capsule::InsertInBoxes(Box* box1, Box* top, long newBubbleId)
{
	Box *box2;

	if (mMyBox && box1->mKey == mMyBox->mKey)
	{
		// We have the same base box as before.
		return;
	}

	if( mRadius <= 0.0f )
	{
		// Capsule doesn't have any volume. No need to insert it in boxes.
		return;
	}

	// Now we know all the boxes we might have to add
	// We assume that when both modifiers are non-null, then the diagonal
	// box should be added as well. This is justified by the fact that it usually
	// very close anyway.

	// Let's run over all the boxes the ball currently thinks it is in and remove it from there
	// Note that this might invalidate the top box and its bubble, but we will restore it
	DeleteFromBoxes();

	// The box we had might well have been deleted. Create it again.
	box1 = mPark->mPartition->GetBox(mRadius, mNewPos);
	Box* boxA = mPark->mPartition->GetBox(mRadius, mHemisphereA);

	int64_t ix, iy, iz;


	long level = boxA->mLevel;

	if (mPark->isMaster)
	{
		// Restore the topmost bubbleID in case it got lost
		top = mPark->mPartition->GetTopmost(box1);
		top->mBubble = newBubbleId;
	}

	mMyBox = box1;

	// Here we determine which neighbouring boxes the ball might overlap, and insert accordingly
	int q;
	short mod[3];
	Vector3d interpolatedPos = mHemisphereA;
	Vector3d direction = mHemisphereB - mHemisphereA;
	float length = (float)direction.Length();
	direction = direction.Normalize() * (double)mRadius * 2.0f;
	while (length >= 0.0f)
	{
		boxA = mPark->mPartition->GetBox(mRadius, interpolatedPos);
		ix = boxA->mKey.ix;
		iy = boxA->mKey.iy;
		iz = boxA->mKey.iz;

		for (q = 0; q < 3; q++)
		{
			if (boxA->lo[q] + (double)mRadius - interpolatedPos[q] > 0.0f)
			{
				// Add box to the left...
				mod[q] = -1;
			}
			else if (boxA->hi[q] - (double)mRadius - interpolatedPos[q] < 0.0f)
			{
				// add box  to the right
				mod[q] = 1;
			}
			else
			{
				// don't need to include any adjacent box
				mod[q] = 0;
			}
		}

		box2 = mPark->mPartition->GetBox(ix, iy, iz, level);
		InsertInBox(box2);

		// Add box in the x dimension
		if (mod[0])
		{
			box2 = mPark->mPartition->GetBox(ix + mod[0], iy, iz, level);
			InsertInBox(box2);
		}

		// Add box in the y dimension
		if (mod[1])
		{
			box2 = mPark->mPartition->GetBox(ix, iy + mod[1], iz, level);
			InsertInBox(box2);
		}

		// Add box in the z dimension
		if (mod[2])
		{
			box2 = mPark->mPartition->GetBox(ix, iy, iz + mod[2], level);
			InsertInBox(box2);
		}

		// Add the corner boxes
		// xy corner
		if (mod[0] && mod[1])
		{
			box2 = mPark->mPartition->GetBox(ix + mod[0], iy + mod[1], iz, level);
			InsertInBox(box2);
		}

		// xz corner
		if (mod[0] && mod[2])
		{
			box2 = mPark->mPartition->GetBox(ix + mod[0], iy, iz + mod[2], level);
			InsertInBox(box2);
		}

		// yz corner
		if (mod[1] && mod[2])
		{
			box2 = mPark->mPartition->GetBox(ix, iy + mod[1], iz + mod[2], level);
			InsertInBox(box2);
		}

		// xyz corner
		if (mod[0] && mod[1] && mod[2])
		{
			box2 = mPark->mPartition->GetBox(ix + mod[0], iy + mod[1], iz + mod[2], level);
			InsertInBox(box2);
		}

		interpolatedPos = interpolatedPos + direction;
		length -= mRadius * 2.0f;
	}
}