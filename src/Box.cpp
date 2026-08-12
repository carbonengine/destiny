// Copyright © 2014 CCP ehf.

#include "StdAfx.h"
#include "Box.h"
#include "SortedSets.h"
#include "StaticCollidable.h"
#include "Ball.h"

static CcpLogChannel_t s_chBox = CCP_LOG_DEFINE_CHANNEL("Box");

static int boxCount = 0;
//---------------------------------------------------------------------------------------
// Box Constructor
//---------------------------------------------------------------------------------------
Box::Box(int64_t _ix, int64_t _iy, int64_t _iz, long _level,Partition *_p)
{
	mLevel = _level;
	mPartition = _p;
	mParent = 0;
	mBubble = -1;
	mObject = 0;
	mHandled = false;
	// Find the number of grid divisions for this level
	// Basically mGridBase elevated to the power of levels
	int64_t n = mPartition->mLevelGrid[mLevel];

	mKey = Key(_ix,_iy,_iz,n);

	double boxWidth = mPartition->mLevelWidth[mLevel];
	double center = boxWidth*n*0.5f;
	// Define the bounding limits of this box
	lo[0] = mKey.ix*boxWidth - center;
	hi[0] = (mKey.ix+1)*boxWidth - center;

	lo[1] = mKey.iy*boxWidth - center;
	hi[1] = (mKey.iy+1)*boxWidth - center;

	lo[2] = mKey.iz*boxWidth - center;
	hi[2] = (mKey.iz+1)*boxWidth - center;

	SetParent();

	// insert the box in the map
	mPartition->mLevels[mLevel][mKey] = this;
}

//---------------------------------------------------------------------------------------
// Box Destructor
//---------------------------------------------------------------------------------------
Box::~Box()
{
	if(mParent)
		mParent->RemoveChildren(this);
	mPartition->mLevels[mLevel].erase(mKey);
	mChildren.clear();
	balls.clear();
	Py_XDECREF(mObject);
	mHandled = false;
}

void Box::ReleaseBox()
{
	mPartition->mBoxAllocator.ReleaseBox(this);
}




//---------------------------------------------------------------------------------------
// Box::SetParent
//---------------------------------------------------------------------------------------

void Box::SetObject(PyObject *object)
{
	if(object==mObject)
		return;

	if(mObject)
	{
		Py_DECREF(mObject);
		mObject = 0;
	}

	if(object)
	{
		Py_INCREF(object);
		mObject = object;
	}
}

//---------------------------------------------------------------------------------------
// Box::SetParent
//---------------------------------------------------------------------------------------

void Box::SetParent()
{
	if(mLevel == 0)
		return; // already top level box

	// get parent box
	mParent = mPartition->GetBox(mKey.ix >> 2 , mKey.iy >> 2 , mKey.iz >> 2  ,mLevel - 1);
	mParent->AddChildren(this);
}

//---------------------------------------------------------------------------------------
// Box::AddChildren
//---------------------------------------------------------------------------------------
void Box::AddChildren(Box *box)
{
	if(!box)
		return;

	mChildren.insert(box);
}

//---------------------------------------------------------------------------------------
// Box::AddBall
//---------------------------------------------------------------------------------------
void Box::AddBall(Ball *ball)
{
	if(!ball)
		return;

	balls.insert(ball);
}

//---------------------------------------------------------------------------------------
// Box::AddStaticCollidable
//---------------------------------------------------------------------------------------
void Box::AddStaticCollidable(StaticCollidable* collidable)
{
	if( !collidable )
	{
		return;
	}
	mStaticCollidables.insert(collidable);
}


//---------------------------------------------------------------------------------------
// Box::RemoveChildren
//---------------------------------------------------------------------------------------

void Box::RemoveChildren(Box *box)
{
	SortedSetOfBoxes::iterator it = mChildren.find(box);
	if (it != mChildren.end())
	{
		mChildren.erase(it); // children found, erase it
	}
	else
	{
		CCP_LOGERR_CH(s_chBox, "Box::RemoveChildren failure: [%d: %I64d, %I64d, %I64d] - Box [%d: %I64d, %I64d, %I64d] not found in mChildren",
			mLevel, mKey.ix, mKey.iy, mKey.iz,
			box->mLevel, box->mKey.ix, box->mKey.iy, box->mKey.iz);
	}

	if(mChildren.empty() && balls.empty() && mStaticCollidables.empty())
	{
		ReleaseBox();
	}
}

//---------------------------------------------------------------------------------------
// Box::RemoveBall
//---------------------------------------------------------------------------------------

void Box::RemoveBall(Ball *ball)
{
	SortedSetOfBalls::iterator it = balls.find(ball);
	if (it != balls.end())
	{
		balls.erase(it); // ball found, erase it
	}
	else
	{
		CCP_LOGERR_CH(s_chBox, "Box::RemoveBall failure: [%d: %I64d, %I64d, %I64d] - Ball %I64d (%I64d) not found in balls",
			mLevel, mKey.ix, mKey.iy, mKey.iz, ball->mId, ball->mOwnerId);
	}

	if(mChildren.empty() && balls.empty() && mStaticCollidables.empty())
	{
		ReleaseBox();
	}
}

//---------------------------------------------------------------------------------------
// Box::RemoveStaticCollidable
//---------------------------------------------------------------------------------------

void Box::RemoveStaticCollidable(StaticCollidable *collidable)
{
	SortedSetOfStaticCollidables::iterator it = mStaticCollidables.find(collidable);
	if (it != mStaticCollidables.end())
	{
		mStaticCollidables.erase(it);
	}
	else
	{
		CCP_LOGERR_CH(s_chBox, "Box::RemoveStaticCollidable failure: [%d: %I64d, %I64d, %I64d] - StaticCollidable %I64d (%I64d) not found in mStaticCollidables",
			mLevel, mKey.ix, mKey.iy, mKey.iz, collidable->mId, collidable->mParentBallId);
	}

	if(mChildren.empty() && balls.empty() && mStaticCollidables.empty())
	{
		ReleaseBox();
	}
}

//---------------------------------------------------------------------------------------
// Box::Traverse
//
// Seed fill space partition at same level as this box with the given seed
//---------------------------------------------------------------------------------------

void Box::Traverse(long seed)
{
	if(mBubble != -1)
		return; // I have already been traversed

	// I have not been traversed. Mark myself traversed and then seed all neighbors
	mBubble = seed;

	int i,j,k;

	// Check all my 26 neighbors
	for(i= -1; i < 2; i++)
	{
		for(j= -1; j < 2; j++)
		{
			for(k= -1; k < 2; k++)
			{
				if(i==0 && j==0 && k==0)
					continue;

				Box *box = mPartition->CheckOffsetBox(this,i,j,k);
				if(box)
					box->Traverse(seed);
			}
		}
	}

	return;
}

//---------------------------------------------------------------------------------------
// Box::NearbyBubbles
//
// Seed fill space partition at same level as this box with the given seed
//---------------------------------------------------------------------------------------

long Box::NearbyBubbles()
{

	int i,j,k;

	// Check all my 26 neighbors
	for(i= -1; i < 2; i++)
	{
		for(j= -1; j < 2; j++)
		{
			for(k= -1; k < 2; k++)
			{
				if(i==0 && j==0 && k==0)
					continue;

				Box *box = mPartition->CheckOffsetBox(this,i,j,k);
				if(box && box->mBubble != -1)
					return box->mBubble;
			}
		}
	}

	return -1;
}


bool BoxSortComparer::operator () (const Box* x, const Box* y) const
{
	// returns true if X precedes and is not equal to Y in the sort order.
	return x->mKey < y->mKey;
}
