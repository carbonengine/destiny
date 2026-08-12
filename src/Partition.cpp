// Copyright © 2014 CCP ehf.

#include "StdAfx.h"
#include "Partition.h"
#include "Box.h"
#include "Ball.h"
#include "Ballpark.h"

static CcpLogChannel_t s_chPart = CCP_LOG_DEFINE_CHANNEL( "Partition" );


// constant filter objects for getting nearby balls corresponding to the two static collision cases: 
//  1) exclude cloaked balls only, and 3) exclude cloaked and missile balls. See GetCollisionCandidates
Partition::NearbyCriteria collisionCase1(1), collisionCase3(3);

//---------------------------------------------------------------------------------------
// Partition constructor
//---------------------------------------------------------------------------------------
Partition::Partition():
    mNumberOfLevels(8),
    mGridUnit(480.0) // base unit of small-scale partition
{
	
    // create as many grids as there are scales in the partition
    // Initialize lookup vectors
    double bigbox = (1 << (2*mNumberOfLevels))*mGridUnit*0.25;
    double width;
    int64_t grid;
    for(int i=0; i < mNumberOfLevels; i++)
    {
        mLevels.push_back(MapOfBoxes());
        width = bigbox/(1 << 2*i);
        mLevelWidth.push_back( width );
        grid = 1;
        grid = grid << (2*i - 2*mNumberOfLevels + 39);

        // Note that the following is pretty much designed for the 15.0 meter level to scan a 200 AU sphere
        mLevelGrid.push_back( grid );
    }
}

void Partition::GetBoxKey(const Vector3d& pos, long level, int64_t& ix, int64_t& iy, int64_t& iz)
{
    if (level >= mNumberOfLevels || level < 0)
    {
        ix = 0;
        iy = 0;
        iz = 0;
        return;
    }

    double boxWidth = mLevelWidth[level];
    double fac = 1.0 / boxWidth;
    int64_t n = mLevelGrid[level];

    ix = int64_t((pos.x + 0.5*boxWidth*n)*fac);
    iy = int64_t((pos.y + 0.5*boxWidth*n)*fac);
    iz = int64_t((pos.z + 0.5*boxWidth*n)*fac);
}

//---------------------------------------------------------------------------------------
// Box Factory numero uno
//---------------------------------------------------------------------------------------
Box* Partition::GetBox(int64_t ix, int64_t iy, int64_t iz,  long level)
{
    Box *box;

    if(level >= mNumberOfLevels || level < 0)
        return 0;    // Illegal level

    // Find the number of grid divisions for this level
    // Basically mGridBase elevated to the power of levels
    int64_t n = mLevelGrid[level];

    //CCP_LOG_CH( s_chPart,"Requesting box [%I64d,%I64d,%I64d] at level %d\n",ix,iy,iz,level);

    // First check if this box already exists
    box = mLevels[level][ Key(ix,iy,iz,n) ];

    // If this box doesn't exist, create it
    if(!box)
        box =  mBoxAllocator.GetNewBox(ix,iy,iz,level,this);

    return box;
}

Box* Partition::CheckBox(int64_t ix, int64_t iy, int64_t iz,  long level)
{
    if(level >= mNumberOfLevels || level < 0)
        return 0;    // Illegal level

    // Find the number of grid divisions for this level
    // Basically mGridBase elevated to the power of levels
    int64_t n = mLevelGrid[level];

    MapOfBoxes::iterator srcIt = mLevels[level].find(Key(ix,iy,iz,n));

    if(srcIt==mLevels[level].end())
        return 0;

    return srcIt->second;
}

//---------------------------------------------------------------------------------------
// Box Factory numero dos
//---------------------------------------------------------------------------------------

Box* Partition::GetBox(double radius, const Vector3d& pos, bool create)
{
    // Find myself a level
    int level = 0;

    for(int i=0; i < mNumberOfLevels; i++)
    {
        if(3.0*radius > mLevelWidth[i])
            break;

        level = i;
    }

    double boxWidth = mLevelWidth[level];
    double fac = 1.0/boxWidth;
    int64_t n = mLevelGrid[level];

    // Given that level, lets find where we are positioned
    int64_t ix = int64_t((pos.x+0.5*boxWidth*n)*fac);
    int64_t iy = int64_t((pos.y+0.5*boxWidth*n)*fac);
    int64_t iz = int64_t((pos.z+0.5*boxWidth*n)*fac);

    if(create)
    {
        return GetBox(ix,iy,iz,level);
    }
    else
    {
        return CheckBox(ix,iy,iz,level);
    }
}

//---------------------------------------------------------------------------------------
// GetExistingTopBox returns the top partition box for the coordinate, if it exists
//---------------------------------------------------------------------------------------
Box* Partition::GetExistingTopBox(const Vector3d& pos)
{
    return GetBox(10000000.0, pos, false);
}

//---------------------------------------------------------------------------------------
// Returns the center of the box corresponding to given coordinate for given level
//---------------------------------------------------------------------------------------

void Partition::GetBoxCenter(int level,  Vector3d& pos)
{
    double boxWidth = mLevelWidth[level];
    double fac = 1.0/boxWidth;
    int64_t n = mLevelGrid[level];

        // Given that level, lets find where we are positioned
    pos.x = int64_t((pos.x+0.5*boxWidth*n)*fac)*boxWidth + boxWidth*0.5 - boxWidth*n*0.5;
    pos.y = int64_t((pos.y+0.5*boxWidth*n)*fac)*boxWidth + boxWidth*0.5 - boxWidth*n*0.5;
    pos.z = int64_t((pos.z+0.5*boxWidth*n)*fac)*boxWidth + boxWidth*0.5 - boxWidth*n*0.5;
}

//---------------------------------------------------------------------------------------
// GetSize returns the total number of active boxes at all levels
//---------------------------------------------------------------------------------------

int Partition::GetSize()
{
    size_t sum = 0;
    for(unsigned i = 0;  i < mLevels.size() ; i++)
    {
        sum += mLevels[i].size();
    }

    return int(sum);
}

//---------------------------------------------------------------------------------------
// GetTopmost returns the topmost box containing the given box
//---------------------------------------------------------------------------------------

Box* Partition::GetTopmost(Box *box)
{
    if(!box)
        return 0;

    // Climb up the parent hierarchy
    while(box->mParent)
        box = box->mParent;

    // we are at the summit
    return box;
}

//---------------------------------------------------------------------------------------
// GetOffsetBox returns a box that is at the offset from the given box
//---------------------------------------------------------------------------------------

Box* Partition::GetOffsetBox(Box *box,int64_t i,int64_t j,int64_t k)
{
    if(!box)
        return 0;

    return GetBox(box->mKey.ix + i,box->mKey.iy + j,box->mKey.iz + k,box->mLevel);
}

//---------------------------------------------------------------------------------------
// CheckOffsetBox returns the box at the offset from the given box iff it already exists
//---------------------------------------------------------------------------------------

Box* Partition::CheckOffsetBox(Box *box,int64_t i,int64_t j,int64_t k)
{
    if(!box)
        return 0;

    return CheckBox(box->mKey.ix + i,box->mKey.iy + j,box->mKey.iz + k,box->mLevel);
}

//---------------------------------------------------------------------------------------
// ClearTraversalFlag clears the bubble flag of all topmost boxes
//---------------------------------------------------------------------------------------

void Partition::ClearTraversalFlag()
{
    MapOfBoxes::iterator boxIt;

    for(boxIt = mLevels[0].begin() ; boxIt != mLevels[0].end(); ++boxIt)
    {
        (boxIt->second)->mBubble = -1;
    }
}


//---------------------------------------------------------------------------------------
// DumpPartition debug logs the content of all partitions
//---------------------------------------------------------------------------------------

void Partition::DumpPartition()
{
    for(int i=0; i < mNumberOfLevels;i++)
    {
        MapOfBoxes::iterator it;
        for(it = mLevels[i].begin() ; it != mLevels[i].end() ; ++it)
        {
            Box *box = it->second;
            CCP_LOG_CH( s_chPart,"Level %d: %d balls and %d children in Box [%I64d,%I64d,%I64d]\n",i,box->balls.size(),box->mChildren.size(),box->mKey.ix,box->mKey.iy,box->mKey.iz);
        }
    }
}

bool isNotMissile(Ball* ball)
{
    return (ball->mMode != DSTBALL_MISSILE);
}


//---------------------------------------------------------------------------------------
// ExplodeChildren recursively traverses a box's children list, accumuluting boxes on the way.
//---------------------------------------------------------------------------------------

void Partition::ExplodeChildren(SortedSetOfBoxes &children, BoxVectorInsertor &ins)
{
	SortedSetOfBoxes::iterator it;
    Box *box;

    for(it = children.begin(); it != children.end() ; ++it)
    {
        box = *it;
		ExplodeChildren(box->mChildren, ins);

        if(!box->balls.empty() || !box->mStaticCollidables.empty())
        {
			ins = box;
        }
    }
}


/*
    Wraps GetNearbyBalls for the context of proximity sensors
*/
void Partition::GetProximityCandidates(Ball* ball, VectorOfBalls& uni, VectorOfStaticCollidables& uniStat, bool isMaster)
{
    // Find all free balls that are 'close' to the proximity sensor, including missiles
    // Note that if ball is cloaked we want fixed balls, otherwise not. Notice also that for 
    // the purpose of proximity events, we want to include cloaked balls and disregard harmonics.
    NearbyCriteria filter;
    filter.EXCLUDE_HARMONICS_TEST = 1;
    if(!ball->isCloaked)
        filter.EXCLUDE_FIXED_BALLS = 1;
    if(ball->mSensor.onlyInteractives)    
        filter.EXCLUDE_INERT_BALLS = 1;
      
    GetNearbyBalls(ball, uni, uniStat, isMaster, filter);
}


/*
    Wraps GetNearbyBalls for the context of collision detection
*/
void Partition::GetCollisionCandidates(Ball* ball, VectorOfBalls& uni, VectorOfStaticCollidables& uniStat, bool isMaster)
{
    // What we want:
    // 1. Normal objects see everything, except missiles and cloaked balls
    // 2. Normal missiles only see the ball it targets 
    // 3. Defender missiles see everything except cloaked balls.
    if(ball->mMode==DSTBALL_MISSILE)
    {
        if(ball->mOwnerId < 0)
        {   // This is case 3
            GetNearbyBalls(ball, uni, uniStat, isMaster, collisionCase3);
        }
        else if(ball->mNewBubble==ball->mFollowPtr->mNewBubble) 
        {  // This bubble check doesn't really do much as the missile ball shouldn't be tracking something in a different bubble anyways (it would break the client if it did)
           // This is case 2
            uni.push_back(ball->mFollowPtr);
        }
    }
    else
    {   // This is case 1
        GetNearbyBalls(ball, uni, uniStat, isMaster, collisionCase1);
    }
}


/*---------------------------------------------------------------------------------------
        GetNearbyBalls:

        - For each box that the ball is in:
            - recursively includes all its parent boxes iff there are ball in them
            - recursively includes all its children boxes iff there are balls in them
            - include the box itself iff there are more balls than the ball itself

        - For each box in this list of boxes:
            - Cycle over all balls in the box and include those that:
                - are not the ball itself
                - are in the same bubble and not global (iff isMaster)
                - do not have the same harmonic as the ball itself (unless EXCLUDE_HARMONICS is specified)
                - aren't excluded by a flag in 'filter'

        - Given that list of balls it is sorted and duplicate balls removed
---------------------------------------------------------------------------------------*/
void Partition::GetNearbyBalls(Ball* ball, VectorOfBalls& uni, VectorOfStaticCollidables& uniStat, bool isMaster, NearbyCriteria filter)
{
	SortedSetOfBoxes::iterator it;
    VectorOfBoxes::iterator lt;
	SortedSetOfBalls::iterator jt;
    Box *box,*save;

    if(!ball)
        return;
    // First, find all boxes up and down the partition hierarchy
    // that are involved with this ball


    if(filter.EXCLUDE_INERT_BALLS)
    {
        // No need to search for anything if there aren't any interactives around
        if(ball->mPark->GetInteractiveCnt(ball)==0)
            return;
    }

    // Keep a static vector for accumulating boxes and clear it after we're done.
	// This vector will grow to the largest set of boxes ever needed, which should
	// happen fairly quickly. After that, there won't be any memory allocations needed
	// and that is a worthwhile optimization.
	static VectorOfBoxes s_boxes;
	ON_BLOCK_EXIT( [=] { s_boxes.clear(); } );

    BoxVectorInsertor ins(s_boxes);

    for(it = ball->mBoxes.begin(); it != ball->mBoxes.end(); ++it)
    {
        save = box = *it;

        if(box->mHandled)
            continue;

        // get all the parents
        while(box->mParent)
        {
            box = box->mParent;
            if(box->mHandled)
                break;

            if(!box->balls.empty() || !box->mStaticCollidables.empty()) // include parent box only if it has any collidable objects
            {
                //only include those parents that have balls that are not missiles
                ins = box;
                box->mHandled = true;
            }
        }

        box = save;

        // Now include all children and their children ad infinitum
        ExplodeChildren(box->mChildren, ins);

        if(box->balls.size() > 1 || box->mStaticCollidables.size() > 0)
        {
            ins = box; // only include box itself if there are some other collidable in there
            box->mHandled = true;
        }

    }

    // Now go over all these boxes and pick out the balls

    Ball *otherBall;

    for(lt = s_boxes.begin(); lt != s_boxes.end(); ++lt)
    {
        // cycle over all boxes that 'ball' belongs to
        box = *lt;
        box->mHandled = false;
        for(jt = box->balls.begin(); jt != box->balls.end(); ++jt)
        {
            otherBall = *jt;
            if (otherBall->mHandled)
                continue;
            // Don't include the ball itself nor dead balls
            if(otherBall == ball || otherBall->isMoribund)
                continue;
            if(filter.EXCLUDE_CLOAKED && (otherBall->isCloaked || !otherBall->isMassive))
                continue;
            if(filter.EXCLUDE_INERT_BALLS)
            {
                if(!otherBall->isInteractive)
                    continue;
            }
            else
            {
                // other bubbles, except for global balls
                if(isMaster && ( otherBall->mNewBubble != ball->mNewBubble && !otherBall->isGlobal ))
                    continue;

                if(filter.EXCLUDE_FIXED_BALLS && !otherBall->isFree)
                    continue;

                // Don't include force fields with the same harmonics as you (unless told to do so).
                if(otherBall->mMode == DSTBALL_FIELD && !filter.EXCLUDE_HARMONICS_TEST){
                    if(ball->mHarmonic == -2) // Special masterpass value
                        continue;
                    if(otherBall->mHarmonic != -1 && otherBall->mHarmonic == ball->mHarmonic) // harmonics match
                        continue;
                    if(otherBall->mCorporationID != -1 && otherBall->mCorporationID == ball->mCorporationID) // your own corp
                        continue;
                    if(otherBall->mAllianceID != -1 && otherBall->mAllianceID == ball->mAllianceID) // your own alliance
                        continue;
                }

                if(filter.EXCLUDE_MISSILES && (otherBall->mMode == DSTBALL_MISSILE))
                    continue;
            }

            otherBall->mHandled = true;
            uni.push_back(otherBall);
        }
	
		SortedSetOfStaticCollidables::iterator sit;

		if(!filter.EXCLUDE_FIXED_BALLS)
		{
			StaticCollidable *collidable;
			for(sit = box->mStaticCollidables.begin(); sit != box->mStaticCollidables.end(); ++sit)
			{
				collidable = *sit;
				if (collidable->mHandled)
					continue;

				// other bubbles
				if(isMaster && ( collidable->mNewBubble != ball->mNewBubble ))
					continue;

				collidable->mHandled = true;
				uniStat.push_back(collidable);
			}
		}
	}
    
    for(ssize_t i = uni.size()-1; i>=0; --i)
        uni[i]->mHandled = false;

	for (ssize_t i = uniStat.size() - 1; i >= 0; --i)
		uniStat[i]->mHandled = false;
}


PyObject * Partition::GetActiveBoxes(int level)
{
    if(level >= mNumberOfLevels || level < 0)
        return 0;    // Illegal level

    unsigned numberOfBoxes = (unsigned)mLevels[level].size();
    if(numberOfBoxes==0)
        return 0; // No active box for that level

    MapOfBoxes::iterator boxIt;

    PyObject* boxes = PyList_New(numberOfBoxes);

    unsigned i = 0;
    // Cycle over all boxes at the given level
    for(boxIt = mLevels[level].begin() ; boxIt != mLevels[level].end(); ++boxIt)
    {
        Box *box = boxIt->second;
        PyObject *ret =  Py_BuildValue("fff", box->lo[0], box->lo[1], box->lo[2]);
        PyList_SET_ITEM(boxes, i, ret); // Note that reference to ret is stealed
        i++;
    }

    PyObject *ret = Py_BuildValue("fO",mLevelWidth[level],boxes);
    Py_DECREF(boxes);

    return ret;
}


BoxAllocator::BoxAllocator(): chunkSize(256)
{}

BoxAllocator::~BoxAllocator()
{
    //Release the uninitialized memory in the vector
    while(!freeBoxes.empty()) {
        operator delete(freeBoxes.back());
        freeBoxes.pop_back();
    }
}

Box * BoxAllocator::GetNewBox(int64_t i,  int64_t j,  int64_t k, long level,Partition *p)
{
    //Allocate.  Either get memory from the vector, or from operator new
    void *mem;
    if (!freeBoxes.empty()) {
        mem = freeBoxes.back();
        freeBoxes.pop_back();
    } else
        mem = operator new(sizeof(Box));

    //and now use a placement new
    return new(mem) Box(i, j, k, level, p); //placement new
}

void BoxAllocator::ReleaseBox(Box *box)
{
    //Either store the memory in the list, or delete it completely.
    if(freeBoxes.size() < chunkSize) {
        box->~Box(); //destruct box
        freeBoxes.push_back(box);
    } else
        delete box;
}
