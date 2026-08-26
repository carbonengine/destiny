// Copyright © 2014 CCP ehf.

#ifndef DST_PARTITION_H
#define DST_PARTITION_H

#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <iterator>

#include "SortedSets.h"

struct Vector3d;

/*------------------------------------------------------------------------
   A Box is the building elment of a space Partition. Basically all
   of space is partitioned into a hierarchical collection of boxes.
   Each box maintains a list of Balls that intersect it.

   The partition keeps only tracks of hierarchies in which there are some
   balls, so its mostly empty space, as it should be.

   The partitition is built in the following manner:

   Space is covered by boxes and sub-boxes contained in them. The subdivision
   of space has a lower bound, which defines the maximum number of boxes
   in each space dimension for that level. We define this level with the
   2 exponent of the number of boxes and call 'mFineLevel'. The number of boxes
   in each dimension af that level is thus given by 2^mFineLevel. The set of
   boxes are centered divided equally around zero.

   On top of this fine grid, we define a number of coarser grids, that wholly
   contain the grid underneath. The subdivision exponent is defined by the
   variable 'mGridBase'. The number of division between level i and i+1
------------------------------------------------------------------------*/
class Box;
class Ball;
class StaticCollidable;
class Key;

typedef int64_t ID;
typedef std::unordered_map<ID, Ball *> DictOfBalls;
typedef std::vector<Ball *> VectorOfBalls;
typedef std::vector<StaticCollidable *> VectorOfStaticCollidables;
typedef std::pair<ID, Ball*> DictEntry;
typedef std::vector<Box *> VectorOfBoxes;
typedef std::back_insert_iterator< VectorOfBoxes > BoxVectorInsertor;

class Partition;
class BoxAllocator
{
public:
    BoxAllocator();

    ~BoxAllocator();

    Box *GetNewBox(int64_t i,  int64_t j,  int64_t k, long level, Partition *p);

    void ReleaseBox(Box *box);

private:
    unsigned chunkSize;
    std::vector<void *> freeBoxes;
};

class Key
{
public:
    Key():ix(0),iy(0),iz(0),n(0) {};
    Key(int64_t i,int64_t j,int64_t k,int64_t N):ix(i),iy(j),iz(k),n(N) { Clamp(); };
    bool operator == ( const Key& k) const  { return ix==k.ix && iy==k.iy && iz==k.iz && n==k.n;};
    //less than operator
    bool operator < (const Key& k) const {
        if (ix < k.ix) return true;
        if (ix > k.ix) return false;
        if (iy < k.iy) return true;
        if (iy > k.iy) return false;
        if (iz < k.iz) return true;
        if (iz > k.iz) return false;
        return n<k.n;
    }
    int64_t ix;
    int64_t iy;
    int64_t iz;
    int64_t n;

    void Clamp()
    {
        if(ix<0)
            ix = 0;
        else if(ix>=n)
            ix = n-1;
        if(iy<0)
            iy = 0;
        else if(iy>=n)
            iy = n-1;
        if(iz<0)
            iz = 0;
        else if(iz>=n)
            iz = n-1;
    }
};

struct KeyHash
{
    size_t operator()(const Key& key) const noexcept
    {
        size_t seed = 0;
        const auto combine = [&seed](int64_t value)
        {
            const size_t valueHash = std::hash<int64_t>{}(value);
            seed ^= valueHash + static_cast<size_t>(0x9e3779b9U) + (seed << 6) + (seed >> 2);
        };

        combine(key.ix);
        combine(key.iy);
        combine(key.iz);
        combine(key.n);
        return seed;
    }
};

typedef std::unordered_map<Key, Box*, KeyHash> MapOfBoxes;


class Partition
{
    friend Box;
public:
    Partition();

    // GetBoxKey outputs the key (ix, iy, iz) for a box at the given level which contains pos.
    void GetBoxKey(const Vector3d& pos, long level, int64_t& ix, int64_t& iy, int64_t& iz);

    //
    // GetBox(i,j,k,l) returns a box for the given grid coordinate at the given level
    // creating one if it didn't exist
    //
    Box* GetBox(int64_t ,  int64_t , int64_t ,  long level);
    //
    // CheckBox returns a box for the given grid coordinate at the given level
    // if and only if it already exists
    //
    Box *CheckBox(int64_t ix, int64_t iy, int64_t iz,  long level);
    //
    // GetBox(r,p) returns a box for the given 3D position and the level corresponding to radius
    // creating one if it didn't exist
    //
    Box* GetBox(double radius, const Vector3d& pos, bool create=true);

    //
    // GetExistingTopBox(p) returns the topmost box for the given 3D position
    // if and only if it already exists
    //
    Box* GetExistingTopBox(const Vector3d& pos);

    //
    // GetOffsetBox() and CheckOffsetBox() are similar to the the GetBox() and CheckBox() functions above
    // except the indices are interpreted as offsets from the box provided.
    //
    Box* GetOffsetBox(Box *box,int64_t i,int64_t j,int64_t k);
    Box* CheckOffsetBox(Box *box,int64_t i,int64_t j,int64_t k);

    // Returns the center of the box corresponding to given coordinate for given level    
    void GetBoxCenter(int level,  Vector3d& pos);

    // returns the total number of boxes in the partition
    int GetSize();

    // Given a box, returns the topmost box of the partition
    Box* GetTopmost(Box *box);

    // Clears the traversal flag for all topmost boxes
    void ClearTraversalFlag();

    // Debug function that logs the current content of all partitions
    void DumpPartition();

    // Returns the coordinates of all active boxes of a given level as Python list
    PyObject *GetActiveBoxes(int level);
    
    // Gets nearby balls that are pertinent to proximity sensors
    void GetProximityCandidates(Ball* ball, VectorOfBalls& uni, VectorOfStaticCollidables& uniStat, bool isMaster);
    
    // Gets nearby balls that are pertinent to collision detection
    void GetCollisionCandidates(Ball* ball, VectorOfBalls& uni, VectorOfStaticCollidables& uniStat, bool isMaster);

    // vector containing partitions for different levels. Zero is the coarsest level
    std::vector<MapOfBoxes> mLevels;
    std::vector<double> mLevelWidth; // size of partition boxes for each level
    std::vector<int64_t> mLevelGrid; // number of grid partitions for each level

    int mNumberOfLevels;

    struct NearbyCriteria {
        NearbyCriteria(int casenum=0){ 
            memset(this, 0, sizeof(*this));
            switch(casenum) // initialize with a known 'static' filter case
            {
            case 1: EXCLUDE_MISSILES = EXCLUDE_CLOAKED = 1; break;
            case 3: EXCLUDE_CLOAKED = 1; break;
            }
        };
        unsigned int EXCLUDE_FIXED_BALLS      : 1;
        unsigned int EXCLUDE_INERT_BALLS      : 1;
        unsigned int EXCLUDE_MISSILES         : 1;
        unsigned int EXCLUDE_CLOAKED          : 1;
        unsigned int EXCLUDE_HARMONICS_TEST   : 1;
    };
	// Get collidables that are 'close', according to the criteria specified in 'filter'
	void GetNearbyBalls(Ball* ball, VectorOfBalls& uni, VectorOfStaticCollidables& uniStat, bool isMaster, NearbyCriteria filter);

	private:

	void ExplodeChildren(SortedSetOfBoxes &children, BoxVectorInsertor &ins);

    double mGridUnit;
    BoxAllocator mBoxAllocator;
};

#endif
