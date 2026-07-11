// Copyright © 2026 CCP ehf.

#include "StdAfx.h"

TEST(PartitionKeyHash, SeparatesCoordinatesThatPreviouslyCollided)
{
    const Key first(1, 0, 0, 16);
    const Key second(0, 2, 0, 16);

    // The previous linear hash produced 24 for both keys:
    // (1 << 3) + 16 == (2 << 2) + 16.
    EXPECT_NE(KeyHash{}(first), KeyHash{}(second));
}

TEST(PartitionKeyHash, RetainsEveryDistinctBoxKey)
{
    MapOfBoxes boxes;
    constexpr int gridSize = 16;

    for (int64_t ix = 0; ix < gridSize; ++ix)
    {
        for (int64_t iy = 0; iy < gridSize; ++iy)
        {
            for (int64_t iz = 0; iz < gridSize; ++iz)
            {
                boxes.emplace(Key(ix, iy, iz, gridSize), nullptr);
            }
        }
    }

    EXPECT_EQ(boxes.size(), static_cast<size_t>(gridSize * gridSize * gridSize));

    for (int64_t ix = 0; ix < gridSize; ++ix)
    {
        for (int64_t iy = 0; iy < gridSize; ++iy)
        {
            for (int64_t iz = 0; iz < gridSize; ++iz)
            {
                const auto found = boxes.find(Key(ix, iy, iz, gridSize));
                ASSERT_NE(found, boxes.end());
            }
        }
    }
}
