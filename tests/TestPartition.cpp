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
