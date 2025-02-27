#include <gtest/gtest.h>
#include "gtest-memleak.h"

#include <cstdio>

GTEST_API_ int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);

#if defined(_MSC_VER)
    testing::UnitTest::GetInstance()->listeners().Append(new testing::MSVCMemoryLeakListener());
#endif // _MSC_VER

    return RUN_ALL_TESTS();
}
