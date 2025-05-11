#define _CRTDBG_MAP_ALLOC

#include <vector>
#include <assert.h>
#include <crtdbg.h>

#include "shared_ptr.hpp"

#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW

struct TstStruct {
    explicit TstStruct(int value) : m_value(value) { ++instances; }
    ~TstStruct() { --instances; }
    int m_value;
    static int instances;
};

int TstStruct::instances = 0;

class ArrayAssertHelper {
public:
    ArrayAssertHelper() { ++instances; }
    ~ArrayAssertHelper() { --instances; }
    static int instances;
};

int ArrayAssertHelper::instances = 0;

int main() {
    {
        SharedPTR<TstStruct> testPtr(new TstStruct(10));
        {
            SharedPTR<TstStruct> anotherPtr(testPtr);
        }
    }
    assert(TstStruct::instances == 0);

    {
        SharedPTR<TstStruct> testPtr(new TstStruct(10));
        SharedPTR<TstStruct> anotherPtr(new TstStruct(11));
        testPtr.swap(anotherPtr);
    }
    assert(TstStruct::instances == 0);

    {
        SharedPTR<TstStruct> testPtr;
    }
    assert(TstStruct::instances == 0);

    {
        SharedPTR<ArrayAssertHelper[]> testPtr(new ArrayAssertHelper[10]);
    }
    assert(ArrayAssertHelper::instances == 0);

    {
        SharedPTR<TstStruct> testPtr(new TstStruct(10));
        testPtr.release();
    }
    assert(TstStruct::instances == 0);

    {
        SharedPTR<TstStruct> testPtr;
        testPtr.reset(new TstStruct(10));
        testPtr.reset();
    }
    assert(TstStruct::instances == 0);

    {
        SharedPTR<TstStruct> testPtr(new TstStruct(10));
        testPtr = testPtr;
    }
    assert(TstStruct::instances == 0);

    {
        auto ptr = new TstStruct(10);
        SharedPTR<TstStruct> testPtr(ptr);
        testPtr = ptr;
    }
    assert(TstStruct::instances == 0);

    {
        SharedPTR<TstStruct> testPtr(new TstStruct(10));
        testPtr = std::move(testPtr);
    }
    assert(TstStruct::instances == 0);

    _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDOUT);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDOUT);
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDOUT);

    _CrtDumpMemoryLeaks();
    return 0;
}
