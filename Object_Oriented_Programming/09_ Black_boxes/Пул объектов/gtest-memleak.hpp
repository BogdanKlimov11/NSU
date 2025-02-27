#pragma once

#include <gtest/gtest.h>

#if defined(_MSC_VER)

#if defined(_DEBUG)
#include <crtdbg.h>
#endif // _DEBUG

namespace testing {

class MSVCMemoryLeakListener : public ::testing::EmptyTestEventListener {
#if defined(_DEBUG)
    class CRTReportRedirector {
    private:
        static int __cdecl crtReportHook(int reportType, char* szMsg, int* retVal) {
            if (retVal) 
                *retVal = 0;

            std::cerr << szMsg;

            if (retVal) 
                *retVal = 0;

            return (reportType == _CRT_ASSERT);
        }

    public:
        CRTReportRedirector() { InstallHook(); }
        ~CRTReportRedirector() { UninstallHook(); }

        static void InstallHook() { _CrtSetReportHook2(_CRT_RPTHOOK_INSTALL, &crtReportHook); }
        static void UninstallHook() { _CrtSetReportHook2(_CRT_RPTHOOK_REMOVE, &crtReportHook); }
    };

public:
    void OnTestStart(const TestInfo&) override { _CrtMemCheckpoint(&m_memState); }

    void OnTestEnd(const TestInfo& test_info) override {
        if (!test_info.result()->Passed())
            return;

        _CrtMemState stateNow, stateDiff;
        _CrtMemCheckpoint(&stateNow);

        if (!_CrtMemDifference(&stateDiff, &m_memState, &stateNow))
            return;

        {
            CRTReportRedirector _;
            _CrtMemDumpAllObjectsSince(&m_memState);
            _CrtMemDumpStatistics(&stateDiff);
        }

        FAIL() << "Memory leak of " << stateDiff.lSizes[1] << " byte(s) detected.";
    }

private:
    _CrtMemState m_memState;
#endif // _DEBUG
};

} // namespace testing

#endif // _MSC_VER
