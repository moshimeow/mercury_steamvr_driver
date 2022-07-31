// #include "stdafx.h"

#include "steamvr_thingy.hpp"
#include <string>
#include <fstream>
#include <windows.h>

char g_modulePath[2048U];

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID /* lpReserved */)
{
    switch(ul_reason_for_call)
    {
        case DLL_PROCESS_ATTACH:
            GetModuleFileNameA(hModule, g_modulePath, 2048U);
            break;
        case DLL_THREAD_ATTACH: case DLL_THREAD_DETACH: case DLL_PROCESS_DETACH:
            break;
    }
    return TRUE;
}

// CServerDriver g_serverDriver;

extern "C" __declspec(dllexport) void* HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
    void *l_result = nullptr;

    // Note - I think 

    // if(!strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName)) l_result = dynamic_cast<vr::IServerTrackedDeviceProvider*>(&g_serverDriver);
    // else
    // {
    //     if(pReturnCode) *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    // }
    std::ofstream of("C:\\dev\\hello.txt", std::ofstream::out);
    

    of << "HI" << std::endl;
    of.flush();
    of.close();
    printf("%s\n", NULL);
    abort();
    *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    return l_result;
}
