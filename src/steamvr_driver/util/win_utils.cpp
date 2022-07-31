#include "win_utils.h"

#include "driver_log.h"

#include <Windows.h>

EXTERN_C IMAGE_DOS_HEADER __ImageBase;

std::string GetLastErrorAsString() {
    const DWORD errorMessageId = ::GetLastError();
    if (errorMessageId == 0) return std::string();

    LPSTR messageBuffer = nullptr;
    const size_t size = FormatMessageA(
            FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
            nullptr,
            errorMessageId,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            reinterpret_cast<LPSTR>(&messageBuffer),
            0,
            nullptr);

    std::string message(messageBuffer, size);

    LocalFree(messageBuffer);

    return message;
}

bool CreateProcessWaitTerminate(const std::string &executable_path, const std::string &executable_name,
                                const std::string &arguments, int timeout) {
    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof si);
    si.cb = sizeof si;
    ZeroMemory(&pi, sizeof pi);

    std::string cl_input = executable_name + " " + arguments;

    char *carguments = new char[cl_input.length() + 1];
    strcpy(carguments, cl_input.c_str());

    if (!CreateProcess((executable_path + "\\" + executable_name).c_str(),
                       carguments,
                       nullptr,
                       nullptr,
                       FALSE,
                       0,
                       nullptr,
                       executable_path.c_str(),
                       &si,
                       &pi)) {
        delete[] carguments;
        DriverLog("CreateProcess failed. Error: %s", GetLastErrorAsString().c_str());
        return false;
    }

    //fail if we had anything but the process exiting by itself
    if (WaitForSingleObject(pi.hProcess, timeout) != WAIT_OBJECT_0) {
        delete[] carguments;
        DriverLog("WaitForSingleObject failed. Error: %s", GetLastErrorAsString().c_str());
        return false;
    }
    
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    delete[] carguments;

    return true;
}

bool GetDLLPath(std::string &out_string) {
    HMODULE hm = nullptr;
    if (GetModuleHandleExA(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(&__ImageBase), &hm) == 0) {
        DriverLog("GetModuleHandle failed, error: %s", GetLastErrorAsString().c_str());
        return false;
    }

    char path[1024];
    if (GetModuleFileNameA(hm, path, sizeof path) == 0) {
        DriverLog("GetModuleFileName failed, error: %s", GetLastErrorAsString().c_str());
        return false;
    }

    out_string = path;

    return true;
}