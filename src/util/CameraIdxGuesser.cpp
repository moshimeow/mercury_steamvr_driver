// Copyright 2022, Collabora, Inc.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The thing is here!
 * @author Moses Turner <moses@collabora.com>
 */

// This block...
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>

#include <mfobjects.h>
#include <Dbt.h>
#pragma comment(lib, "mf")
#pragma comment(lib, "mfplat")
template <class T>
void SafeRelease(T **ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}
#include "CameraIdxGuesser_win_capture.h"
// has to be first, or else you'll get weird errors. Windows is so strange.


#include <comdef.h>


#include <fstream>
#include <string>

#include "os/os_time.h"
// #include "driver_log.h"
#include "u_subprocess_logging.h"

#define DriverLog U_SP_LOG_E

void DeviceList::Clear()
{
    for (UINT32 i = 0; i < m_cDevices; i++)
    {
        SafeRelease(&m_ppDevices[i]);
    }
    CoTaskMemFree(m_ppDevices);
    m_ppDevices = NULL;

    m_cDevices = 0;
}

HRESULT DeviceList::EnumerateDevices()
{
    HRESULT hr = S_OK;
    IMFAttributes *pAttributes = NULL;

    Clear();

    // Initialize an attribute store. We will use this to
    // specify the enumeration parameters.

    hr = MFCreateAttributes(&pAttributes, 1);

    // Ask for source type = video capture devices
    if (SUCCEEDED(hr))
    {
        hr = pAttributes->SetGUID(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    }

    // Enumerate devices.
    if (SUCCEEDED(hr))
    {
        hr = MFEnumDeviceSources(pAttributes, &m_ppDevices, &m_cDevices);
    }

    SafeRelease(&pAttributes);

    return hr;
}

HRESULT DeviceList::GetDeviceName(UINT32 index, WCHAR **ppszName)
{
    if (index >= Count())
    {
        return E_INVALIDARG;
    }

    HRESULT hr = S_OK;

    hr = m_ppDevices[index]->GetAllocatedString(
        MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
        ppszName,
        NULL);

    return hr;
}



int GetIndexIndex() 
{
    DriverLog("Getting Index Index!");

        int match_idx = -1;

    {
        DeviceList devlist = DeviceList();
        devlist.EnumerateDevices();

        // I've only seen Indices with these. Fingers crossed :)
        const WCHAR *match = L"eTronVideo";

        for (uint32_t i = 0; i < devlist.Count(); i++)
        {
            WCHAR *name = NULL;

            devlist.GetDeviceName(i, &name);

            _bstr_t b(name);
            const char* c = b;

            DriverLog("Found camera: %s\n", c);

            if (wcscmp(match, name) == 0)
            {
                DriverLog("Was a match!");
                match_idx = i;
                break;
            }
        }

        if (match_idx == -1)
        {
            DriverLog("Didn't find a camera 😭");
            return match_idx;
        }
    }


    return match_idx;
}