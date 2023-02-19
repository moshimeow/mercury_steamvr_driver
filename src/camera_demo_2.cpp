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
template <class T> void SafeRelease(T** ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}
#include "capture.h"
// has to be first, or else you'll get weird errors. Windows is so strange.

#include <opencv2/opencv.hpp>

#include <fstream>
#include <string>

// #include "monado/src/xrt/include/xrt/xrt_config.h"
#include "xrt/xrt_config.h"

#include "vive/vive_config.h"

#include "tracking/t_hand_tracking.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "../monado/src/xrt/tracking/hand/mercury/hg_interface.h"

#include "os/os_time.h"
#include "steamvr_driver/oxr_sdl2_hack.h"

namespace xat = xrt::auxiliary::tracking;





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
    IMFAttributes* pAttributes = NULL;

    Clear();

    // Initialize an attribute store. We will use this to 
    // specify the enumeration parameters.

    hr = MFCreateAttributes(&pAttributes, 1);

    // Ask for source type = video capture devices
    if (SUCCEEDED(hr))
    {
        hr = pAttributes->SetGUID(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
        );
    }

    // Enumerate devices.
    if (SUCCEEDED(hr))
    {
        hr = MFEnumDeviceSources(pAttributes, &m_ppDevices, &m_cDevices);
    }

    SafeRelease(&pAttributes);

    return hr;
}


HRESULT DeviceList::GetDeviceName(UINT32 index, WCHAR** ppszName)
{
    if (index >= Count())
    {
        return E_INVALIDARG;
    }

    HRESULT hr = S_OK;

    hr = m_ppDevices[index]->GetAllocatedString(
        MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
        ppszName,
        NULL
    );

    return hr;
}


std::string read_file(std::string_view path) {
    constexpr auto read_size = std::size_t(4096);
    auto stream = std::ifstream(path.data());
    stream.exceptions(std::ios_base::badbit);
    
    std::string out = std::string();
    auto buf = std::string(read_size, '\0');
    while (stream.read(& buf[0], read_size)) {
        out.append(buf, 0, stream.gcount());
    }
    out.append(buf, 0, stream.gcount());
    return out;
}



int main() {

    int match_idx = -1;

    {
        DeviceList devlist = DeviceList();
        devlist.EnumerateDevices();

        // I've only seen Indices with these. Fingers crossed :)
        const WCHAR* match = L"eTronVideo";


        for (uint32_t i = 0; i < devlist.Count(); i++){
            WCHAR* n = NULL;
            

            devlist.GetDeviceName(i, &n);
            std::wcout << n << std::endl;


            if (wcscmp(match, n) == 0) {
                U_LOG_E("match!");
                match_idx = i;
                break;
            }

        }

        if (match_idx == -1) {
            U_LOG_E("Didn't find a camera 😭");
            return 0;
        }
    }




	std::string t20_config = read_file("C:\\dev\\mercury_steamvr_driver\\T20_config.json");

	std::cout << t20_config << std::endl;


	struct vive_config c = {};

	const char* e = t20_config.c_str();

	char* j = (char*)malloc(sizeof(char) * 500000);

	strcpy(j, e);

	vive_config_parse(&c, j, U_LOGGING_DEBUG);

	struct t_stereo_camera_calibration *calib = NULL;

	xrt_pose head_in_left; // unused

	vive_get_stereo_camera_calibration(&c, &calib, &head_in_left);

	void *sdl2_hack;

    oxr_sdl2_hack_create(&sdl2_hack);
    oxr_sdl2_hack_start(sdl2_hack, NULL, NULL);



    // zero-initialized out of paranoia
	struct t_camera_extra_info info = {};

	info.views[0].camera_orientation = CAMERA_ORIENTATION_0;
	info.views[1].camera_orientation = CAMERA_ORIENTATION_0;

	info.views[0].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;
	info.views[1].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;


	//!@todo This changes by like 50ish pixels from device to device. For now, the solution is simple: just
	//! make the circle a bit bigger than we'd like.
	// Maybe later we can do vignette calibration? Write a tiny optimizer that tries to fit Index's
	// gradient? Unsure.
	info.views[0].boundary.circle.normalized_center.x = 0.5f;
	info.views[0].boundary.circle.normalized_center.y = 0.5f;

	info.views[1].boundary.circle.normalized_center.x = 0.5f;
	info.views[1].boundary.circle.normalized_center.y = 0.5f;

	info.views[0].boundary.circle.normalized_radius = 0.55;
	info.views[1].boundary.circle.normalized_radius = 0.55;

struct t_hand_tracking_sync * sync =
	t_hand_tracking_sync_mercury_create(calib, info, "C:\\dev\\mercury_steamvr_driver\\hand-tracking-models\\");

    xrt_frame_context blah = {};

    t_hand_tracking_async *async = t_hand_tracking_async_default_create(&blah, sync);


    // So we probably want to steal OpenCV's cap_msmf.cpp at some point for all this
    #if 0
    cv::VideoCapture cap(match_idx);
    #else
    
    cv::VideoCapture cap;
    cap.open(0, cv::CAP_DSHOW);

    #endif

    // https://stackoverflow.com/questions/22393875/how-to-use-cv-cap-prop-fourcc
    #if 0
    double fourcc_d = cap.get(cv::CAP_PROP_FOURCC);
   int ex = static_cast<int>(fourcc_d);

    // Transform from int to char via Bitwise operators
    char EXT[] = {(char)(ex & 0XFF),(char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24),0};
    U_LOG_E("fourcc is %f %d %s", fourcc_d, ex, EXT);
#elif 0
    double f = cap.get(cv::CAP_PROP_FOURCC);
    char* fourcc = (char*) (&f); // reinterpret_cast
    U_LOG_E("%d %d %d %d", fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
    U_LOG_E("fourcc is %f %s", f, fourcc);

#endif


#if 1
    cap.set(cv::CAP_PROP_MODE, cv::VideoWriter::fourcc('Y','U','Y','V'));
#endif
    //

    while (true) {
        cv::Mat mat_ = {};
        cv::Mat mat = {};

        // Often returns [ WARN:0@11.016] global C:\dev\vcpkg\buildtrees\opencv4\src\4.6.0-9a95a1b699.clean\modules\videoio\src\cap_msmf.cpp (1752) CvCapture_MSMF::grabFrame videoio(MSMF): can't grab frame. Error: -2147483638 
        // Grrrr.
        bool success = cap.read(mat_);

        if (!success) {
            U_LOG_E("Failed!");
            break;
        }


        //!@todo
        uint64_t time = os_monotonic_get_ns();
        cv::cvtColor(mat_, mat, cv::COLOR_BGR2GRAY);

        
		cv::Mat mats_grayscale[2];

		// xat::FrameMat fms[2]; // = {};
		xrt_frame *frames[2] = {NULL, NULL};

		mats_grayscale[0] = mat(cv::Rect(0,0,960,960));
		mats_grayscale[1] = mat(cv::Rect(960, 0, 960, 960));

		for (int i = 0; i < 2; i++) {
			xat::FrameMat::Params params;
			params.stereo_format = XRT_STEREO_FORMAT_NONE;
			params.timestamp_ns = time;
			xat::FrameMat::wrapL8(mats_grayscale[i], &frames[i], params);

		}

        xrt_sink_push_frame(async->sinks.left, frames[0]);
        xrt_sink_push_frame(async->sinks.right, frames[1]);


        

        cv::imshow("h", mat);
        #if 0
        if (cv::waitKey(1) == 'q') {
            U_LOG_E("Bye!");
            break;
        }
        #endif
        

        U_LOG_E("meow");

    }

    cap.release();



	
	// while (true) {
    //     U_LOG_E("%zu", os_monotonic_get_ns());
    // }

    return 0;
}

