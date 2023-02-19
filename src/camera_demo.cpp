// Copyright 2022, Collabora, Inc.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The thing is here!
 * @author Moses Turner <moses@collabora.com>
 */


#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "videoInput.h"

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

	videoInput in;

	in.setVerbose(true);
	int num_devices = in.listDevices();
	std::cout << num_devices << std::endl;

	std::vector<std::string> devs = in.getDeviceList();

	int wanted_idx = 0;
	bool is_camera = false;
	for (int i = 0; i < devs.size(); i++) {
		std::cout << devs[i] << std::endl;
		is_camera = devs[i] == "eTronVideo";
		std::cout << is_camera << std::endl;
		if (is_camera) {
			wanted_idx = i;
			is_camera = true;
			break;
		}
	}
	if (!is_camera) {
		std::cout << "Didn't find camera!" << std::endl;
		abort();
	}

	in.setUseCallback(true);
	in.setIdealFramerate(wanted_idx, 54);
	in.setupDevice(wanted_idx, 1920, 960);

	

	//in.setFormat(wanted_idx, 6);
	//in.setFormat(wanted_idx, 4);

	in.setRequestedMediaSubType(6); // YUYV
	

	int width = in.getWidth(wanted_idx);
	int height = in.getHeight(wanted_idx);
	int size = in.getSize(wanted_idx);

	std::cout << width << " " << height << " " << size << std::endl;

	unsigned char* buf = new unsigned char[size];

	while (true) {
		in.getPixels(wanted_idx, buf, false, true);
		//cv::Mat(full_size, CV_8UC3, debug_frame->data, debug_frame->stride);

		uint64_t time = os_monotonic_get_ns();



		cv::Mat eh = cv::Mat(cv::Size(1920, 960), CV_8UC3, buf, 1920*3);

		std::cout << "lol\n";

		cv::Mat mats_rgb[2];
		cv::Mat mats_grayscale[2];

		// xat::FrameMat fms[2]; // = {};
		xrt_frame *frames[2] = {NULL, NULL};

		mats_rgb[0] = eh(cv::Rect(0,0,960,960));
		mats_rgb[1] = eh(cv::Rect(960, 0, 960, 960));

		for (int i = 0; i < 2; i++) {
			cv::cvtColor(mats_rgb[i], mats_grayscale[i], cv::COLOR_BGR2GRAY);
			xat::FrameMat::Params params;
			params.stereo_format = XRT_STEREO_FORMAT_NONE;
			params.timestamp_ns = time;
			xat::FrameMat::wrapL8(mats_grayscale[i], &frames[i], params);

		}
		U_LOG_E("about to push");
		xrt_sink_push_frame(async->sinks.left, frames[0]);
        xrt_sink_push_frame(async->sinks.right, frames[1]);
		U_LOG_E("done pushing");
// 		struct xrt_hand_joint_set hands[2];
// 		uint64_t out_timestamp;
// t_ht_sync_process(sync, frames[0], frames[1], &hands[0], &hands[1], &out_timestamp);





	for (int i = 0; i < 2; i ++) {
		// Don't need 'em
		xrt_frame_reference(&frames[i], NULL);
	}

		// 		cv::imshow("0", mats_grayscale[0]);
		// 		cv::imshow("l", mats_grayscale[1]);
		// cv::waitKey(1);
	}

	return 0;

}

