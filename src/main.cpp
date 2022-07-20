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

int main() {

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
		abort();
	}

	in.setUseCallback(false);
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



		cv::Mat eh = cv::Mat(cv::Size(1920, 960), CV_8UC3, buf, 1920*3);
		cv::imshow("h", eh);
		cv::waitKey(1);
		std::cout << "lol\n";
	}

	return 0;



	cv::VideoCapture bleh;
	bleh.open(0);

	while (true) {
		cv::Mat img;
		bleh.read(img);

		cv::imshow("h", img);
		cv::waitKey(1);
	}
	return 0;
}

