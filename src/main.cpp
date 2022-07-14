// Copyright 2022, Moses Turner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The thing is here!
 * @author Moses Turner <mosesturner@collabora.com>
 * @ingroup tracking
 */


#include <opencv2/opencv.hpp>
#include <stdio.h>

int main() {
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

