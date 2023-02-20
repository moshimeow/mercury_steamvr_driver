// Copyright 2022, Collabora, Inc.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The thing is here!
 * @author Moses Turner <moses@collabora.com>
 */

// This block...
#include "escapi.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>


int main() {
    std::cout << "meow" << std::endl;
    int devices = setupESCAPI();
    std::cout << "meow" << std::endl;

    if (devices == 0)
	{
		printf("ESCAPI initialization failure or no devices found.\n");
		return 0;
	}

  /* Set up capture parameters.
   * ESCAPI will scale the data received from the camera 
   * (with point sampling) to whatever values you want. 
   * Typically the native resolution is 320*240.
   */

	struct SimpleCapParams capture;
	capture.mWidth = 24;
	capture.mHeight = 18;
	capture.mTargetBuf = new int[24 * 18];
	
	/* Initialize capture - only one capture may be active per device,
	 * but several devices may be captured at the same time. 
	 *
	 * 0 is the first device.
	 */
	
	if (initCapture(1, &capture) == 0)
	{
		printf("Capture failed - device may already be in use.\n");
		return 0;
	}

    int i,j;

    
	/* Go through 10 capture loops so that the camera has
	 * had time to adjust to the lighting conditions and
	 * should give us a sane image..	 
	 */
	for (int i = 0; i < 10; i++)
	{
		/* request a capture */			
        printf("doing_cap");
		doCapture(0);
        printf("done_cap");
		
		while (isCaptureDone(0) == 0)
		{
			/* Wait until capture is done.
			 * Warning: if capture init failed, or if the capture
			 * simply fails (i.e, user unplugs the web camera), this
			 * will be an infinite loop.
			 */		   
            printf("infininte-loop");

		}
	}

    	char light[] = " .,-o+O0@";
	for (i = 0; i < 18; i++)
	{
		for (j = 0; j < 24; j++)
		{
			printf("%c", light[(capture.mTargetBuf[i*24+j] >> 13) & 7]);
		}
		printf("\n");
	}

	deinitCapture(0);	
    return 0;
}

