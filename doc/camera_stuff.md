Tried:

* Theodore Watson's videoInput. Used DirectShow. Worked but autoconverted to RGB and did not provide timestamps. https://github.com/ofTheo/videoInput/tree/master/videoInputSrcAndDemos/libs/videoInput

* ESCAPI. https://github.com/jarikomppa/escapi Much better, uses MSMF, but autoconverts to RGBA (??) and no timestamps

* Realized that OpenCV does provide accurate system time timestamps through MSMF+cap.get(cv2.CAP_PROP_POS_MSEC). cool! I'm too lazy to get it to not autoconvert to RGB, but that's fine, the extra copies are nbd.

* Okay so mmf wasn't working for unclear reasons. We're just going to keep fixing things with directshow and have sad wrong timestamps.

* With escapi_test.cpp, I saw similar behaviour with MSMF not working. This still doesn't prove anything IMO

* Do I know what OBS is using? I'm pretty sure it's DirectShow considering references to "Configure Crossbar" in the UI but I'm very unsure. OBS's github has references to both DirectShow and MSMF.
    * https://github.com/obsproject/obs-studio/search?q=MFGetAttributeSize returns nothing
    * Neither does any DShow code keywords I could think of, hmmmmm
    * Oh hey nice https://github.com/obsproject/libdshowcapture
    * Definitely leaning towards "yea it uses dshow"




https://learn.microsoft.com/en-us/windows/win32/directshow/time-and-clocks-in-directshow



https://learn.microsoft.com/en-us/windows/win32/directshow/time-and-clocks-in-directshow


strmif.h IMediaSample GetMediaTime
    no work


https://github.com/opencv/opencv/issues/17687#issuecomment-800227819



I finally seem to have a good solution getting timestamped video frames on Windows, and the delay between capture and arrival is about 17ms
(which is intriguing because that's just about the index's frametime)
moshi — Today at 7:19 AM
[hmm I'm seeing it jump around, lowest i've seen is 4ms highest i've seen is like 30]
it doesn't change that quickly so i'm not worried, and tbh i've never looked at this on Linux