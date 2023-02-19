Tried:

* Theodore Watson's videoInput. Used DirectShow. Worked but autoconverted to RGB and did not provide timestamps. https://github.com/ofTheo/videoInput/tree/master/videoInputSrcAndDemos/libs/videoInput

* ESCAPI. https://github.com/jarikomppa/escapi Much better, uses MSMF, but autoconverts to RGBA (??) and no timestamps

* Realized that OpenCV does provide accurate system time timestamps through MSMF+cap.get(cv2.CAP_PROP_POS_MSEC). cool! I'm too lazy to get it to not autoconvert to RGB, but that's fine, the extra copies are nbd.

* Okay so mmf wasn't working for unclear reasons. We're just going to keep fixing things with directshow and have sad wrong timestamps.