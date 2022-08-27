

This is hilariously in-development. Check back later!



flow:

One `MercuryHandDevice` per hand.

??? creation stuff
DeviceProvider::Init:
    uses Get/CreateHMDConfig which uses lighthouse_console.exe and saves it to ...some internal path, who cares, then loads it using std::ifstream. Very good.
     creates: DeviceProvider::HandTrackingThread using std::thread
DeviceProvider::HandTrackingThread



without pretending to be index controllers:
the hand's forward axis seems to be my wrist's +Z, so totally backwards. the thumb seems to be closer, when my back of my hand is facing me/the camera, and the palms are both facing out. 