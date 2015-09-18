# gesture_pkg

This repository is a combination of hand gesture detection code and the corresponding launch files for the detection codes. This repository as a simple gesture detection code uses the webcam or the kinect camera to detect the gesture made by the user and performs some basic operations accordingly. The user has to perform a specific gesture. The webcam or kinect camera captures this gestures as  frames and displays it. Further, the code extracts the ROI(region of interest or otherwise called background subtraction), finds the the contours, then finds the convexity defects after drawing the convexity hull and then uses the number of defects to perform some basic gestures like counting number of fingers and hand waving. 

The code is not much difficult to understand and add any other features but needs hard work-through and mastering on OpenCV(Open Source Computer Vision Library), which was developed by Intel as a library of programming functions mainly aimed at real-time computer vision. For more information [click here](http://opencv.org/)  


