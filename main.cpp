#include <iostream>
// #include "11.openCV_rerun_poseImage.cpp"
// #include "8.oakDCapture.cpp"
// #include "5.depthAITutorial_GetIntrinsics.cpp"
// #include "12.poseImage_forEVK.cpp"
// #include "12.gtsamTests.cpp"

#include "13.OAKD_save_vid.cpp"
// #include "5.depthAITutorial_GetIntrinsics.cpp"
// #include "14.daiOAKDRGBCameraControl.cpp"

// #include "11.openCV_rerun_poseImage.cpp"

// #include "3.openCVTutorials_DetectArucoVideo.cpp"
#include "4.depthAITutorial_DispLeftAndRight.cpp"
// #include "5.depthAITutorial_GetIntrinsics.cpp"

int main() {
    // logImagePoseRerun();
   //captureVid(720, "DICT_APRILTAG_36h11", "vid_APRILTAG_36h11_720p");
   // captureVid(720, "DICT_APRILTAG_36h11", "roomMap2");
    // captureVid(480, "DICT_APRILTAG_36h11", "vid_APRILTAG_36h11_480p_hello");
    // printCameraIntrinsics();

    display_vid_sample();
    // printCameraIntrinsics();
  // detectMarker_Video();


    std::cout << "Hello World";
    return 0;
}

/**
 * NOTE: just noting somewhere that in order to get the rerun viewer to show using C++ in CLion
 * I edited the PATH var in the CLion run configurations option.
 *
* Intrinsics from defaultIntrinsics function:
[[3075.795166, 0.000000, 1927.994995]
[0.000000, 3075.795166, 1078.134277]
[0.000000, 0.000000, 1.000000]]

Width: 3840
Height: 2160
[2.088857, -82.303825, -0.000713, 0.002002, 315.661438, 1.858882, -80.083954, 308.980713, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]

 */

