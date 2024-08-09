#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>


/**
 * draw detected markers from video feed from default laptop camera
 */
void detectMarker_Video(){
    cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), cv::aruco::DetectorParameters());

    cv::VideoCapture inputVideo;
    inputVideo.open(1); // open default camera


    while (inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);


        //detect fiducials
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        detector.detectMarkers(image, corners, ids, rejected);


        //draw detected markers
        image.copyTo(imageCopy);
        if (!ids.empty())
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

        imshow("out", imageCopy);

        if (char key = (char)cv::waitKey(10); key == 27) break;
    }
}
