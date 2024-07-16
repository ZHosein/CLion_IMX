#include <iostream>
#include <opencv2/objdetect/aruco_detector.hpp>


/**
 * draw detected markers from video feed from default laptop camera
 */
void detectMarker_Video(){
    cv::aruco::ArucoDetector detector(aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), cv::aruco::DetectorParameters());

    cv::VideoCapture inputVideo;
    inputVideo.open(0); // open default camera


    while (inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);


        //detect fiducials
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        detector.detectMarkers(image, corners, ids, rejected);


        //draw detected markers
        image.copyTo(imageCopy);
        if (!ids.empty())
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

        imshow("out", imageCopy);

        if (char key = (char)waitKey(10); key == 27) break;
    }
}
