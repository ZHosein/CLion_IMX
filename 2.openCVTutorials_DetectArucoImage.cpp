#include <iostream>
#include "opencv2/objdetect/aruco_detector.hpp"

void detectMarkersInImage() {
    String inFileExt = ".jpg";
    // String filename = "singlemarkersoriginal.jpg";
    // String filename = "imageArucoDICT_6x6_250.jpg";
    // int dictionary = cv::aruco::DICT_6X6_250;
    // String inputFilename = "arucoBoardDICT_4x4";
    // int dictionaryToUse = cv::aruco::DICT_4X4_50;
    // String inputFilename = "AprilTag16h5";
    // int dictionaryToUse = cv::aruco::DICT_APRILTAG_16h5;

    String inputFilename = "AprilTag36h11";
    int dictionaryToUse = cv::aruco::DICT_APRILTAG_36h11;

    String inputImagePath = format("%s/media/", SOURCE_DIR) + inputFilename + inFileExt;
    Mat image = imread(inputImagePath, IMREAD_COLOR);

    std::vector<int> markerIDs;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    // params to refine how the detection is done; going with defaults for now
    cv::aruco::DetectorParameters detectorParameters = cv::aruco::DetectorParameters();

    // dic of tags to detect
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictionaryToUse);

    // create a Detector obj to be used for detecing the markers of the dict in the image
    cv::aruco::ArucoDetector detector(dictionary, detectorParameters);
    detector.detectMarkers(image, markerCorners, markerIDs, rejectedCandidates);

    /*for (auto id : markerIDs) std::cout << id << ", ";
    std::cout << std::endl;
    for (const auto& crner : markerCorners) std::cout << crner << std::endl;*/

    //draw the ouput on image
    Mat detectedOutputImage = image.clone();
    Mat rejectedOutImage = image.clone();
    cv::aruco::drawDetectedMarkers(detectedOutputImage, markerCorners, markerIDs);
    cv::aruco::drawDetectedMarkers(rejectedOutImage, rejectedCandidates);

    imwrite(format("%s/media/", SOURCE_DIR) + inputFilename + "_Detected" + ".jpg", detectedOutputImage);
    imwrite(format("%s/media/", SOURCE_DIR) + inputFilename + "_Rejected" + ".jpg", rejectedOutImage);

    namedWindow("Detected Markers", WINDOW_NORMAL);
    imshow("Detected Markers", detectedOutputImage);
    namedWindow("Rejected Markers", WINDOW_NORMAL);
    imshow("Rejected Markers", rejectedOutImage);
    waitKey(0);
}


/**
 * size of marker in pixels
 * generates the markers rotated 180deg from what I generated using the other method
 * not sure which one has the correct orientation
 */
void my_generateMarkers() {
    Mat markerImage;
    const aruco::Dictionary dictionary = getPredefinedDictionary(aruco::DICT_APRILTAG_16h5);
    generateImageMarker(dictionary, 1, 200, markerImage, 1);
    imwrite(format("%s/marker23.png", SOURCE_DIR), markerImage);
}
