#include <iostream>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <depthai/depthai.hpp>



void detectPose_Color() {
    // setting up aruco detector
    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    const cv::aruco::ArucoDetector detector(dictionary); // using defualt parameters; no need to pass in.

    // setting up OAK
    dai::Pipeline pipeline;
    const auto colorNode = pipeline.create<dai::node::ColorCamera>();
    const auto xOut = pipeline.create<dai::node::XLinkOut>();
    xOut->setStreamName("color");
    colorNode->preview.link(xOut->input);

    dai::Device device(pipeline);
    dai::CalibrationHandler calibrationData = device.readCalibration();
    auto [instrinsics, width, height] = calibrationData.getDefaultIntrinsics(dai::CameraBoardSocket::CAM_A);
    auto distCoeffs = calibrationData.getDistortionCoefficients(dai::CameraBoardSocket::CAM_A);

    cv::Mat intrinsicsMat(3, 3, CV_32F);
    for(int i=0; i<intrinsicsMat.rows; ++i)
        for(int j=0; j<intrinsicsMat.cols; ++j) {
            intrinsicsMat.at<float>(i, j) = instrinsics.at(i).at(j);
        }
    std::cout << intrinsicsMat;
    cv::Mat distCoeffsMat(1, distCoeffs.size(), CV_32F);
    for (int i = 0; i < distCoeffs.size(); ++i) {
        distCoeffsMat.at<float>(0, i) = distCoeffs.at(i);
    }
    std::cout << distCoeffsMat;
    return;

    const auto colorQueue = device.getOutputQueue("color", 1);


    //creating marker obj in coordinate sys
    float markerLength = 0.019; //m
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    cv::namedWindow("output", cv::WINDOW_NORMAL);
    while (true) {
        cv::Mat frameCopy,
                frame = colorQueue->get<dai::ImgFrame>()->getCvFrame();

        // detecting markers and pose
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(frame, corners, ids);

        size_t nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        if (!ids.empty())
            for (size_t i = 0; i < nMarkers; i++)
                cv::solvePnP(objPoints, corners.at(i), intrinsicsMat, distCoeffsMat, rvecs.at(i), tvecs.at(i));

        // drawing and displaying
        frame.copyTo(frameCopy);
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frameCopy, corners, ids);
            for (unsigned int i =0; i < ids.size(); ++i) {
                cv::drawFrameAxes(frame, intrinsicsMat, distCoeffsMat, rvecs[i], tvecs[i], markerLength * 20.f, 10);
            }
        }

        cv::imshow("output", frameCopy);

        if (cv::waitKey(1) == 27) return;
    }

}