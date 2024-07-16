#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <depthai/depthai.hpp>

cv::Mat intrinsicsMatrix = (cv::Mat_<float>(3,3) <<
    3075.7952, 0, 1927.995,
    0, 3075.7952, 1078.1343,
    0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<float>(1, 14) << 2.0888574, -82.303825, -0.00071347022, 0.0020022474, 315.66144,
    1.8588818, -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);


void dispVidPose () {
    dai::Pipeline pipeline;
    auto colorNode  = pipeline.create<dai::node::ColorCamera>();
    auto xOut = pipeline.create<dai::node::XLinkOut>();
    xOut->setStreamName("colorStream");
    colorNode->preview.link(xOut->input);

    dai::Device device(pipeline);
    auto outQueue = device.getOutputQueue("colorStream");


    // set coordinate system
    float markerLength = 0.019;
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));

    cv::namedWindow("out", cv::WINDOW_NORMAL);
    while (true) {
        cv::Mat image = outQueue->get<dai::ImgFrame>()->getCvFrame();


        cv::Mat imageCopy;


        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;

        // detect markers and estimate pose
        detector.detectMarkers(image, corners, ids, rejected);

        size_t nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if(!ids.empty()) {
            // Calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++) {
                cv::solvePnP(objPoints, corners.at(i), intrinsicsMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
        }

        // draw results
        image.copyTo(imageCopy);
        if(!ids.empty()) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);


            for(unsigned int i = 0; i < ids.size(); i++)
                cv::drawFrameAxes(imageCopy, intrinsicsMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
        }


        imshow("out", imageCopy);
        char key = static_cast<char>(cv::waitKey(10));
        if(key == 27) return;
    }
    std::cout << "Hello World!";
}
