#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <depthai/depthai.hpp>


void displayLeftAndRight() {
    dai::Pipeline pipeline;

    const auto leftCamNode = pipeline.create<dai::node::MonoCamera>();
    const auto rightCamNode = pipeline.create<dai::node::MonoCamera>();
    leftCamNode->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    rightCamNode->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    const auto xOutLeft = pipeline.create<dai::node::XLinkOut>();
    const auto xOutRight = pipeline.create<dai::node::XLinkOut>();
    xOutLeft->setStreamName("leftStream");
    xOutRight->setStreamName("rightStream");

    leftCamNode->out.link(xOutLeft->input);
    rightCamNode->out.link(xOutRight->input);

    dai::Device device(pipeline);

    const auto leftQueue = device.getOutputQueue("leftStream", 1);
    const auto rightQueue = device.getOutputQueue("rightStream", 1);

    cv::namedWindow("leftRight", cv::WINDOW_NORMAL);

    while (true) {
        double alpha = 0.5;
        const auto leftFrame = leftQueue->get<dai::ImgFrame>()->getCvFrame();
        const auto rightFrame = rightQueue->get<dai::ImgFrame>()->getCvFrame();
        cv::Mat leftRightBlend;

        // blending images
        cv::addWeighted(leftFrame, alpha, rightFrame, alpha ,0.0, leftRightBlend);
        cv::imshow("leftRight", leftRightBlend);

        if (const auto k = cv::waitKey(1); k == 27) return;
    }
}

void displayLeft() {
    dai::Pipeline pipeline; // declare a pipline obj

    // create a monochrom node and "link" it to the left cam
    const auto leftCam = pipeline.create<dai::node::MonoCamera>();
    leftCam->setBoardSocket(dai::CameraBoardSocket::CAM_B);     // leftCam->setCamera("CAM_B");

    // create an xlink out stream and name it (Xlink Nodes are resposible for carrying data to host
    const auto xOut = pipeline.create<dai::node::XLinkOut>();
    xOut->setStreamName("left");

    // link the xlink stream to the leftCam
    leftCam->out.link(xOut->input);

    // link camera (device) to host (set up the pipeline on the device)
    dai::Device device(pipeline);

    // get a ptr to the output queue of the xlinkout node's stream (which has the output from the left mono cam node)
    const auto leftQueue = device.getOutputQueue("left");

    while (true) {
        // get frame from queue
        const auto frame(leftQueue->get<dai::ImgFrame>());

        // convert to OpenCV frame
        cv::Mat cvFrame = frame->getCvFrame();

        // dsiaply the frame
        cv::imshow("left", cvFrame);
        if (const char key = static_cast<char>(cv::waitKey(1)); key == 27) return;
    }

}

/**
 * sample from depthAI docs with some OpenCV code I added for marker detection
 *
 * displays video feed from color camera
 */
void display_vid_sample () {
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    colorCam->setInterleaved(true);
    colorCam->preview.link(xlinkOut->input);

    try {
        dai::Device device(pipeline);

        auto preview = device.getOutputQueue("preview");

        cv::Mat frame;
        while (true) {
            auto imgFrame(preview->get<dai::ImgFrame>());

            cv::Mat openCVFrame = cv::Mat(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3,
                                          imgFrame->getData().data());
            //my code------------------------------------------------
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners, rejected;

            cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));

            detector.detectMarkers(openCVFrame, corners, ids);
            if(!ids.empty())
                cv::aruco::drawDetectedMarkers(openCVFrame, corners, ids);

            cv::namedWindow("preview", cv::WINDOW_NORMAL);

            //END:my code-------------------------------------------
            cv::imshow("preview", openCVFrame);
            if (cv::waitKey(1) == 'q') return;
        }
    }
    catch (const std::runtime_error &error) {
        std::cout << error.what() << std::endl;
    }
}
