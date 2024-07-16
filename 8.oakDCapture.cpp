#include <iostream>
#include <depthai/depthai.hpp>

void captureImage_FullResolution() {
    dai::Pipeline pipeline;
    auto colorNode = pipeline.create<dai::node::ColorCamera>();
    auto xOut = pipeline.create<dai::node::XLinkOut>();

    xOut->setStreamName("colorOut");

    colorNode->setBoardSocket(dai::CameraBoardSocket::CAM_A); // not strictly necessary to specify, would have automatically been chosen by device
    colorNode->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    colorNode->setInterleaved(true); // not sure if really necessary
    colorNode->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR); // defualt color order is RGB but openCV uses BGR

    colorNode->video.link(xOut->input);

    dai::Device device(pipeline);

    auto colorQueue = device.getOutputQueue("colorOut", 1);

    cv::namedWindow("Video Feed", cv::WINDOW_NORMAL);
    cv::namedWindow("Capture", cv::WINDOW_NORMAL);
    cv::Mat image;
    while (true) {
        auto cvFrame = colorQueue->get<dai::ImgFrame>()->getCvFrame(); // getCvFrame converts to BGR interleaved so explicitly setting before is not necessary
        cv::imshow("Video Feed", cvFrame);

        auto key = cv::waitKey(10);
        if (key == 13) { // Enter key pressed
            image = cvFrame.clone();
            cv::imshow("Capture", image);
        } else if (key == 'c') {
            cv::imwrite("/home/zakareeyah/CLionProjects/TestProj/media/rgb_data/testImg2.jpeg", image);
            std::cout << "Saved.\n";
            std::cout << "res: " << image.cols << "x" << image.rows << std::endl;
        } else if (key == 27) return;

    }

}

// attempts 2 and 3
#include <chrono>
#include <iostream>
int save_jpg() {
    /*
    //-------------------------------------attempt 2-------------------------------------
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRGB = pipeline.create<dai::node::XLinkOut>();
    auto xin = pipeline.create<dai::node::XLinkIn>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    auto xoutStill = pipeline.create<dai::node::XLinkOut>();

    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);    // using default res (4K)
    xoutRGB->setStreamName("rgb");
    xin->setStreamName("control");
    xoutStill->setStreamName("still");
    videoEnc->setDefaultProfilePreset(camRgb->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);

    camRgb->video.link(xoutRGB->input);
    xin->out.link(camRgb->inputControl);
    camRgb->still.link(videoEnc->input);
    videoEnc->bitstream.link(xoutStill->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto qRgb = device.getOutputQueue("rgb", 30, false);
    auto qStill = device.getOutputQueue("still", 30, true);
    auto qControl = device.getInputQueue("control");

    std::string dirName = "/home/zakareeyah/CLionProjects/TestProj/media/rgb_data";

    cv::namedWindow("rgb", cv::WINDOW_NORMAL);
    while(true) {
        auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        if(inRgb != NULL) {
            cv::imshow("rgb", inRgb->getCvFrame());
        }

        auto encFrames = qStill->tryGetAll<dai::ImgFrame>();
        for(const auto& encFrame : encFrames) {
            uint64_t time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            std::stringstream videoStr;
            videoStr << dirName << "/" << time << ".jpeg";
            auto videoFile = std::ofstream(videoStr.str(), std::ios::binary);
            videoFile.write((char*)encFrame->getData().data(), encFrame->getData().size());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
        if (key == 'c') {
            auto ctrl = dai::CameraControl();
            ctrl.setCaptureStill(true);
            qControl->send(ctrl);
            std::cout << "Still sent\n";
        }
    }
    */

    //-------------------------------------attempt 3-------------------------------------


    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    auto xoutJpeg = pipeline.create<dai::node::XLinkOut>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

    xoutJpeg->setStreamName("jpeg");
    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    videoEnc->setDefaultProfilePreset(camRgb->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);

    // Linking
    camRgb->preview.link(xoutRgb->input);
    camRgb->video.link(videoEnc->input);
    videoEnc->bitstream.link(xoutJpeg->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Queues
    auto qRgb = device.getOutputQueue("rgb", 30, false);
    auto qJpeg = device.getOutputQueue("jpeg", 30, true);

    std::string dirName = "/home/zakareeyah/CLionProjects/TestProj/media/rgb_data";

    cv::namedWindow("rgb", cv::WINDOW_NORMAL);
    while(true) {
        auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        if(inRgb != NULL) {
            cv::imshow("rgb", inRgb->getCvFrame());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q')
            return 0;
        if (key == 'c') {
            auto encFrame = qJpeg->tryGet<dai::ImgFrame>();
            uint64_t time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            std::stringstream videoStr;
            videoStr << dirName << "/" << time << ".jpeg";
            auto videoFile = std::ofstream(videoStr.str(), std::ios::binary);
            videoFile.write((char*)encFrame->getData().data(), encFrame->getData().size());
        }
    }

}
