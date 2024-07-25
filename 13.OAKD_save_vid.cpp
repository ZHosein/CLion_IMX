#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <depthai/depthai.hpp>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * Helper function for printing matrices; found on depthAI API docs
 * @param matrix: matrix to be printed (basically a 2d vector (ie. a vector of vectors)
 */
void printMatrix(const std::vector<std::vector<float>>& matrix) {
    using namespace std;
    std::string out = "[";
    for(const auto& row : matrix) {
        out += "[";
        for(const auto val : row) out += to_string(val) + ", ";
        out = out.substr(0, out.size() - 2) + "]\n";
    }
    out = out.substr(0, out.size() - 1) + "]\n\n";
    std::cout << out;
}

/**
 * @brief Saves video feed from connected OAK-D device to driectory of .png's scaled to specified resolution
 * @param height: height to scale image to, scaling factor calculatd and applied to width
 * @param tagFamily: DICT_APRILTAG_36h11 | DICT_APRILTAG_16h5 | DICT_4X4_50 (tag families used during testing)
 * @param dir: directory in which to store images; must already exist
 */
void captureVid(int height, const std::string& tagFamily, const char dir[]) {

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and outputs
    const auto colorNode = pipeline.create<dai::node::ColorCamera>();
    const auto xOut = pipeline.create<dai::node::XLinkOut>();
    auto controlIn = pipeline.create<dai::node::XLinkIn>(); // af (fr adjstng autofcs)

    xOut->setStreamName("colorOut");
    controlIn->setStreamName("control"); // af

    // Properties
    colorNode->setBoardSocket(dai::CameraBoardSocket::CAM_A); // not strictly necessary to specify, would have automatically been chosen by device since there is only one color camera
    colorNode->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K); // resolution for which default inrinsics was calibrated (getDefaultIntrinsics())
    colorNode->setInterleaved(true); // not sure if really necessary
    colorNode->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR); // defualt color order is RGB but openCV uses BGR

    // Linking
    colorNode->video.link(xOut->input);
    controlIn->out.link(colorNode->inputControl); // af

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Queues
    auto colorQueue = device.getOutputQueue("colorOut", 1000); // set a queue limit? non-blocking?
    auto controlQueue = device.getInputQueue("control"); // af

    // Adjust autofocus
    /*dai::CameraControl ctrl;
    ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
    ctrl.setAutoFocusTrigger();
    controlQueue->send(ctrl);*/

    // Accesing/calc camera/recording properties
    auto calibration_data = device.readCalibration();
    auto [defaultIntrinsics, calibWidth, calibHeight] = calibration_data.getDefaultIntrinsics(dai::CameraBoardSocket::CAM_A);
    double scaleFactor = static_cast<double>(height) / calibHeight;
    int scaledWidth = static_cast<int>(calibWidth * scaleFactor),
        scaledHeight = static_cast<int>(calibHeight * scaleFactor);
    auto scaledIntrinsics = calibration_data.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, scaledWidth, scaledHeight);


    // std output
    std::cout << "Camera: " <<  colorNode->getName() << std::endl;
    std::cout << "Current Resolution: " << colorNode->getResolutionWidth() << "x" << colorNode->getResolutionHeight() << std::endl;
    std::cout << "Resolution at which calibrated: " << calibWidth << "x" << calibHeight << std::endl;
    std::cout << "Default Intrinsics Matrix: " << std::endl;
    printMatrix(defaultIntrinsics);
    std::cout << "Scaling factor: " << scaleFactor << "\nScaling to: " << scaledWidth << "x" << scaledHeight << std::endl;
    std::cout << "Scaled Intrinsics Matrix" << std::endl;
    printMatrix(scaledIntrinsics);


    // JSON output
    json info;
    info["tagFamily"] = tagFamily;
    info["intrinsics"] = scaledIntrinsics;
    info["distCoeffs"] = calibration_data.getDistortionCoefficients(dai::CameraBoardSocket::CAM_A);
    info["fps"] = colorNode->getFps();
    info["resolution"] = height;

    std::cout << "Info saved to " << "info.json (move JSON file into folder " << dir << "):\n" << info.dump(4) << std::endl;
    std::ofstream infoFile(cv::format("%s/%s/info.json", SOURCE_DIR, dir));
    if (!infoFile.is_open()) {
        std::cout << "could not open json file. Ensure that directory " << dir << " exists.";
        std::cout << "Exiting...";
        return;
    }
    infoFile << std::setw(4) << info;

    // Other info about cameras on device
    /*auto features = device.getConnectedCameraFeatures();
    for (auto camera : features) std::cout << camera.name << " "; std::cout << std::endl;
    auto colorFeatures = features[0];
    std::cout << colorFeatures << std::endl << colorFeatures.calibrationResolution->width << " " << colorFeatures.calibrationResolution->height << std::endl;*/



    cv::namedWindow("Video Feed", cv::WINDOW_NORMAL);
    int imageNum = 0;
    while (true) {
        auto cvFrame = colorQueue->get<dai::ImgFrame>()->getCvFrame(); // getCvFrame converts to BGR interleaved so explicitly setting before is not necessary
        cv::Mat resizedFrame;
        cv::resize(cvFrame, resizedFrame, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA); // cv::INTER_AREA recommended by OpenCV docs for scaling down

        cv::imshow("Video Feed", resizedFrame);
        cv::String filename = cv::format("%s/%s/image%d.png", SOURCE_DIR, dir, imageNum);
        if(!cv::imwrite(filename, resizedFrame)) {
            std::cout << "Could not save image. Ensure that path: " << filename << " exists.";
            std::cout << "Exiting...";
            return;
        }
        imageNum++;
        if (const auto key = cv::waitKey(1); key == 27) return;
    }

}


/**
 * Resolution THE_4_K: 3840x2160 (WxH) [this corresponds to the no. of ColsxRows in the cvFrame]
 *
 * Connected cameras (to device): color left right
 *
 * Color camera features:
 *      {socket: CAM_A, sensorName: IMX378, width: 4056, height: 3040, orientation: AUTO, supportedTypes: [COLOR],
 *      hasAutofocus: 1, hasAutofocusIC: 1, name: color}
 *
 *      Recommended calib. res.: 4056x3040 (THE_12_MP)
 * from getDefaultIntrinsics(dai::CameraBoardSocket::CAM_A)
 *      calibration resolution: 3840x1260
 *      Default Intrinsics Matrix:
            [[3075.795166, 0.000000, 1927.994995]
            [0.000000, 3075.795166, 1078.134277]
            [0.000000, 0.000000, 1.000000]]
 * fps: 30
 */
