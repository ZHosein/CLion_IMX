#include <iostream>
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
 * Saves video feed from connected OAK-D device to driectory of .png's scaled to specified resolution
 * @param height: height to scale image to, scaling factor calculatd and applied to width
 * @param dir: directory in which to store images
 * @param tagFamily: DICT_APRILTAG_36h11 | DICT_APRILTAG_16h5 | DICT_4X4_50 (tag families used during testing)
 */
void captureVid(int height, std::string tagFamily) {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and outputs
    const auto colorNode = pipeline.create<dai::node::ColorCamera>();
    const auto xOut = pipeline.create<dai::node::XLinkOut>();

    xOut->setStreamName("colorOut");

    // Properties
    colorNode->setBoardSocket(dai::CameraBoardSocket::CAM_A); // not strictly necessary to specify, would have automatically been chosen by device since there is only one color camera
    colorNode->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K); // resolution for which default inrinsics was calibrated (getDefaultIntrinsics())
    colorNode->setInterleaved(true); // not sure if really necessary
    colorNode->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR); // defualt color order is RGB but openCV uses BGR

    // Linking
    colorNode->video.link(xOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Queues
    auto colorQueue = device.getOutputQueue("colorOut");
    // auto colorQueue = device.getOutputQueue("colorOut", 1);


    // stdout Outputs
    std::cout << "Camera: " <<  colorNode->getName() << std::endl;
    std::cout << "Current Resolution: " << colorNode->getResolutionWidth() << "x" << colorNode->getResolutionHeight() << std::endl;
    std::cout << "Fps: " << colorNode->getFps() << std::endl;

    auto calibration_data = device.readCalibration();
    // calibration_data.eepromToJsonFile("/home/zakareeyah/CLionProjects/TestProj/calibData.json"); //
    auto [defaultIntrinsics, calibWidth, calibHeight] = calibration_data.getDefaultIntrinsics(dai::CameraBoardSocket::CAM_A);
    std::cout << "Resolution at which calibrated: " << calibWidth << "x" << calibHeight << std::endl;
    std::cout << "Default Intrinsics Matrix: " << std::endl;
    printMatrix(defaultIntrinsics);

    json deviceEEPROM = calibration_data.eepromToJson();
    auto ints = deviceEEPROM["/cameraData/2/1/intrinsicMatrix"_json_pointer];
    std::cout << ints << std::endl;
    printMatrix(ints);
    std::cout << deviceEEPROM["/cameraData/2"_json_pointer] << std::endl; // assumes 3 cameras and that the color camer is the last in the list
    std::cout << deviceEEPROM << std::endl;
    double scaleFactor = static_cast<double>(height) / calibHeight;
    int scaledWidth = static_cast<int>(calibWidth * scaleFactor),
        scaledHeight = static_cast<int>(calibHeight * scaleFactor);
    std::cout << "Scaling factor: " << scaleFactor << "\nScaling to: " << scaledWidth << "x" << scaledHeight << std::endl;
    auto scaledIntrinsics = calibration_data.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, scaledWidth, scaledHeight);
    std::cout << "Scaled Intrinsics Matrix";
    printMatrix(scaledIntrinsics);

    // JSON
    /*json info = {
        {"tag family", tagFamily},
        {"intrinsics", scaledIntrinsics}, // row-major
        {"distortion coefficients", },
        {"fps", },
        {"resolution", }
    };*/

    json info;
    info["tagFamily"] = tagFamily;
    info["intrinsics"] = scaledIntrinsics;
    info["distCoeffs"] = calibration_data.getDistortionCoefficients(dai::CameraBoardSocket::CAM_A);
    info["fps"] = colorNode->getFps();
    info["resolution"] = height;

    std::cout << info;


    // Other info about cameras on device
    /*auto features = device.getConnectedCameraFeatures();
    for (auto camera : features) std::cout << camera.name << " "; std::cout << std::endl;
    auto colorFeatures = features[0];
    std::cout << colorFeatures << std::endl << colorFeatures.calibrationResolution->width << " " << colorFeatures.calibrationResolution->height << std::endl;*/




    cv::namedWindow("Video Feed", cv::WINDOW_NORMAL);
    while (true) {
        auto cvFrame = colorQueue->get<dai::ImgFrame>()->getCvFrame(); // getCvFrame converts to BGR interleaved so explicitly setting before is not necessary
        // std::cout << cvFrame.cols << " " << cvFrame.rows;
        cv::Mat resizedFrame;
        cv::resize(cvFrame, resizedFrame, cv::Size(), scaleFactor, scaleFactor);
        cv::imshow("Video Feed", resizedFrame);

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
