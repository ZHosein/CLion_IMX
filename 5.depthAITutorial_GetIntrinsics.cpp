#include <iostream>

#include <depthai-shared//common/CameraBoardSocket.hpp>
#include <depthai-shared/common/EepromData.hpp>
#include <depthai/depthai.hpp>


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
 * found on depthAI API docs
 */
void printCameraIntrinsics() {
    dai::Device device;

    const dai::CalibrationHandler calibData = device.readCalibration();

    std::cout << "Intrinsics from defaultIntrinsics function:" << std::endl;
    auto [intrinsics, width, height] = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::CAM_A);
    printMatrix(intrinsics);

    std::cout << "Width: " << width << std::endl;
    std::cout << "Height: " << height << std::endl;

    auto disCoeff = calibData.getDistortionCoefficients(dai::CameraBoardSocket::CAM_A);

    auto disCoeffB = calibData.getDistortionCoefficients(dai::CameraBoardSocket::CAM_B);

    auto disCoeffC = calibData.getDistortionCoefficients(dai::CameraBoardSocket::CAM_C);

    std::string coeffString = "[";
    for(const auto& coeff : disCoeff) coeffString += std::to_string(coeff) + ", ";
    coeffString = coeffString.substr(0, coeffString.size() - 2) + "]\n";
    std::cout << coeffString;
    std::string coeffStringB = "[";
    for(const auto& coeff : disCoeff) coeffStringB += std::to_string(coeff) + ", ";
    coeffStringB = coeffStringB.substr(0, coeffStringB.size() - 2) + "]\n";
    std::cout << coeffStringB;
    std::string coeffStringC = "[";
    for(const auto& coeff : disCoeff) coeffStringC += std::to_string(coeff) + ", ";
    coeffStringC = coeffStringC.substr(0, coeffStringC.size() - 2) + "]\n";
    std::cout << coeffStringC;



    // other intrinsics/extrinsics data including for different "resolutions" (widths and heights)
    std::cout << "Stereo baseline distance: " << calibData.getBaselineDistance() << " cm" << std::endl;

    std::cout << "Mono FOV from camera specs: " << calibData.getFov(dai::CameraBoardSocket::CAM_B)
         << ", calculated FOV: " << calibData.getFov(dai::CameraBoardSocket::CAM_B, false) << std::endl;

    std::cout << "Intrinsics from getCameraIntrinsics function full resolution:" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A);
    printMatrix(intrinsics);

    std::cout << "Intrinsics from getCameraIntrinsics function 1280 x 720:" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, 1280, 720);
    printMatrix(intrinsics);

    std::cout << "Intrinsics from getCameraIntrinsics function 720 x 450:" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, 720);
    printMatrix(intrinsics);

    std::cout << "Intrinsics from getCameraIntrinsics function 600 x 1280:" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, 600, 1280);
    printMatrix(intrinsics);

    std::vector<std::vector<float>> extrinsics;

    std::cout << "Extrinsics from left->right test:" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C);
    printMatrix(extrinsics);

    std::cout << "Extrinsics from right->left test:" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B);
    printMatrix(extrinsics);

    std::cout << "Extrinsics from right->rgb test:" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_A);
    printMatrix(extrinsics);

    std::cout << "Extrinsics from rgb->right test:" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_C);
    printMatrix(extrinsics);

    std::cout << "Extrinsics from left->rgb test:" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_A);
    printMatrix(extrinsics);



}