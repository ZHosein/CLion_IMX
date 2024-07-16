#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>

#include <rerun.hpp>
#include <opencv2/highgui.hpp>

#include "camSpecs.cpp"

cv::aruco::PredefinedDictionaryType dictionaryName = cv::aruco::DICT_4X4_50;
float markerLength = 0.019;
char imageName[] = "testImg2.jpeg";

void logImagePoseRerun () {
    const cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(dictionaryName));
    const cv::Mat image = cv::imread(cv::format("%s/media/%s", SOURCE_DIR, imageName), cv::IMREAD_COLOR);

    // create obj coordinate system
    // constexpr float markerLength = 0.15;
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    // detect markers and estimate pose
    std::vector<int> ids; std::vector<std::vector<cv::Point2f>> corners; detector.detectMarkers(image, corners, ids);
    const size_t nMarkers = corners.size(); std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
    if(!ids.empty()) for (size_t i = 0; i < nMarkers; i++) cv::solvePnP(objPoints, corners.at(i), intrinsicsMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));


    // testing with OpenCV draw/show funcs
    std::cout << "hello\n";
    cv::aruco::drawDetectedMarkers(image, corners, ids);
    for(unsigned int i = 0; i < ids.size(); i++)
        cv::drawFrameAxes(image, intrinsicsMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
    /*cv::namedWindow("pose", cv::WINDOW_NORMAL);
    cv::imshow("pose", image);
    cv::waitKey(0);
    return;*/
    // ------- ---- ------ ----/---- -----

    // log results
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);     // Rerun expects RGB format

    const auto rec = rerun::RecordingStream("Image pose7");
    rec.spawn().exit_on_failure();
    rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);



    //                                  --------------- logging camera ---------------


    // cv::Mat uses row-major order whereas rerun::Mat3x3 uses col-major order
    // consider simply getting and storing the focal length & resolutions we're going to use

    // identity transformation so that the axes show up
    rec.log(
        "world/camera",
        rerun::Transform3D(rerun::RotationAxisAngle({0.0f, 0.0f, 0.0f}, rerun::Angle::radians(0)), 1.0f)
        );


    // ------------------- actual camera setup I'll be using -------------------
    cv::Mat intsMatTrans;
    transpose(intrinsicsMatrix, intsMatTrans);
    const auto cam = rerun::Mat3x3(intsMatTrans.ptr<float>(0));
    rec.log("world/camera/image",
            rerun::Pinhole(rerun::components::PinholeProjection(cam)));

    // ------------------- actual camera setup I'll be using END -------------------



    // ----------------------- Dummy camera for testing -----------------------
    /*rec.log(
        "world/camera/image",
        rerun::Pinhole::from_focal_length_and_resolution({500.0, 500.0}, {640.0, 480.0})
    );*/

    // ----------------------- END Dummy camera for testing -----------------------


    //                          ---------------  logging image---------------


    rec.log("world/camera/image",
            rerun::Image({image.rows, image.cols, image.channels()}, reinterpret_cast<const uint8_t*>(image.data)));



    //                         ---------------  logging corners ---------------


    // logging all corners as one point cloud
    std::vector<rerun::Position2D> corner_positions;
    for (auto marker : corners) {
        for (auto corner : marker) {
            corner_positions.push_back(rerun::Position2D(corner.x, corner.y));
        }
    }
    //TODO: log lines or bounding boxes instead (and perhaps log/project the bounding boxes into the world space as well
    rec.log("world/camera/image/corners", rerun::Points2D(corner_positions).with_radii(2.0f));




    //                               ---------------  logging poses ---------------
    for (size_t i = 0; i < ids.size(); ++i)
    {
        cv::Mat rotMat, rotMatTrans;
        cv::Rodrigues(rvecs[i], rotMat);

        // std::cout << rotMat;
        // std::cout << tvecs[0];

        cv::transpose(rotMat, rotMatTrans);
        // rerun::Mat3x3 rot(reinterpret_cast<float*>(rotMatTrans.ptr<double>(0)));
        std::array<float, 9> rotVals;
        for (int j = 0; j < 9; ++j) {
            rotVals.at(j) = static_cast<float>(rotMatTrans.at<double>(j));
        }
        rerun::Mat3x3 rot(rotVals);

        rerun::Vec3D trans(tvecs[i][0], tvecs[i][1], tvecs[i][2]);


        /*
        rec.log(
            "world/camera/marker/" + std::to_string(ids[0]),
            // rerun::Transform3D(trans, rot, true)
            rerun::Transform3D(trans, rot)
        );*/
        rec.log(
            "world/marker" + std::to_string(ids[i]),
            // rerun::Transform3D(trans, rot, true)
            rerun::Transform3D(trans, rot)
        );
    }

    /*
    // -------------------- example --------------------
    rec.log("world/image", rerun::Pinhole::from_focal_length_and_resolution(3.0f, {3.0f, 3.0f}));
    std::vector<uint8_t> random_data(3 * 3 * 3);
    std::generate(random_data.begin(), random_data.end(), [] {
        return static_cast<uint8_t>(std::rand());
    });
    rec.log("world/image", rerun::Image({3, 3, 3}, random_data));
    // -------------------- end example --------------------
    */

}


/**
 * proposed workflow for pose detection:
 *      load video
 *      loop through frames
 *          detect marker
 *          detect poses
 *          transform data to rerun
 *          log data to rerun
 * porposed workflow with GTSAM
 *      load video
 *      loop through frames
 *          detect markers and poses
 *          use data with whatever else necessary for SLAM
 *          transform data to rerun format
 *          log data to rerun
 */