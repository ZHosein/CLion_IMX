#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
// #include "opencv2/opencv.hpp"

#include <iostream>

using namespace cv;

int displayAndSaveImage_Tutorial() {
    samples::addSamplesDataSearchPath("/home/zakareeyah/CLionProjects/TestProj/media/");
    const std::string image_path = samples::findFile("mouse-cropped.png");
    // const std::string image_path = samples::findFile("starry_night.jpg");
    Mat image = imread(image_path, IMREAD_COLOR);
    // with regard to falg meanings: https://docs.opencv.org/4.x/db/deb/tutorial_display_image.html#:~:text=IMREAD_COLOR%20loads%20the,an%20intensity%20one


    if (image.empty()) {
        std::cout << "Could not read image: " << image_path << std::endl;
        return -1;
    }

    imshow("Display Image", image); // displays image in window with name: winname (1st arg)
    int k = waitKey(0); // awiati keystroke in window

    if (k == 's')
        imwrite(format("%s/starrynight.png", SOURCE_DIR), image);

    return 0;
}

int displagImg_Tutorial() {
    // const String imagePath = "/home/zakareeyah/CLionProjects/TestProj/media/mouse.jpg";
    const String imagePath = "/home/zakareeyah/CLionProjects/TestProj/media/mouse-cropped.png";

    Mat image = imread(imagePath, IMREAD_COLOR);

    if (!image.data) {
        printf("No image data\n");
        return -1;
    }
    std::cout << "Displaying image." << std::endl;
    namedWindow("Display Image", WINDOW_NORMAL);
    imshow("Display Image", image);

    waitKey(0);
    return 0;
}
