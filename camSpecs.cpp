#include <opencv2/core.hpp>

/*
 * Inrinsics Matrix:
 *
 *      f_x   0    c_x
 *      0     f_y  c_y
 *      0     0    1
 *
 * ``f_x and f_y are essentially the focal length epxressed in pixels.
 * (We introduce 2 new params because each pixel on a typical image is rectangluar,
 *      hence the pixle lengths in x and y are different
 *      [the physical focal length f, expressed in millimeters, cannot be measured directly]
 *
 * c_x (float) – The x-axis optical center of the camera in pixels.
 * c_y (float) – The y-axis optical center of the camera in pixels.
 * c_x and c_y are two parameters that handle possible misalignment of the principal point with the center of the image;
 */

cv::Mat intrinsicsMatrix = (cv::Mat_<float>(3,3) <<
    3075.7952, 0, 1927.995,
    0, 3075.7952, 1078.1343,
    0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<float>(1, 14) << 2.0888574, -82.303825, -0.00071347022, 0.0020022474, 315.66144,
    1.8588818, -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);
