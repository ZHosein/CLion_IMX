#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

using namespace std;
using namespace gtsam;

// For loading the data
//************************************************************************* #1#
std::vector<gtsam::Point3> createPoints() {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

  return points;
}

//************************************************************************* #1#
std::vector<gtsam::Pose3> createPoses() {

  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;
  double radius = 30.0;
  int i = 0;
  double theta = 0.0;
  gtsam::Point3 up(0,0,1);
  gtsam::Point3 target(0,0,0);
  for(; i < 8; ++i, theta += 2*M_PI/8) {
    gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
    gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
    poses.push_back(camera.pose());
  }
  return poses;
}
// ************************************************************************* #1#

void test() {
    std::cout << "Hello World!";


  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  noiseModel::Isotropic::shared_ptr noise = //
      noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();

  // Create the set of ground-truth poses
  vector<Pose3> poses = createPoses();

  // Create a NonlinearISAM object which will relinearize and reorder the variables
  // every "relinearizeInterval" updates
  int relinearizeInterval = 3;
  NonlinearISAM isam(relinearizeInterval);

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // Loop over the different poses, adding the observations to iSAM incrementally
  for (size_t i = 0; i < poses.size(); ++i) {

    // Add factors for each landmark observation
    for (size_t j = 0; j < points.size(); ++j) {
      // Create ground truth measurement
      SimpleCamera camera(poses[i], *K);
      Point2 measurement = camera.project(points[j]);
      // Add measurement
      graph.add(
          GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, noise,
              Symbol('x', i), Symbol('l', j), K));
    }

    // Intentionally initialize the variables off from the ground truth
    Pose3 noise(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
    Pose3 initial_xi = poses[i].compose(noise);

    // Add an initial guess for the current pose
    initialEstimate.insert(Symbol('x', i), initial_xi);

    // If this is the first iteration, add a prior on the first pose to set the coordinate frame
    // and a prior on the first landmark to set the scale
    // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
    // adding it to iSAM.
    if (i == 0) {
      // Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
      noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
      graph.add(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));

      // Add a prior on landmark l0
      noiseModel::Isotropic::shared_ptr pointNoise =
          noiseModel::Isotropic::Sigma(3, 0.1);
      graph.add(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise));

      // Add initial guesses to all observed landmarks
      Point3 noise(-0.25, 0.20, 0.15);
      for (size_t j = 0; j < points.size(); ++j) {
        // Intentionally initialize the variables off from the ground truth
        Point3 initial_lj = points[j] + noise;
        initialEstimate.insert(Symbol('l', j), initial_lj);
      }

    } else {
      // Update iSAM with the new factors
      isam.update(graph, initialEstimate);
      Values currentEstimate = isam.estimate();
      cout << "****************************************************" << endl;
      cout << "Frame " << i << ": " << endl;
      currentEstimate.print("Current estimate: ");

      // Clear the factor graph and values for the next iteration
      graph.resize(0);
      initialEstimate.clear();
    }
  }


}
