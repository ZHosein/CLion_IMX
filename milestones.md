# Concrete Project Milestones
- Used OpenCV to detect markers in image from internet
- Got 1080p (default) intrinsics matrix from OAK
- Captured 1080p image using OAK (of ARUCO calib board (DICT_...))
- Pose detection of fiducials in 1080p image
- Rendering pose detection in rerun
- Got Dockerfile and added cmds to cross-compile OpenCV 4.6.0
- Created run configuration in IDE to edit script on laptop, build in docker container, and execute on IMX (ssh)



decided to rewrite script to work with cv 4.6.0\
    don't know the best configuration for build flags.