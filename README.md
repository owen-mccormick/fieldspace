## Fieldspace

This is intended to be a tool to check the accuracy of mock / practice field setups for the FIRST Robotics Competition through AprilTag pose estimation and a webcam. The idea is that someone could walk around a field setup with a webcam and have visible tags plotted on a map relative to an assumed correct seed tag seen in the same image.

Not sure how practical this is for field setup - mostly just a tech demo.

### Building
Depends on OpenCV, Gtkmm, the [University of Michigan AprilTag library](https://github.com/AprilRobotics/apriltag), and the [Eigen C++ linear algebra library](https://eigen.tuxfamily.org/index.php?title=Main_Page).
This compiles the CMake project (currently needs to be done from project root directory to find stuff in resources folder by path):

```bash
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .
```
