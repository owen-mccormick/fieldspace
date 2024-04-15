## Fieldspace

This is intended to be a tool to check the accuracy of mock / practice field setups for the FIRST Robotics Competition through AprilTag pose estimation and a webcam. The idea is that someone could walk around a field setup with a webcam and have visible tags plotted on a map relative to an assumed correct seed tag.

Not sure how practical this is for field setup - mostly just a tech demo.

### Building
This builds the GResource bundle with assets and compiles the CMake project:

```bash
$ glib-compile-resources --target=resources/resources.c --generate-source resources/resources.gresource.xml
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .
```
