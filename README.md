# LIDAR Viewer
Point cloud viewer with surface reconstruction for LIDAR data using OpenGL

# Structure

## OGLW

This project is a object oriented wrapper for `OpenGL`. It offers utility functions for shaders, memory and window management.

## OGLWTest

This project is ment is a demo project to show and test functions of `OGLW`.

## LIDARViewer

This project uses `OpenGL` to render point clouds, i.e. the recordings of LIDAR. It creates a 3D world in which the user can navigate using mouse and keyboard. If the project is built against the `PointCloudLibrary` one can also use surface reconstruction to obtain surfaces based on the recorded point cloud.

# How to build

1. Open the Visual Studio 2017 solution
2. Restore the nuget packages
    - glfw
    - glm
    - boost [Only required if `pcl` is *not* used]
2. Install and setup [vcpkg](https://github.com/Microsoft/vcpkg)
3. [Optional] Install `pcl` ([PointCloudLibrary](http://pointclouds.org/)) via `vcpkg`
4. Build `OGLW`
5. [Optional] Set precompiler directive `USE_PCL` to enable `pcl` support 
6. Build `LIDARViewer`