# ArUcoDetector Native Plugin

Native C++ plugin for Unity that provides ArUco marker detection using OpenCV. This library is developed as part of a bachelor's thesis for the main [AR Manual project](https://github.com/CoolLikeGoose/ArManual).

The project in its current state implements the essential native functionality required for the thesis application, with a compact API designed for Unity integration.

## Architecture and Structure

The plugin is intentionally minimal and organized around a single native module:

- **`src/ArUcoPlugin.cpp`** - Core implementation of ArUco detection, marker filtering, and pose estimation
- **`CMakeLists.txt`** - Cross-platform CMake configuration for Android builds
- **`android_build.bat`** - Helper script for Android NDK build configuration on Windows

The exported interface Unity can call through native bindings - `DLLImport`.

## Installation

### Requirements

- CMake 3.10+
- C++17 compatible compiler
- [OpenCV SDK](https://github.com/opencv/opencv)
- Ninja
- Android NDK (for Android builds)

### Setup

1. Clone the repository and navigate to the project directory:
```bash
cd ArUcoDetector
```

2. Configure the project with OpenCV SDK path in `CMakeLists.txt`:
```bash
set(OpenCV_DIR "OpenCV_SDK_PATH")
```

3. Configure Android SDK path in `android_build.bat` if used:
```bash
set SDK_ROOT=Android_SDK_PATH
```

4. For automatic export in unity set desired directory in `Makefile`:
```bash
UNITY_PLUGIN_DIR=
```

## Building

### For Windows

For automatic build and export run:
```bash
make
```
If you dont want automatic export, use:
```bash
make compile
``` 

### Other platforms
You need to build manually using `CMakeLists.txt`.

## Exported Functionality

The plugin exports the following native functions:

- **`ArucoInit(int dictionaryId)`** - Initializes the detector with a predefined OpenCV ArUco dictionary
- **`ArucoSetWhitelist(int* markerIds, int count)`** - Enables filtering so only selected marker IDs are kept
- **`ArucoSetCameraIntrinsics(float fx, float fy, float cx, float cy)`** - Update the camera intrinsic parameters
- **`ArucoProcess(uint8_t* rgba, int width, int height)`** - Converts and processes an RGBA32 frame, detects markers, refines corners, and applies whitelist filtering
- **`ArucoGetCount()`** - Returns the number of detected markers from the last processed frame
- **`ArucoGetId(int index)`** - Returns the detected marker ID at the given index
- **`ArucoGetCornersByID(int arucoId, float* cornerX, float* cornerY)`** - Returns the four corner points for a detected marker
- **`ArucoEstimatePose(int arucoId, float markerSize, float* rvec, float* tvec)`** - Estimates marker pose using OpenCV solvePnP 

## Technology Stack

- **C++17** - Native plugin implementation
- **OpenCV** - Marker detection and computer vision operations
- **CMake**, **Make** - Build system configuration
- **Android NDK** - Android native compilation