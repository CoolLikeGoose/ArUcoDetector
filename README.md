# ArUcoDetector

A minimalist C++ shared library that wraps OpenCV's ArUco marker
detection for use in Unity projects. The native plugin exposes a small
C-compatible API so that Unity can feed camera frames and query
marker information via `DllImport`.

This repo forms the part of a bachelor project focused on
augmented reality and marker tracking.

## Key features

* Detects ArUco markers in RGBA32 images coming from Unity.
* Optional whitelist of allowed marker IDs.
* Pose estimation using OpenCV `solvePnP`.
* Builds for Windows/desktop and Android (NDK).