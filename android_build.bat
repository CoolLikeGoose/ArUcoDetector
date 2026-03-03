@echo off
set SDK_ROOT=C:/Users/thekr/AppData/Local/Android/Sdk

set NDK=%SDK_ROOT%/ndk/27.0.12077973

set CMAKE=%SDK_ROOT%/cmake/3.22.1/bin/cmake.exe


rem 
if not exist build (
    mkdir build
)
cd build

rem
%CMAKE% -G "Ninja" ^
 -DANDROID_ABI=arm64-v8a ^
 -DANDROID_PLATFORM=android-30 ^
 -DCMAKE_TOOLCHAIN_FILE=%NDK%/build/cmake/android.toolchain.cmake ^
 -DCMAKE_BUILD_TYPE=Release ^
 ..

rem
%CMAKE% --build .