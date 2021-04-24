# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

include_directories(./include)
include_directories(./lzma)
# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.
#ADD_SUBDIRECTORY(./biz/CMakeLists.txt)
#ADD_SUBDIRECTORY(./xhook/CMakeLists.txt)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../)
ADD_SUBDIRECTORY(./demangle)
ADD_SUBDIRECTORY(./lzma)
ADD_SUBDIRECTORY(./unwindstack)
ADD_SUBDIRECTORY(./malloc_debug)
