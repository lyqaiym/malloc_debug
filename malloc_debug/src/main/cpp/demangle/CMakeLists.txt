# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)
include_directories(./)
# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.
aux_source_directory(./ NATIVE_SRC)
#set(distribution_DIR ${CMAKE_SOURCE_DIR}/libs/${ANDROID_ABI})
#add_library(XXXX SHARED IMPORTED)

add_definitions(-D_7ZIP_ST)

#message(DEBUG "distribution_DIR=${distribution_DIR}")
message(DEBUG "CMAKE_LIBRARY_OUTPUT_DIRECTORY=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(DEBUG "CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}")

add_library( # Sets the name of the library.
        demangle

             # Sets the library as a shared library.
        SHARED

             # Provides a relative path to your source file(s).
        demangle.cpp demangle_fuzzer.cpp Demangler.cpp
        )

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log)

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.
#target_link_libraries(c_malloc_debug ${CMAKE_SOURCE_DIR}/malloc_debug/libc_malloc_debug.so)
target_link_libraries( # Specifies the target library.
        demangle
                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} )