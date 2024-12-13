cmake_minimum_required(VERSION 3.10.2)

# 导入ncnn库
set(ncnn_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ncnn-20231027-android-vulkan/${ANDROID_ABI}/lib/cmake/ncnn)
find_package(ncnn REQUIRED)

# opencv
set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opencv/sdk/native/jni)
find_package(OpenCV REQUIRED core imgproc highgui features2d)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/opencv/sdk/native/jni/include)

add_library(libopencv_java4 SHARED IMPORTED)
set_target_properties(libopencv_java4 PROPERTIES IMPORTED_LOCATION
        ${CMAKE_CURRENT_SOURCE_DIR}/opencv/sdk/native/libs/${ANDROID_ABI}/libopencv_java4.so)

file(GLOB jni_srcs
        "*.cpp"
        "live/*.cpp"
        "age/*.cpp"
        "face/*.cpp"
        )
add_library(engine SHARED ${jni_srcs})

target_link_libraries(engine log jnigraphics ncnn libopencv_java4 ${OpenCV_LIBS} android)