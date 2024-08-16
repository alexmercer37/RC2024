cmake_minimum_required(VERSION 3.16)

include_directories(.)

project(camera)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/workspace)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
set(CUDA_GEN_CODE "-gencode=arch=compute_87,code=sm_87")

set(PROTOBUF_DIR "/home/nf/Downloads/protobuf-3.11.4")

# 建议cuda10.2以上
set(CUDA_DIR     "/usr/local/cuda")

# 需要与cuda、tensorRT配对
set(CUDNN_DIR    "/usr/local/cuda")
set(TENSORRT_DIR "/usr/src/tensorrt")

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(k4a REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/application
    ${PROJECT_SOURCE_DIR}/src/tensorRT
    ${PROJECT_SOURCE_DIR}/src/tensorRT/common    
    ${PCL_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_DIR}/include
    ${PROTOBUF_DIR}/include
    ${TENSORRT_DIR}/include
    ${CUDNN_DIR}/include
    )
link_directories(
    ${PCL_LIBRARY_DIRS} 
    ${OpenCV_LIBS_DIRS}
    ${PROTOBUF_DIR}/lib
    ${CUDA_DIR}/lib64
    ${CUDNN_DIR}/lib
    ${TENSORRT_DIR}/lib
    )
add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS})
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O0 -Wfatal-errors -pthread -w -g")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O0 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")
file(GLOB_RECURSE cpp_srcs ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE cuda_srcs ${PROJECT_SOURCE_DIR}/src/*.cu)
cuda_add_library(plugin_list SHARED ${cuda_srcs})

add_executable(camera ${cpp_srcs})
# add_executable(camera src/main.cpp src/pointcloud.cpp src/opencv.cpp src/camera.cpp src/application/app_yolo/yolo.cpp)

# add_executable(seePLY src/camera.cpp src/seePLY.cpp)

target_link_libraries(camera ${PCL_LIBRARIES} ${OpenCV_LIBS} k4a::k4a)
target_link_libraries(camera nvinfer nvinfer_plugin)
target_link_libraries(camera cuda cublas cudart cudnn)
target_link_libraries(camera protobuf pthread plugin_list)
# target_link_libraries(selectColor ${PCL_LIBRARIES} ${OpenCV_LIBS} k4a::k4a)
# target_link_libraries(seePLY ${PCL_LIBRARIES} ${OpenCV_LIBS} k4a::k4a)

add_custom_target(
        yolo
        DEPENDS camera
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/workspace
        COMMAND ../camera yolo
)

# # 查看点云
# project(v2)

# set(PCL_INCLUDE_DIRS  "/usr/local/include/pcl-1.13")

# find_package(k4a REQUIRED)
# find_package(PCL REQUIRED)

# include_directories(.)
# include_directories( 
#     ${PCL_INCLUDE_DIRS}
# )
# add_executable(v2 src1/seePLY.cpp)
# target_link_libraries(v2 k4a::k4a)
# target_link_libraries(v2 
#     ${PCL_LIBRARIES}
# )
