cmake_minimum_required(VERSION 2.8.3)
project(camera_model)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    )

find_package(Boost REQUIRED COMPONENTS 
filesystem 
program_options 
system
)

# find_package(OpenCV REQUIRED)
# Set(OpenCV_DIR "/home/cindy/16833/opencv/OpenCV/build") # important find opencv version
find_package(OpenCV 3.2.0 REQUIRED)

# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Ceres REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES camera_model
CATKIN_DEPENDS roscpp std_msgs
#    DEPENDS system_lib
)

# Ceres
include_directories(${CERES_INCLUDE_DIRS})

# Boost
include_directories(${Boost_INCLUDE_DIRS})

include_directories(
include 
${OpenCV_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

add_library(camera_model
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

# target_link_libraries(Calibration ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
