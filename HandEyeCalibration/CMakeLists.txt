cmake_minimum_required(VERSION 3.10)
project(TrajectoryAlignment)

set(CMAKE_CXX_STANDARD 17)

# Set the Eigen3 include directory (modify the path if necessary)
find_package (Eigen3 REQUIRED)
find_package (OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(trajectory_alignment trajectory_alignment.cpp)
target_link_libraries (trajectory_alignment Eigen3::Eigen ${OpenCV_LIBS})
