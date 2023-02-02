# opt/ros/melodic/share

# cmake needs this line
cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

# Define project name
project(orb_depth_cam)

set(THREADS_HAVE_PTHREAD_ARG 1)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI

find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)
find_package(Threads REQUIRED)

set(OpenCV_DIR=/usr/include/opencv4/)
set(cv_bridge_DIR /home/vslam/catkin_ws/src/vision_opencv/cv_bridge/)
find_package(OpenCV 4.5.5 REQUIRED)

##ROS STUFF
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  std_msgs
)
###



# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "    OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES orb_depth_mono
  CATKIN_DEPENDS cv_bridge image_transport std_msgs
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


# Declare the executable target built from your sources
add_executable(${PROJECT_NAME} src/main.cpp src/ )


target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
# Link your application with OpenCV libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${CUDA_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
