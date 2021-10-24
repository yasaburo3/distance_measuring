set(CMAKE_BUILD_TYPE "Debug")

cmake_minimum_required(VERSION 3.0.2)
project(MotionSolver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package( PCL REQUIRED COMPONENTS common io visualization filters)

find_package(OpenCV REQUIRED)

find_package(realsense2 REQUIRED)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library( slambase ./src/slamBase.cpp)
target_link_libraries(slambase
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}  )



# add_executable(imageconverter ./src/imageConverter.cpp)
# target_link_libraries( imageconverter
#     ${OpenCV_LIBRARIES}
#     ${catkin_LIBRARIES}
#     )

add_executable( motionsolution ./src/motionSolution.cpp )
target_link_libraries(motionsolution
    slambase
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${realsense2_LIBRARY}  )

add_executable( keyboard_receiver ./src/keyboard_receiver.cpp )
target_link_libraries( keyboard_receiver
    ${catkin_LIBRARIES}  )