cmake_minimum_required(VERSION 2.8.3)
project(rosddsbridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(FASTRTPS_INC_DIR ~/Fast-DDS/install/fastrtps/include)
set(FASTRTPS_LINK_DIR ~/Fast-DDS/install/fastrtps/lib)
set(FASTCDR_INC_DIR ~/Fast-DDS/install/fastcdr/include)
set(FASTCDR_LINK_DIR ~/Fast-DDS/install/fastcdr/lib)
set(FOONATHAN_INC_DIR ~/Fast-DDS/install/foonathan_memory_vendor/include)
set(FOONATHAN_LINK_DIR ~/Fast-DDS/install/foonathan_memory_vendor/lib)
set(OpenCV_DIR /usr/share/OpenCV)

include_directories(${FASTRTPS_INC_DIR}) 
link_directories(${FASTRTPS_LINK_DIR})

include_directories(${FASTCDR_INC_DIR}) 
link_directories(${FASTCDR_LINK_DIR})

include_directories(${FOONATHAN_INC_DIR}) 
link_directories(${FOONATHAN_LINK_DIR})

find_package(catkin REQUIRED COMPONENTS tf message_generation roscpp rospy std_msgs sensor_msgs tf2_msgs nav_msgs octomap_msgs geometry_msgs cv_bridge
  image_transport 
  pcl_ros
  pcl_conversions 
  OpenCV
  eigen_conversions
  cv_bridge
  sensor_msgs 
  nav_msgs
  message_generation
  )

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT foonathan_memory_FOUND)
    find_package(foonathan_memory REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL REQUIRED)

file(GLOB DDS_EXAMPLE_SOURCES_CXX "include/*.cxx")
file(GLOB DDS_EXAMPLE_SOURCES_CPP "*.cpp")
#file(GLOB XML_CONFIG_FILES "*.xml")
#message(STATUS "XML Files: " ${XML_CONFIG_FILES})
configure_file("config/HelloWorldSubscriber.xml" "HelloWorldSubscriber.xml" COPYONLY)
configure_file("config/HelloWorldPublisher.xml" "HelloWorldPublisher.xml" COPYONLY)
configure_file("config/dh2048.pem" "dh2048.pem" COPYONLY)
configure_file("config/server.pem" "server.pem" COPYONLY)
configure_file("config/ca.pem" "ca.pem" COPYONLY)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PclWithOdom.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime
  INCLUDE_DIRS include
#  LIBRARIES dds_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
	${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs nav_msgs
  DEPENDS EIGEN3 PCL  
  INCLUDE_DIRS include
)

add_library(DDSPublisher 
	include/DDSPublisher.h
	src/DDSPublisher.cpp
)

#add_library(DDSSubscriber 
#	include/DDSSubscriber.h
#	src/DDSSubscriber.cpp
#)

add_dependencies(DDSPublisher  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(DDSSubscriber  ${${PROJECT_NAME}_EXPORTED_TARGETS} #{catkin_EXPORTED_TARGETS})

add_executable(ros2dds 
  ros2dds.cpp 
  src/DDSPublisher.cpp
  ${DDS_EXAMPLE_SOURCES_CXX} 
)

add_executable(dds2ros 
  dds2ros.cpp
  ${DDS_EXAMPLE_SOURCES_CXX} 
)

add_executable(vel_ros2dds 
  vel_ros2dds.cpp 
  src/DDSPublisher.cpp
  ${DDS_EXAMPLE_SOURCES_CXX} 
)

add_executable(vel_dds2ros 
  vel_dds2ros.cpp
  ${DDS_EXAMPLE_SOURCES_CXX} 
)

target_link_libraries(ros2dds  DDSPublisher  fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)
target_link_libraries( dds2ros   fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)

target_link_libraries(vel_ros2dds  DDSPublisher  fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)
target_link_libraries( vel_dds2ros   fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)

add_executable(pubImage 
  src/pubImage.cpp
)
target_link_libraries( pubImage
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(tfBr
  src/tfBr.cpp
)
target_link_libraries( tfBr
   ${catkin_LIBRARIES}
)

add_executable(readCamera
  src/readCamera.cpp
)
target_link_libraries( readCamera
   ${catkin_LIBRARIES}
)

add_executable(robotCmdSub 
  src/robotCmdSub.cpp 
  ${DDS_EXAMPLE_SOURCES_CXX} 
)

add_executable(pcCmdPub 
  src/pcCmdPub.cpp
  ${DDS_EXAMPLE_SOURCES_CXX} 
)
target_link_libraries(robotCmdSub  fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)
target_link_libraries( pcCmdPub   fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)

add_executable(robotJoySub 
  src/robotJoySub.cpp 
  ${DDS_EXAMPLE_SOURCES_CXX} 
)

add_executable(pcJoyPub 
  src/pcJoyPub.cpp
  ${DDS_EXAMPLE_SOURCES_CXX} 
)
target_link_libraries(robotJoySub  fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)
target_link_libraries( pcJoyPub   fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)

add_executable(robotGoalSub 
  src/robotGoalSub.cpp 
  ${DDS_EXAMPLE_SOURCES_CXX} 
)

add_executable(pcGoalPub 
  src/pcGoalPub.cpp
  ${DDS_EXAMPLE_SOURCES_CXX} 
)
target_link_libraries(robotGoalSub  fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)
target_link_libraries( pcGoalPub   fastrtps fastcdr foonathan_memory
  ${catkin_LIBRARIES}
)

add_executable(floam_laser_mapping_node src/laserMappingNode.cpp src/laserMappingClass.cpp src/lidar.cpp)
target_link_libraries(floam_laser_mapping_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(robotMapPub src/robotMapPub.cpp src/DDSPublisher.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries(robotMapPub  DDSPublisher  fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES})

add_executable(robotOdomPub src/robotOdomPub.cpp src/DDSPublisher.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries(robotOdomPub  DDSPublisher  fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES})

add_executable(robotVelodynePub src/robotVelodynePub.cpp src/DDSPublisher.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries(robotVelodynePub  DDSPublisher  fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES})

add_executable(pcMapSub src/pcMapSub.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( pcMapSub   fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )

add_executable(pcOdomSub src/pcOdomSub.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( pcOdomSub   fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )

add_executable(pcVelodyneSub src/pcVelodyneSub.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( pcVelodyneSub   fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )

add_executable(robotVelodyneWithOdometry src/robotVelodyneWithOdometry.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( robotVelodyneWithOdometry   DDSPublisher fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )

add_executable(pcVelodyneWithOdometry src/pcVelodyneWithOdometry.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( pcVelodyneWithOdometry   fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )
add_dependencies(pcVelodyneWithOdometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(visualizationDDS src/visualizationDDS.cpp)
target_link_libraries( visualizationDDS ${catkin_LIBRARIES} )
add_dependencies(visualizationDDS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robotMeasurePub src/robotMeasurePub.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( robotMeasurePub   DDSPublisher fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )
##add_dependencies(robotMeasurePub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pcMeasureSub src/pcMeasureSub.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( pcMeasureSub   fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )
add_dependencies(pcMeasureSub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robotImagePub src/robotImagePub.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( robotImagePub   DDSPublisher fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )
##add_dependencies(robotMeasurePub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pcImageSub src/pcImageSub.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( pcImageSub   fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )
add_dependencies(pcImageSub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## second
add_executable(robotVelodyneWithOdometry2 src/robotVelodyneWithOdometry2.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( robotVelodyneWithOdometry2   DDSPublisher fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )

add_executable(pcVelodyneWithOdometry2 src/pcVelodyneWithOdometry2.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries( pcVelodyneWithOdometry2   fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES} )
add_dependencies(pcVelodyneWithOdometry2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(udpComm src/udpComm.cpp)
target_link_libraries( udpComm ${catkin_LIBRARIES} )
add_dependencies(udpComm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(udpComm2 src/udpComm2.cpp)
target_link_libraries( udpComm2 ${catkin_LIBRARIES} )
add_dependencies(udpComm2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robotState src/robotState.cpp)
target_link_libraries( robotState ${catkin_LIBRARIES} )
add_dependencies(robotState ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robotMapPubIn src/robotMapPubIn.cpp src/DDSPublisher.cpp ${DDS_EXAMPLE_SOURCES_CXX} )
target_link_libraries(robotMapPubIn   fastrtps fastcdr foonathan_memory ${catkin_LIBRARIES})

add_executable(pcl2mat src/pcl2mat.cpp)
target_link_libraries( pcl2mat ${catkin_LIBRARIES} )
add_dependencies(pcl2mat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

