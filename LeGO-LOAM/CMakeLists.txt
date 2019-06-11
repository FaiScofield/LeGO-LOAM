cmake_minimum_required(VERSION 2.8.3)
project(lego_loam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs

  message_generation
)

add_service_files(
    FILES
    saveMap.srv
)

generate_messages()

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
#find_package(fast REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs
  DEPENDS PCL OpenCV
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
#    ${fast_INCLUDE_DIRS}
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
#    ${fast_LIBRARIES}
)

add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation src/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
add_dependencies(mapOptmization lego_loam_gencpp)

add_executable(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(saveTrajectory src/saveTrajectory.cpp)
target_link_libraries(saveTrajectory ${catkin_LIBRARIES})

add_executable(kittiVelodyneRawData2Bag src/kittiVelodyneRawData2Bag.cpp)
target_link_libraries(kittiVelodyneRawData2Bag ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(simpleVO src/simpleVO.cpp)
target_link_libraries(simpleVO ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(testmatch src/testmatch.cpp)
#target_link_libraries(testmatch ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(saveMapService src/saveMapService.cpp)
#target_link_libraries(saveMapService ${catkin_LIBRARIES})
#add_dependencies(saveMapService lego_loam_gencpp)
