cmake_minimum_required(VERSION 2.8.3)
project(lego_loam)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_CXX_STANDARD 14)

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

#        roslib
        )

# find_package (Eigen3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

include(cmake/YAML.cmake)


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS cloud_msgs
        DEPENDS PCL
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
)

link_directories(
        include
        ${OpenCV_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
        ${GTSAM_LIBRARY_DIRS}
)
link_libraries(yaml-cpp)

add_executable(imageProjection src/image_projection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation src/feature_association.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
    target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mapOptmization
        src/map_optmization.cpp
        include/scan_context/Scancontext.cpp
        )
target_link_libraries(mapOptmization
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        gtsam
        # Eigen3::Eigen
        )

add_executable(transformFusion src/transform_fusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(mapLoader src/map_loader.cpp)
#target_link_libraries(mapLoader ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


