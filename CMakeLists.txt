cmake_minimum_required(VERSION 2.8.3)
project(vans_copter)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  image_transport)

find_package(gazebo REQUIRED)

find_package(OpenCV REQUIRED)

find_package(darknet_ros_msgs)

add_message_files(
    FILES 
        XYZ.msg
        RPY.msg
        rpm.msg
        Command.msg
        PlanningPoint.msg
        Num.msg
        CamParam.msg
        CamCtrl.msg
        VecForward.msg
)

generate_messages(
    DEPENDENCIES 
        std_msgs
)


catkin_package(
    INCLUDE_DIRS 
        include
    LIBRARIES 
        vans_copter
    CATKIN_DEPENDS 
        gazebo_ros 
        roscpp 
        rospy
        std_msgs
        sensor_msgs
        message_runtime
        cv_bridge
        image_transport
    DEPENDS 
        system_lib
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS})
include_directories(
    include 
    ${Boost_INCLUDE_DIR} 
    ${catkin_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(
    customLIB
    include/vans_copter/customPID.h
    src/lib/customPID.cpp
    include/vans_copter/DualFilter.h
    src/lib/DualFilter.cpp
)
add_dependencies(
    customLIB
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(customLIB    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable        (MyCtrl src/main/MyCtrl.cpp)
target_link_libraries (MyCtrl ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (MyCtrl ${PROJECT_NAME}_generate_messages_cpp)

add_executable        (TargetCtrl src/main/TargetCtrl.cpp)
target_link_libraries (TargetCtrl ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (TargetCtrl ${PROJECT_NAME}_generate_messages_cpp)


add_executable        (planningPoint src/main/planningPoint.cpp)
target_link_libraries (planningPoint ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (planningPoint ${PROJECT_NAME}_generate_messages_cpp)


add_library           (motor_plgn SHARED src/main/motor_plgn.cpp)
target_link_libraries (motor_plgn ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (motor_plgn ${PROJECT_NAME}_generate_messages_cpp)


add_library           (RPY_test SHARED src/main/RPY_test.cpp)
target_link_libraries (RPY_test customLIB ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (RPY_test ${PROJECT_NAME}_generate_messages_cpp)


add_library           (RPY_Cam SHARED src/main/RPY_Cam.cpp)
target_link_libraries (RPY_Cam customLIB ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (RPY_Cam ${PROJECT_NAME}_generate_messages_cpp)


add_library           (XYZ_test SHARED src/main/XYZ_test.cpp)
target_link_libraries (XYZ_test customLIB ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (XYZ_test ${PROJECT_NAME}_generate_messages_cpp)


add_library           (Com2 SHARED src/main/Com2.cpp)
target_link_libraries (Com2 customLIB ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (Com2 ${PROJECT_NAME}_generate_messages_cpp)


add_library           (camStablizer SHARED src/main/camStablizer.cpp)
target_link_libraries (camStablizer customLIB ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (camStablizer ${PROJECT_NAME}_generate_messages_cpp)


add_library           (KeycmdProcess SHARED src/main/KeycmdProcess.cpp)
target_link_libraries (KeycmdProcess customLIB ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (KeycmdProcess ${PROJECT_NAME}_generate_messages_cpp)


add_executable        (BBoxInfo src/main/BBoxInfo.cpp)
target_link_libraries (BBoxInfo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (BBoxInfo ${PROJECT_NAME}_generate_messages_cpp)


add_executable        (PosSaver src/main/PosSaver.cpp)
target_link_libraries (PosSaver ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies      (PosSaver ${PROJECT_NAME}_generate_messages_cpp)
