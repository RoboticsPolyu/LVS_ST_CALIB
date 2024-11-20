cmake_minimum_required(VERSION 3.10)
project(LVS_ST_CALIB)

option(BUILD_SHARED_LIBS "Build shared gtsam library, instead of static" ON)

set(wio_factor_version 1.0)
set(wio_factor_soversion 1.0)

set(DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

set(ENV{CC} "/usr/local/opt/llvm/bin/clang-8")
set(ENV{CXX} "/usr/local/opt/llvm/bin/clang++")
set(ENV{LDFLAGS} "-L/usr/local/opt/llvm/lib")
set(ENV{CPPFLAGS} "-I/usr/local/opt/llvm/include")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wno-reorder" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG -Wno-reorder -O2" CACHE STRING "" FORCE)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "/usr/local/cmake/lib/cmake")

#############################################################################################
# third party libs
# eigen
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("opencv include :" ${OpenCV_INCLUDE_DIRS})

find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)
# # tbb
# find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)

# # gtsam
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# # yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# # ros
# find_package(catkin REQUIRED COMPONENTS roscpp)
# catkin_package(
#   CATKIN_DEPENDS roscpp
# )
# include_directories(
#  ${catkin_INCLUDE_DIRS}
# )

# include
include_directories(include)

#############################################################################################
# Wheel IMU Calibration
set(WIO_SOURCE
    src/wio_factor.cpp
    src/wio_integration.cpp)

IF(UNIX)
ELSEIF(APPLE)    
ENDIF()

message(STATUS "Building GTSAM - shared: ${BUILD_SHARED_LIBS}")

###########################################################################################
# ABB EGM LIBRARIE
message("PROTOBUF INCLUDES: " ${PROTOBUF_INCLUDE_DIRS})
set(ABB_EGM_SOURCE
    src/egm/egm_control.cpp
    src/egm/handeye_path_control.cpp
    include/egm/egm.pb.cc)
add_library(abb_egm_lib ${ABB_EGM_SOURCE})
target_include_directories(abb_egm_lib PUBLIC
    ${GTSAM_INCLUDE_DIR} 
    ${EIGEN_INCLUDE_DIR} 
    ${PROTOBUF_INCLUDE_DIRS})
target_link_libraries(abb_egm_lib PUBLIC 
    yaml-cpp   
    Boost::regex
    Boost::system
    Boost::thread
    gtsam 
    gtsam_unstable
    ${PROTOBUF_LIBRARIES}
    Threads::Threads)
set_target_properties(abb_egm_lib PROPERTIES
        OUTPUT_NAME         abb_egm_lib
        CLEAN_DIRECT_OUTPUT 1
        VERSION             1.0
        SOVERSION           1.0)

#############################################################################################
# Laser Camera Sensor Calibration 
set(XYF_CALIBRATION_SOURCE
    src/laser_camera_cal.cpp
    src/laser_camera_cal.cpp
    src/hand_eye_calibration.cpp
    src/td_factor.cpp)

add_library(laser_camera_cal ${XYF_CALIBRATION_SOURCE})
target_include_directories(laser_camera_cal PUBLIC ${GTSAM_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
target_link_libraries(laser_camera_cal PUBLIC ${OpenCV_LIBS}  yaml-cpp)
set_target_properties(laser_camera_cal PROPERTIES
    OUTPUT_NAME         laser_camera_cal
    CLEAN_DIRECT_OUTPUT 1
    VERSION             1.0
    SOVERSION           1.0)
 
#############################################################################################
# Daheng Camera
set(SENSORS_SOURCE
    src/daheng_camera_wrapper.cpp)
add_library(sensors_wrapper_lib ${SENSORS_SOURCE})
target_include_directories(sensors_wrapper_lib PUBLIC })
target_link_libraries(sensors_wrapper_lib PUBLIC gxiapi pthread ${OpenCV_LIBS})

set_target_properties(sensors_wrapper_lib PROPERTIES
    OUTPUT_NAME         sensors_wrapper_lib
    CLEAN_DIRECT_OUTPUT 1
    VERSION             ${abb_wrapper_version}
    SOVERSION           ${abb_wrapper_soversion}
)

############################################################################################
# SLAM
set(SENSORS_SOURCE
    src/charuco_create_detect.cpp)
add_library(slam_lib ${SENSORS_SOURCE})
target_include_directories(slam_lib PUBLIC })
target_link_libraries(slam_lib PUBLIC pthread ${OpenCV_LIBS})

set_target_properties(slam_lib PROPERTIES
    OUTPUT_NAME         slam_lib
    CLEAN_DIRECT_OUTPUT 1
    VERSION             1.0
    SOVERSION           1.0
)

# APP


######################################### Calibration ###################################
add_executable(laser_camera_calib app/laser_camera_calib.cpp)
target_link_libraries(laser_camera_calib laser_camera_cal gtsam gtsam_unstable)

add_executable(camera_calibration app/camera_calibration.cpp)
target_link_libraries(camera_calibration laser_camera_cal gtsam gtsam_unstable)

add_executable(fillet_weld_detector app/fillet_weld_detector.cpp)
target_link_libraries(fillet_weld_detector laser_camera_cal gtsam gtsam_unstable)

add_executable(laser_line_detector app/laser_line_detector.cpp)
target_link_libraries(laser_line_detector laser_camera_cal gtsam gtsam_unstable)

add_executable(daheng_camera app/test_daheng_camera.cpp)
target_link_libraries(daheng_camera sensors_wrapper_lib)

add_executable(test_charuco app/test_charuco_cv.cpp)
target_link_libraries(test_charuco slam_lib)

######################################### Handeye ########################################
add_executable(plot_tracjectory app/plot_tracjectory.cpp)
target_link_libraries(plot_tracjectory ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(plot_end_effector app/plot_end_effector.cpp)
target_link_libraries(plot_end_effector ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(test_handeye app/test_handeye.cpp)
target_link_libraries(test_handeye ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(sensor_evaluation app/sensor_evaluation.cpp)
target_link_libraries(sensor_evaluation ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(align_LVS_EGM app/align_LVS_EGM.cpp)
target_link_libraries(align_LVS_EGM ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(align_LVS_EGM1 app/align_LVS_EGM1.cpp)
target_link_libraries(align_LVS_EGM1 ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(align_LVS_EGM2 app/align_LVS_EGM2.cpp)
target_link_libraries(align_LVS_EGM2 ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(sim_lvrws app/sim_lvrws.cpp)
target_link_libraries(sim_lvrws ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(calib_td_sim app/calib_td_sim.cpp)
target_link_libraries(calib_td_sim ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

add_executable(td_Factor_test app/td_factor_test.cpp)
target_link_libraries(td_Factor_test gtsam gtsam_unstable laser_camera_cal)

add_executable(timestamp_process app/timestamp_process.cpp)
target_link_libraries(timestamp_process gtsam gtsam_unstable laser_camera_cal)

add_executable(spatiotemporal_calibration_lvs app/spatiotemporal_calibration_lvs.cpp)
target_link_libraries(spatiotemporal_calibration_lvs ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)

######################################### EGM ############################################
add_executable(abb_egm_example app/abb_egm_example.cpp)
target_link_libraries(abb_egm_example abb_egm_lib)

add_executable(plot_egm app/plot_egm.cpp)
target_link_libraries(plot_egm ${Pangolin_LIBRARIES} gtsam gtsam_unstable laser_camera_cal)
#############################################################################################
