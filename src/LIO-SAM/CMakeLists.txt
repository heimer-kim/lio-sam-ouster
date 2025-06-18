cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
# C++ 표준을 14로 설정 (Noetic + GTSAM 4.x 호환을 위해)
set(CMAKE_CXX_FLAGS "-std=c++14")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

## ─── PCL, OpenCV, GTSAM, Boost 찾기 ─────────────────────────────────────##
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)          # PCL 패키지 찾기
find_package(OpenCV REQUIRED QUIET)       # OpenCV 찾기
find_package(GTSAM REQUIRED QUIET)        # GTSAM 찾기
find_package(Boost REQUIRED COMPONENTS timer serialization)  # Boost.timer + Boost.serialization

## ─── PCL 전역 매크로 정의 ─────────────────────────────────────────────##  
## PCL_NO_PRECOMPILE : PCL이 미리 컴파일된(core) 포인트 타입만 사용하도록 강제  
## PCL_ONLY_CORE_POINT_TYPES : PCL 템플릿 인스턴스화를 최소화  
add_definitions(-DPCL_NO_PRECOMPILE -DPCL_ONLY_CORE_POINT_TYPES)

## ─── 메시지/서비스 파일 등록 ───────────────────────────────────────##
add_message_files(
  DIRECTORY msg
  FILES
    cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    save_map.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    nav_msgs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM Boost

  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    message_generation
    visualization_msgs
)

## ─── 인클루드 디렉토리 ───────────────────────────────────────────────##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

## ─── 라이브러리 디렉토리 (링크 디렉토리) ───────────────────────────##
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

####################
## Build targets ##
####################

# 1. Range Image Projection 노드
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(
  ${PROJECT_NAME}_imageProjection
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)

# 2. Feature Association 노드
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(
  ${PROJECT_NAME}_featureExtraction
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)

# 3. Mapping Optimization 노드
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(
  ${PROJECT_NAME}_mapOptmization
  Boost::timer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

# 4. IMU Preintegration 노드
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
add_dependencies(${PROJECT_NAME}_imuPreintegration 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(
  ${PROJECT_NAME}_imuPreintegration
  Boost::timer
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  gtsam
)

#timesync node
add_executable(lidar_imu_sync src/lidar_imu_sync.cpp)
target_link_libraries(lidar_imu_sync ${catkin_LIBRARIES})

