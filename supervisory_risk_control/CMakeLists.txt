cmake_minimum_required(VERSION 3.10)
project(supervisory_risk_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_CXX_FLAGS "-Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -DCMAKE_BUILD_TYPE=Debug -g -O0 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -O3 ${CMAKE_CXX_FLAGS}")

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             roslib 
             mavros_msgs
             std_msgs
             supervisory_risk_control_msgs
             )

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  mavros_msgs
  std_msgs
  roslib
  supervisory_risk_control_msgs
  )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CMAKE_SOURCE_DIR}/include/smile/libsmile.a)
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.launch")
