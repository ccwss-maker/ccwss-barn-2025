cmake_minimum_required(VERSION 3.0.2)
project(trajectory_tracking LANGUAGES CXX)

# 设置C++编译选项（仅适用于C++编译器）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
# set (CMAKE_CXX_STANDARD 11)
# 查找需要的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_filters
  astar_msgs
)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OsqpEigen REQUIRED)
find_package(OpenCV REQUIRED)
# 设置catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_tracking
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs tf message_filters astar_msgs
)

# 包含目录
include_directories(
  ../navigation/include
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
# 添加库路径
link_directories(/usr/local/lib)
set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# 创建主可执行文件（不包含CUDA源文件）
add_executable(trajectory_tracking_node
  src/main.cpp
  ../navigation/src/TFSubscriberNode.cpp
  ../navigation/src/Matrix.cpp
  src/MSG_Process.cpp
  src/Pure_Pursuit.cpp
)

# 链接所有需要的库
target_link_libraries(trajectory_tracking_node
  ${catkin_LIBRARIES}
  yaml-cpp
  OsqpEigen::OsqpEigen
  ${OpenCV_LIBS}
)