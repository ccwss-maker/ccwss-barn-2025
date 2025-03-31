cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planning LANGUAGES CXX)

# 设置C++编译选项（仅适用于C++编译器）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

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

# 设置catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_planning
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs tf message_filters astar_msgs
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 添加库路径
link_directories(/usr/local/lib)
set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# 创建主可执行文件（不包含CUDA源文件）
add_executable(trajectory_planning_node
  src/main.cpp
  src/Matrix.cpp
  src/TFSubscriberNode.cpp
  # src/Mapping.cpp
  src/A_Star_Planning_Node.cpp
)

# 链接所有需要的库
target_link_libraries(trajectory_planning_node
  ${catkin_LIBRARIES}
  yaml-cpp
)
