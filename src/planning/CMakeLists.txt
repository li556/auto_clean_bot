cmake_minimum_required(VERSION 3.0.2)
project(artificial_potential_field_planner)

add_compile_options(-std=c++14)

# 找到需要的 ROS 包，加入 bot_msg
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  base_planner
  dynamic_reconfigure
  bot_msg
)

# 生成动态配置
generate_dynamic_reconfigure_options(
  cfg/config.cfg
)

# 包含头文件目录
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
) 

# 定义 catkin_package，并包含 bot_msg 依赖
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES artificial_potential_field_planner 
  CATKIN_DEPENDS base_planner bot_msg
)

# 定义源码文件
set(SRC_FILES
        src/apf_node.cpp
        src/apf/apf_planner.cpp
)

# 定义可执行文件，并设置依赖项
add_executable(${PROJECT_NAME}_node ${SRC_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# 定义库，并设置依赖项
add_library(${PROJECT_NAME} ${SRC_FILES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test
         test/test_apfp.cpp
         test/artificial_potantial_planner/test_apf_planner2D.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
