cmake_minimum_required(VERSION 3.1.3)
project(moveit_task_constructor_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_task_constructor_core
	moveit_ros_planning_interface
	rosparam_shortcuts
)

catkin_package(
	CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cartesian src/cartesian.cpp)
target_link_libraries(cartesian ${catkin_LIBRARIES})

add_executable(modular src/modular.cpp)
target_link_libraries(modular ${catkin_LIBRARIES})

add_library(pick_place_task OBJECT src/pick_place_task.cpp)

add_executable(pick_place_demo src/pick_place_demo.cpp $<TARGET_OBJECTS:pick_place_task>)
target_link_libraries(pick_place_demo ${catkin_LIBRARIES})
#--- o2ac/1
add_executable(modular_pick_hold src/modular_pick_hold.cpp)
target_link_libraries(modular_pick_hold ${catkin_LIBRARIES})

add_executable(mtc_modules_server src/mtc_modules_server.cpp)
target_link_libraries(mtc_modules_server ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_lib src/pick_place_task.cpp)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME moveit_task_constructor_demo_pick_place)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#--- o2ac
add_library(o2ac_lib src/o2ac_pick_place_task.cpp)
set_target_properties(o2ac_lib PROPERTIES OUTPUT_NAME o2ac_demo_pick_place)
add_dependencies(o2ac_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(pick_place_task_for_action src/pick_place_task_for_action.cpp)
set_target_properties(pick_place_task_for_action PROPERTIES OUTPUT_NAME pick_place_planning)
add_dependencies(pick_place_task_for_action ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_demo src/moveit_task_constructor_demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_demo PROPERTIES OUTPUT_NAME moveit_task_constructor_demo)
add_dependencies(${PROJECT_NAME}_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# --- o2ac
add_executable(alternative_path_costs src/alternative_path_costs.cpp)
target_link_libraries(alternative_path_costs ${catkin_LIBRARIES})

add_executable(ik_clearance_cost src/ik_clearance_cost.cpp)
target_link_libraries(ik_clearance_cost ${catkin_LIBRARIES})

add_executable(pick_place_planning_action_server src/pick_place_planning_action_server.cpp)
target_link_libraries(pick_place_planning_action_server
  pick_place_task_for_action
  ${catkin_LIBRARIES}
)

install(TARGETS cartesian modular pick_place_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_subdirectory(test)
