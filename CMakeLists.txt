cmake_minimum_required(VERSION 3.0.2)
project(shin_spider)

add_compile_options(-std=c++17 -Wall -Wextra)
# Processing (Java) float semantics: every float operation must be rounded to
# float individually. Never allow fused multiply-add or fast-math.
add_compile_options(-ffp-contract=off -fno-fast-math)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES shin_spider_core
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ROS-free logic library (kinematics, motions, gamepad logic, firmware protocol, serial)
add_library(shin_spider_core
  src/processing.cpp
  src/Target.cpp
  src/Motion.cpp
  src/MotionList.cpp
  src/RobotLeg.cpp
  src/RobotHand.cpp
  src/Robot.cpp
  src/SpiderController.cpp
  src/leg_motion_commands.cpp
  src/hand_motion_commands.cpp
  src/gamepad_command.cpp
  src/JoyMapper.cpp
  src/FirmwareProtocol.cpp
  src/SerialPort.cpp
  src/MotionTestHook.cpp
)

add_executable(shin_spider_node src/shin_spider_node.cpp)
target_link_libraries(shin_spider_node shin_spider_core ${catkin_LIBRARIES})
add_dependencies(shin_spider_node ${catkin_EXPORTED_TARGETS})

# Dumps every motion in the golden-data text format (for diffing against the Java dump)
add_executable(shin_spider_dump tools/shin_spider_dump.cpp)
target_link_libraries(shin_spider_dump shin_spider_core)

install(TARGETS shin_spider_core shin_spider_node shin_spider_dump
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test_golden test/test_golden.cpp)
  target_compile_definitions(${PROJECT_NAME}_test_golden PRIVATE
    SHIN_SPIDER_GOLDEN_PATH="${PROJECT_SOURCE_DIR}/test/golden/golden.txt")
  target_link_libraries(${PROJECT_NAME}_test_golden shin_spider_core)

  catkin_add_gtest(${PROJECT_NAME}_test_kinematics test/test_kinematics.cpp)
  target_link_libraries(${PROJECT_NAME}_test_kinematics shin_spider_core)

  catkin_add_gtest(${PROJECT_NAME}_test_pty test/test_pty_integration.cpp)
  target_link_libraries(${PROJECT_NAME}_test_pty shin_spider_core ${catkin_LIBRARIES} util pthread)
endif()
