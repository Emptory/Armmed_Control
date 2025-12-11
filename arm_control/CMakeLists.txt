cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Must set BOOST_MPL limits BEFORE any headers are included
add_definitions(-DBOOST_MPL_LIMIT_LIST_SIZE=30)
add_definitions(-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_definitions(-DPINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp controller_interface hardware_interface pluginlib ocs2_core ocs2_oc ocs2_ddp ocs2_mpc ocs2_pinocchio_interface
  DEPENDS Boost pinocchio
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# OCS2 Interface Library
add_library(${PROJECT_NAME}
  src/MobileManipulatorInterface.cpp
  src/MobileManipulatorPinocchioMapping.cpp
  src/MobileManipulatorPreComputation.cpp
  src/FactoryFunctions.cpp
  src/SineReferenceManager.cpp
  src/dynamics/DefaultManipulatorDynamics.cpp
  src/constraint/EndEffectorConstraint.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

# Controller Library
add_library(arm_controller_lib
  src/ArmController.cpp
)
target_link_libraries(arm_controller_lib
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} arm_controller_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES arm_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
