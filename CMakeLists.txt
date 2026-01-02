cmake_minimum_required(VERSION 3.0.2)
project(weaving_inspection)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  roslib
  message_generation
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
)
find_package(tkDNN)

if(tkDNN_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS} ${tkDNN_INCLUDE_DIRS} /usr/local/include/tkDNN)
endif()

# ## Automatically identify the architecture and set the library path accordingly.
EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Architecture: ${ARCHITECTURE}")

if(${ARCHITECTURE} STREQUAL "x86_64")
  set(MVS_LIBRARY "/opt/MVS/lib/64/libMvCameraControl.so")
elseif(${ARCHITECTURE} STREQUAL "aarch64")
  set(MVS_LIBRARY "/opt/MVS/lib/aarch64/libMvCameraControl.so")
endif()

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
add_message_files(
  FILES
  StoppingDefectImages.msg
)

# # Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPulseDistance.srv
  GetCurrentPulses.srv
  GetSimulationImage.srv
  LogsAPI.srv
  InsertDataLog.srv
  GetRollLog.srv
)

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs # Or other packages containing msgs
  sensor_msgs
)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/DynReconf1.cfg
# cfg/DynReconf2.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime
  cv_bridge
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/dalsa/GigeV/examples/common
  /usr/dalsa/GigeV/include
  /opt/genicam_v3_0/library/CPP/include
  /opt/MVS/include
  /usr/local/cuda-11.7/targets/x86_64-linux/include
)

add_executable(camera_simulation
  src/cameraHighSpeedSimulator.cpp)

add_executable(weaving_main
  src/weavingInspection.cpp
  src/main.cpp
  src/baseType.cpp
  src/job.cpp
  src/roll.cpp
  src/paramStore.cpp
)

add_executable(real_world_map_node
  src/realWorldMapNode.cpp
  src/defect.cpp
  src/body.cpp
  src/plcComm.cpp
  src/baseType.cpp
  src/worldMapDrawer.cpp
  src/cuttingManager.cpp
  src/buzzerControl.cpp
  src/cuttingMachinePLCComm.cpp
)

add_executable(brightness_adjuster
  src/brightnessAdjuster.cpp
  src/notifier.cpp
)

add_executable(reporting_node
  src/reportingNode.cpp
  src/body.cpp
  src/job.cpp
  src/defect.cpp
  src/roll.cpp
  src/baseType.cpp
)

add_executable(camera_node
  src/cameraNode.cpp
  src/jobAndRecipeManager.cpp
  src/defectsProcessor.cpp
  src/networkManager.cpp
  src/cameraManager.cpp
  src/notifier.cpp
  src/cameraIndicator.cpp
  src/paramStore.cpp
)

add_dependencies(camera_simulation ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(weaving_main ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(real_world_map_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(brightness_adjuster ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(reporting_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(camera_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(camera_simulation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
  cameraSimulation
  robroCameraAbstractClass)

target_link_libraries(weaving_main
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  robroMySQL
  robroImageSaver
)

target_link_libraries(real_world_map_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  robroImageSaver
  robroModbus
  robroMySQL
)

target_link_libraries(brightness_adjuster
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
  robroMySQL
)

target_link_libraries(reporting_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  robroMySQL
)

if(tkDNN_FOUND)
  target_link_libraries(camera_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenCV_LIBS}
    ${MVS_LIBRARY}
    GevApi
    kernels # only needed when you use target_compile_definitions
    nvinfer # only needed when you use target_compile_definitions
    robroTkDNN
    dalsaCamera
    robroYOLOV8
    hikRobotCamera
    robroImageSaver
    cameraSimulation
    robroCameraAbstractClass
    robroInferenceAbstractClass
    robroMySQL
    robroFloatingLicense
  )
  target_compile_definitions(camera_node PRIVATE ROBRO_TKDNN_FOUND=True)

# Variable to idnetify the compiling architecture support the tkDNN_FOUND or not.
else()
  target_link_libraries(camera_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenCV_LIBS}
    ${MVS_LIBRARY}
    GevApi
    dalsaCamera
    hikRobotCamera
    robroImageSaver
    robroYOLOV8
    cameraSimulation
    robroCameraAbstractClass
    robroInferenceAbstractClass
    robroMySQL
    robroFloatingLicense
  )
endif()

# # Declare a C++ library
# add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/files_inspection.cpp
# )

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/files_inspection_node.cpp)

# # Rename C++ executable without prefix
# # The above recommended prefix causes long target names, the following renames the
# # target back to the shorter version for ease of user use
# # e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
# ${catkin_LIBRARIES}
# )

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  gui/webGUIServer.py
  src/apiServer.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# # Mark executables for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS camera_node
  weaving_main
  brightness_adjuster
  reporting_node
  camera_simulation
  real_world_map_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
install(DIRECTORY
  db
  gui
  images
  launch
  nn
  license
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_files_inspection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
