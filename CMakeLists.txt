cmake_minimum_required(VERSION 3.8)
project(so101_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS2 dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(SCSERVO_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/SCServo/src")

set(SCSERVO_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/SCServo/src")

if(EXISTS "${SCSERVO_SDK_DIR}")
  file(GLOB SCSERVO_SOURCES
    ${SCSERVO_SDK_DIR}/*.cpp
    ${SCSERVO_SDK_DIR}/*.c
  )
  add_library(scservo STATIC ${SCSERVO_SOURCES})
  target_include_directories(scservo PUBLIC ${SCSERVO_SDK_DIR})
  set(SCSERVO_FOUND TRUE)
  message(STATUS "SCServo SDK found at ${SCSERVO_SDK_DIR}")
else()
  message(WARNING "SCServo SDK not found at ${SCSERVO_SDK_DIR}. Building with stub driver.")
  set(SCSERVO_FOUND FALSE)
endif()

# ── Shared library ────────────────────────────────────────────────────────────
add_library(
  ${PROJECT_NAME}
  SHARED
  src/so101_hardware_interface.cpp
  src/st_servo_driver.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(SCSERVO_FOUND)
  target_include_directories(${PROJECT_NAME} PRIVATE ${SCSERVO_SDK_DIR})
  target_link_libraries(${PROJECT_NAME} SCServo)       # Our own static target
  target_compile_definitions(${PROJECT_NAME} PRIVATE SCSERVO_SDK_FOUND)
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface so101_hardware_interface_plugin.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
