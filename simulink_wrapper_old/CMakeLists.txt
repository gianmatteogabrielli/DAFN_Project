cmake_minimum_required(VERSION 3.8)
project(simulink_wrapper_cpp)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

# Specifica il percorso completo della directory mylibs
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/mylibs)

# publisher
add_executable(publisher src/publisher.cpp)
target_include_directories(publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  publisher
  rclcpp
  std_msgs)
#target_link_libraries(publisher digitalfilter) # Assicurati che il nome della libreria sia corretto

# subscriber
add_executable(subscriber src/subscriber.cpp)
target_include_directories(subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  subscriber
  rclcpp
  std_msgs)
#target_link_libraries(subscriber digitalfilter) # Assicurati che il nome della libreria sia corretto

# modelNode
add_executable(modelnode src/modelnode.cpp src/simulink_classes.cpp)
target_include_directories(modelnode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  modelnode
  rclcpp
  std_msgs)
  target_link_libraries(modelnode dl) # Assicurati che il nome della libreria sia corretto

# Install executables
install(TARGETS publisher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS subscriber
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS modelnode
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
