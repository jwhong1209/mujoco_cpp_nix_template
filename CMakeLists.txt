### Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.16)
project(mujoco_cpp_nix_template)

### Set CMake policies to suppress warnings
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)  # Use BoostConfig.cmake instead of FindBoost
endif()

set(CMAKE_WARN_DEPRECATED FALSE)  # hide deprecated warning

### Set C/C++ Standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

### Set repo's absolute directory
add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

### Find required packages
find_package(PkgConfig REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(pinocchio REQUIRED) # after find urdfdom_headers
find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)

### Set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

### Find source and header files to create executable simulation file
file(GLOB_RECURSE SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/common/src/*.cpp
)

### Create executable and link libraries
set(EXECUTABLE_NAME simulation)
add_executable(${EXECUTABLE_NAME} ${SOURCES})

target_include_directories(${EXECUTABLE_NAME}
PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/controller
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/math
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/motion
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/robot
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include/utilities
)

target_link_libraries(${EXECUTABLE_NAME}
PRIVATE
  mujoco
  glfw
  pthread
  urdfdom_headers::urdfdom_headers
  pinocchio::pinocchio
  casadi
)