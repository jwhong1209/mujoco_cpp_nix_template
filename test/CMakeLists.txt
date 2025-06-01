cmake_minimum_required(VERSION 3.10)
project(test)

### Set CMake policies to suppress warnings
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)  # Use BoostConfig.cmake instead of FindBoost
endif()

# set(CMAKE_WARN_DEPRECATED FALSE)  # hide deprecated warning

# hpp-fcl/coal 관련 경고 메시지 숨기기
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-cpp")

### Set C/C++ Standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

### Set repo's absolute directory
# add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

### Find required packages
find_package(PkgConfig REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(pinocchio REQUIRED) # after find urdfdom_headers
find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)

### Set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic) # when you can't find build error, try this
  add_compile_options(-Wpedantic)
endif()

add_executable(${PROJECT_NAME}
  casadi-aba.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
  urdfdom_headers::urdfdom_headers
  pinocchio::pinocchio
  casadi
)