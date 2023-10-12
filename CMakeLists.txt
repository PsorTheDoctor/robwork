#
# Test CMake version
#
cmake_minimum_required(VERSION 3.10.0)

# The name of the project.
project(graspAnalysis)

add_compile_options(-std=c++14)

if(DEFINED ENV{RW_ROOT} AND DEFINED ENV{RWS_ROOT})
  file(TO_CMAKE_PATH "$ENV{RW_ROOT}" RW_ROOT)
  file(TO_CMAKE_PATH "$ENV{RWS_ROOT}" RWSTUDIO_ROOT)
else()
  set(RW_ROOT "${ROOT}/../../../RobWork")
  set(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
endif()

# We use the settings that robwork studio uses
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE relwithdebinfo)
endif()

# Include default settings for constructing a RobWork dependent project
set(RobWork_DIR ${RW_ROOT}/cmake) 
find_package(RobWork REQUIRED)
message("-- RobWork found: " ${RW_ROOT})

# Include default settings for constructing a RobWorkStudio dependent project
set(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
find_package(RobWorkStudio REQUIRED)
message("-- RobWorkStudio found: " ${RWSTUDIO_ROOT})

add_executable(reachability src/reachability.cpp)
add_executable(linear src/linear.cpp)
add_executable(parabolic src/parabolic.cpp)
add_executable(rrt src/rrt_connect.cpp)

target_link_libraries(reachability ${ROBWORK_LIBRARIES})
target_link_libraries(linear ${ROBWORK_LIBRARIES})
target_link_libraries(parabolic ${ROBWORK_LIBRARIES})
target_link_libraries(rrt ${ROBWORK_LIBRARIES})
