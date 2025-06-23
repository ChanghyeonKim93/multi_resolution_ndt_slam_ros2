cmake_minimum_required(VERSION 3.16)
project(multi_resolution_ndt_slam)
message("   -- current build: " ${PROJECT_NAME}_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

set(SRC_UTILITY ${SRC_UTILITY}
  utility/time_monitor.cc
)
set(SRC ${SRC}
  ndt_slam.cc
)

add_library(${PROJECT_NAME}_core SHARED ${SRC} ${SRC_UTILITY})
target_link_libraries(${PROJECT_NAME}_core ${Eigen3_LIBRARIES} ${CERES_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_core
        DESTINATION lib)
install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}
  DESTINATION include # /usr/local/include
  FILES_MATCHING
  PATTERN "*.h")
