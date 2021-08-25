cmake_minimum_required(VERSION 3.0.2)
project(visualization_brubotics)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    mrs_msgs
    geometry_msgs
    tf2_eigen
    eigen_conversions
    mrs_lib
    std_msgs
    trackers_brubotics
)

find_package(Eigen3 REQUIRED)


catkin_package(
  CATKIN_DEPENDS 
    mrs_msgs 
    mrs_lib
    geometry_msgs
    Eigen
    std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



add_library(visual2
  src/visual2.cpp
)

#This will create executables of the nodes
add_executable(visual src/visual.cpp)
add_executable(visual_test src/visual_test.cpp)

target_link_libraries(visual_test ${catkin_LIBRARIES} ${Boost_LIBRAIRIES} visual2)
# ## Declare a C++ library
# add_library(visualization_brubotics
#   src/visual.cpp
# )

#This will link executables to the appropriate libraries
target_link_libraries(visual ${catkin_LIBRARIES})

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)

if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(visualization_brubotics FILES ${SOURCES} ${HEADERS})
  add_tidy_target(visualization_brubotics
    FILES ${SOURCES}
    DEPENDS visualization_brubotics
  )
endif()