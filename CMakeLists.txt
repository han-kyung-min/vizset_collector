cmake_minimum_required(VERSION 3.0.2)
project(vizset_collector)

#add_compile_options(-std=c++11)
set (CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(VIZSET_PROJ_DIRECTORY ${PROJECT_SOURCE_DIR} )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(OpenMP REQUIRED)
find_package(OpenMP)

find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


file(GLOB_RECURSE SRC_FILES 
 src/*.cpp 
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${BULLET_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${VIZSET_PROJ_DIRECTORY}/include
)



# build ratrace lib 
add_library(vizset_collector SHARED ${SRC_FILES})
target_link_libraries(vizset_collector 
-dpthread
${Boost_LIBRARIES}
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${BULLET_LIBRARIES}
${SDL_LIBRARY}
${SDL_IMAGE_LIBRARIES}
)

set_target_properties( vizset_collector PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${VIZSET_PROJ_DIRECTORY}/lib 
)

add_executable(vizset_collector_run ${VIZSET_PROJ_DIRECTORY}/example/vizset_collector_run.cpp ${SRC_FILES} ) 
target_link_libraries( vizset_collector_run vizset_collector ${YAMLCPP_LIBRARIES}) 
set_target_properties( vizset_collector_run PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


#add_executable(ffp_vs_wfd ${FD_PROJ_DIRECTORY}/example/ffp_vs_wfd.cpp ${SRC_FILES} ) 
#target_link_libraries(ffp_vs_wfd ${OpenCV_LIBS}) 
#set_target_properties( ffp_vs_wfd PROPERTIES
#  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
