cmake_minimum_required(VERSION 3.15)
project(cuda_pcl_conversions)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        diagnostic_msgs
        ga_cmake
        pcl_conversions
        geometry_msgs
        message_generation
)

find_package(PCL REQUIRED QUIET)
find_package(GFlags REQUIRED QUIET)
find_package(CUDA REQUIRED QUIET)
find_package(thrust REQUIRED QUIET)

GA_CHECK_CUDA()

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} ${CUPOCH_NVCC_FLAGS})

add_definitions(${CUPOCH_DEFINITIONS})

message(STATUS "zs: ${CMAKE_PROJECT_NAME} CUDA_NVCC_FLAGS = " ${CUDA_NVCC_FLAGS})

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS
        pcl_conversions
        roscpp
        sensor_msgs
        ga_cmake
        std_msgs
        geometry_msgs
)

include_directories(
        ./include
        ${CUDA_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(${CUPOCH_LIBRARY_DIRS})
link_libraries(${CUPOCH_LIBRARIES})

# C++ library
add_library(cuda_pcl_conversions STATIC src/cuda_pcl_conversions.cpp)
add_dependencies(cuda_pcl_conversions ${cuda_pcl_conversions_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(cuda_pcl_conversions PRIVATE
        ${PCL_INCLUDE_DIRS}
        ${CUPOCH_INCLUDE_DIRS}
)
target_link_directories(cuda_pcl_conversions PRIVATE
    ${CUPOCH_LIBRARY_DIRS}
)
target_link_libraries(cuda_pcl_conversions
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CUPOCH_LIBRARIES}
)

# Tests
if(GA_BUILD_TEST)
#     add_subdirectory(test)
    add_subdirectory(tool)
endif()

# set(GA_CONFIG_FOLDER ${CMAKE_CURRENT_LIST_DIR}/config)
# set(GA_LOG_FOLDER ${CMAKE_CURRENT_LIST_DIR}/log)
# set(GA_RES_FOLDER ${CMAKE_CURRENT_LIST_DIR}/res)
# file(COPY ${GA_CONFIG_FOLDER} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
# file(COPY ${GA_LOG_FOLDER} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
# file(COPY ${GA_RES_FOLDER} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#         FILES_MATCHING PATTERN "*.h"
# )

# install(DIRECTORY res/
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/res
# )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install
install(TARGETS cuda_pcl_conversions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/cuda_pcl_conversions/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
