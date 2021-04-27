cmake_minimum_required(VERSION 3.15)
project(cuda_pcl_ros)

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

message(STATUS "zs: ga_cupoch_test CUDA_NVCC_FLAGS = " ${CUDA_NVCC_FLAGS})

catkin_package(
        INCLUDE_DIRS ./extern/cuda-filter/src/include
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
        ./extern/cuda-filter/src/include
        ${CUDA_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
link_directories(${CUPOCH_LIBRARY_DIRS})
link_libraries(${CUPOCH_LIBRARIES})

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
        add_subdirectory(extern)
        add_subdirectory(modules)
        add_subdirectory(tool)
        catkin_package(
                INCLUDE_DIRS ./extern/cuda-filter/src/include
                LIBRARIES cudafilter
        )
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

install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
