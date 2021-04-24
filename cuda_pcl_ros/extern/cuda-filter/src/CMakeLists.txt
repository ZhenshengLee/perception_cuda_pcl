project(cudafilter)

find_package(PCL REQUIRED QUIET)

include_directories(
    ./include
    ${CUDA_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
link_directories(lib)

set(LIB_NAME "cudafilter")
add_library(${LIB_NAME} SHARED IMPORTED GLOBAL)
set_target_properties(${LIB_NAME} PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/lib${LIB_NAME}.so
)
add_dependencies(${LIB_NAME}
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
)
