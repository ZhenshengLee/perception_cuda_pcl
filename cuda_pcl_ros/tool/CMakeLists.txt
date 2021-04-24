set(TOOL_NAME "cuda-pcl-filter")
cuda_add_executable(${TOOL_NAME} ./main.cpp)
target_compile_definitions(${TOOL_NAME} PRIVATE
)
target_include_directories(${TOOL_NAME} PRIVATE
    ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(${TOOL_NAME}
    cudafilter
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(${TOOL_NAME}
    ${catkin_EXPORTED_TARGETS}
)
install(TARGETS ${TOOL_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
