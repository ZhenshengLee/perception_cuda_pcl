if (NOT TARGET cudafilter)
    add_subdirectory(src EXCLUDE_FROM_ALL)
else()
    message(STATUS "cudafilter is already a target. Skipping adding it twice")
endif()

include_directories(./src/include)

add_library(ga::cudafilter ALIAS cudafilter)

# CMake doesn't allow to install IMPORTED libraries as TARGETS. Use install(FILES) instead.
get_target_property(CUDA_FILTER_LIB_FILE cudafilter IMPORTED_LOCATION)

file(COPY
    ${CUDA_FILTER_LIB_FILE} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    FOLLOW_SYMLINK_CHAIN
)
