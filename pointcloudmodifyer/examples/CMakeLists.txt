# Example executable
add_executable(example example.cpp)

# Link against the main library and PCL
target_link_libraries(example 
    pointcloudmodifyer
    ${PCL_LIBRARIES}
)

# Install example if desired
install(TARGETS example
    RUNTIME DESTINATION bin/examples
)