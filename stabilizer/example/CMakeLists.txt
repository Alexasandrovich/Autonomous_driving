project(stabilizer_example)

add_executable(stabilizer_example stab_example.cpp)
target_link_libraries(stabilizer_example stabilizer sysutil warping)
