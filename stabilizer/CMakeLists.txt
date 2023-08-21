project(stabilizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(stabilizer_HDRS
    stabilizer.h
    feature_match.h)

set(stabilier_SRCS
    src/stabilizer.cpp
    src/feature_match.cpp)

add_library(stabilizer 
  ${stabilizer_HDRS}
  ${stabilier_SRCS}
)

target_link_libraries(stabilizer
    ${OpenCV_LIBS}
    arcore
    rtdb)

add_subdirectory(example)

