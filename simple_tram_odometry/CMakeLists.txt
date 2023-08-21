project(tram_odometry)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(simple_tram_odometry
    src/main.cpp
    src/estimate_translation.h
    src/estimate_translation_mono.cpp
    src/evaluate_poses.cpp
    src/find_essential.cpp
    src/kittiodometer.h
    src/kittiodometer.cpp
    src/setup_optflow.h
    src/setup_optflow.cpp
    src/snapshot.h
    src/tram_filter.h
    src/tram_filter.cpp
    src/precompiled.h
)

target_link_libraries(simple_tram_odometry
          ${OpenCV_LIBS}
        roadobjectpool # (private libs)
        localization
        stlutil
        arcore
        coords
        warping
        worldobject
        worldobjectpool
        uplotlib
        geomap
        sysutil
        arml
        semanticsegm
        imgproc
        math_functions
        popcorn4_core
        )
