project(geo_spline_vis)

add_executable(geo_spline_vis
    src/main.cpp
    src/get_data.h
    src/get_data.cpp
    src/vis_geo_spline.h
    src/vis_geo_spline.cpp
    src/SGSmooth.cpp
    src/SGSmooth.h
)

target_link_libraries(geo_spline_vis
          ${OpenCV_LIBS}
          geomap # for SRTM data (private lib)
          stlutil # for working with tsv (private lib)
          coords # for reading and working with splines (private lib)
          uplotlib # vis all data (private lib)
          localization # right conversion between GMPoint and ENPoint (private lib)
          bspline # private lib
          )
