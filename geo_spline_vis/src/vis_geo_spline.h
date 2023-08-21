//! @file    geo_spline/vis_geo_spline.h
//! @author  Arsentev Alexandr
//! @brief   Display SRTM, geodetic, spline data on one way
//! @date    18.01.2021

#include <uplotlib/uplotlib.h>
#include <bspline/bspline.h>
#include <algorithm>
#include <iostream>
#include <string>
#include "get_data.h"

#pragma once

class GeoSpline_display
{
public:
  //! @brief getting all spline points
  //! @param[in] spline path
  //! @retval - status
  bool get_spline_points(const RailPath& path);

  //! @brief getting all geodetic data points
  //! @param[in] read_geodatic - read geodetic points
  //! @param[in] enOrigin - base point for right orientation
  //! @retval - status
  bool get_geodetic_traj_points(const std::vector<Useful_from_geodetic_Data>& read_geodatic,
                                const cv::Point2d &enOrigin);

  //! @brief putting on spline and geodetic measurements for comparing trajectories
  //! @param[in] path - spline path
  //! @param[in] read_geodatic - geodetic points
  //! @retval - status
  bool draw_lapped_splines(const RailPath& path,
                           const std::vector<Useful_from_geodetic_Data>& read_geodatic);

  //! @brief caclulating SRTM and geodetic data shift, which appeared due to different sources
  //! @param[in] h1_points - geodetic height
  //! @param[in] h2_points - srtm points
  //! @retval - shift (meter)
  double calc_points_shift(const std::vector<cv::Point2d>& h1_points,
                           const std::vector<cv::Point2d>& h2_points);

  //! @brief running calculated shift to SRTM heights
  //! @param[in] shift_error - caclulated SRTM shift
  //! @param[in] h2_points - SRTM data
  void shift_SRTM(const double& shift_error, std::vector<cv::Point2d>& h2_points);

  //! @brief drawing SRTM-geodetic-heights for ability local comparison and spline for geodetic data
  //! @param[in] SRTM_geodetic_heights - geodetic height
  //! @param[in] h2_points - srtm points
  //! @retval - status
  bool draw_normalized_heights(const std::vector<xhh> &SRTM_geodetic_heights);

  //! @brief drawing error lapped splines (geodetic trajectory and spline)
  //! @param[in] __dist2spline - projection error
  //! @retval - status
  bool draw_diff_lapped_splines(const std::vector<spline_error>& __dist2spline);

  //! @brief display local measurments (shift - 50 meters)
  //! @note use arrow keys on keyboard (left; right; q - exit)
  //! @param[in] path_xy_values - data for local window shifting
  //! @retval - status
  bool display_animation(const std::vector<path_xy> &path_xy_values);

  //! @brief display animation with local features and save it
  //! @param[in] path_xy_values - data for local window shifting
  //! @param[in] save_splines - path to save splines image
  //! @param[in] save_heights - path to save heights image
  //! @param[in] save_dist2spline - path to save dist2spline image
  //! @retval - status
  bool display_and_save(const std::vector<path_xy>& path_xy_values,
                        const std::string& save_splines = "splines.png",
                        const std::string& save_heights = "heigts.png",
                        const std::string& save_dist2spline = "dist2spline.png");

private:
  CPlot plt_splines; //!< spline image
  CPlot plt_heights; //!< heights image
  CPlot plt_dist2spline; //!< dist2spline projection image

  double max_dist = 0.0;
  double max_height = 0.0;
  double min_height = 0.0;

  // indexes for redrawing some features
  int h1_points_index = -1;
  int h2_points_index = -1;
  int dist2spline_points_index = -1;
  int now_in_spline = -1;

  // data for drawing
  std::vector<cv::Point2d> spline_points;
  std::vector<cv::Point2d> geodetic_traj_points;
  std::vector<cv::Point2d> h1_points;
  std::vector<cv::Point2d> h2_points;
  std::vector<cv::Point2d> dist2spline_points;
};
