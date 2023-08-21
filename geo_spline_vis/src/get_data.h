//! @file    geo_spline/get_data.h
//! @author  Arsentev Alexandr
//! @brief   Collecting SRTM, geodetic, spline data on one way
//! @date    18.01.2021

#include <coords/rail_path.h>
#include <geomap/elevation.h>
#include <coords/coordinatetransformer.h>
#include <ar10/sx_srm.h>
#include <ar10/tsvio.h>
#include <localization/trm_presets.h>

#include <string>
#include <vector>
#include <cstdint>
#include <iostream>
#include <optional>

#pragma once

//! @brief Struct for time measure
struct Timer
{
  long long start = 0;
  std::string title; //!< name of timer
  double msec() { return (1000 * (cv::getTickCount() - start)) / cv::getTickFrequency();}
  Timer(std::string title): title(title) { start = cv::getTickCount();}
  ~Timer() { std::cout << title << " " << msec() << " msec\n";}
};

//! @brief Data from geodetic measurments
struct Geodetic_data{
  std::string Name; // railroad switch links
  double Easting; // in EN
  double Northing; // in EN
  double Elevation; // in meters
  std::string Description;
  double Easting_RMS;
  double Northing_RMS;
  double Elevation_RMS;
  double Lateral_RMS;
  double Antenna_height;
  std::string Solution_status;
  double Averaging_start;
  double Averaging_end;
  double Samples;
  double Base_easting;
  double Base_northing;
  double Base_elevation;
  double Baseline;
  double x_gm; // in GM
  double y_gm; // in GM
};

//! @brief Useful data from geodetic measurments
struct Useful_from_geodetic_Data
{
  std::string name; //!< name of railway switches
  double Easting; // in EN
  double Northing; // in EN
  double Elevation; // in meters
  double x_gm; // in GM
  double y_gm; // in GM
};

//! @brief Useful data from geodetic measurments
struct xy_h
{
  ENPoint2d xy; //!< 2d point (EN-format)
  double height; //!< height for xy-point (meter)
};

//! @brief Data for drawing heights
struct xhh
{
  double x; //!< the distance traveled from the beginning of the report in meters
  double h1; //!< gedetic heights
  double h2; //!< srtm (geomap) heights
};

//! @brief Data for drawing spline projection error
struct spline_error
{
  double x; //!< the distance traveled from the beginning of the report in meters
  double dist2spline; //!< projection geodetic path on spline
};

//! @brief Data for drawing splines
struct path_xy
{
  double x; //!< the distance traveled from the beginning of the report in meters
  double x_gm;
  double y_gm;
};

//! @brief Data for writting in tsv-file
struct Output
{
  double path;
  double h_geodetic;
  double h_srtm;
  std::string name;
  double x;
  double y;
  double Easting;
  double Northing;
  double dist2spline;
};

class Data_origination{
public:
  //! @brief constructor for init paths to spline, SRTM, geodetic data
  //! @param[in] init_spline_path - spline path
  //! @param[in] init_srtm_path - SRTM path (for geomap)
  //! @param[in] init_geodeticdata_path - geodetic data path
  Data_origination(std::string init_spline_path = "",
                   std::string init_srtm_path = "",
                   std::string init_geodeticdata_path = "");

  //! @brief spline reading using init_spline_path
  //! @retval - spline if it was read. Else - false value
  const std::optional<RailPath> &read_spline();

  //! @brief init geomap for getting SRTM data using init_srtm_path
  //! @retval - SRTM data if it was read. Else - false value
  std::optional<std::vector<xy_h>> get_data_from_geomap();

  //! @brief getting SRTM data by ENPoint
  //! @param[in] en_point - EN point
  void read_srtm_by_one_EN(const routes::ENPoint &en_point);

  //! @brief reading geodetic data as csv-format
  //! @retval - geodetic data if it was read. Else - false value
  std::optional<std::vector<Useful_from_geodetic_Data>> read_geodetic_data();

  //! @brief reading geodetic data as tsv-format
  //! @retval - geodetic data if it was read. Else - false value
  std::optional<std::vector<Useful_from_geodetic_Data> > read_geodetic_data_as_smart();

  //! @brief merging spline, geodetic and STRM data for checking them validity
  //! @retval - status
  bool merge_data_and_write_tsv();

  //! @brief returning final normalized heights (SRTM + geodetic)
  //! @retval - heights
  const std::vector<xhh>& get_heights() { return final_heights; }

  //! @brief returning final dist2spline values
  //! @retval - projection geodetic data on spline
  const std::vector<spline_error>& get_dist2spline() { return dist2spline; }

  //! @brief returning final normalized heights (SRTM + geodetic)
  //! @retval - struct of 2d points + path (0, ..., path.length())
  const std::vector<path_xy>& get_path_xy() { return path_with_xygm; }

  //! @brief setting new path to spline
  //! @param[in] new_path - new path to spline
  //! @retval - object of Data_origination for probably next init (.set_srtm_path().set_geodeticdata_path())
  Data_origination& set_spline_path(const std::string & new_path) {_default_spline_path = new_path; return *this;}

  //! @brief setting new path to SRTM data
  //! @param[in] new_path - new path to SRTM
  //! @retval - object of Data_origination
  Data_origination& set_srtm_path(const std::string & new_path) {_default_srtm_path = new_path; return *this;}

  //! @brief setting new path to geodetic data
  //! @param[in] new_path - new path to geodetic
  //! @retval - object of Data_origination
  Data_origination& set_geodeticdata_path(const std::string & new_path) {_default_geodeticdata_path = new_path; return *this;}

private:
  std::string _default_spline_path = "/testdata/trm/spline/17.1/route.spline.tsv";
  std::string _default_srtm_path = "/testdata/maps/elevation/srtm/mosregion";
  std::string _default_geodeticdata_path = "/testdata/geosurvey/v36/out_VDNH.csv";

  MapElevation elevation_map; //!< all SRTM heights in geomap

  // output_data
  std::vector<xhh> final_heights;
  std::vector<spline_error> dist2spline;
  std::vector<path_xy> path_with_xygm;

  std::optional<RailPath> spline_ready;
  std::optional<std::vector<xy_h>> geomap_ready;
  std::optional<std::vector<Useful_from_geodetic_Data>> geodeticdata_ready;
};

//! @brief try to emplace data from geodetic table
//! @param[in] data - table
//! @param[in] row_index - index of current row
//! @param[in] prob_header_name - name of depended col
//! @param[in] prob_column_index - index of depended col
//! @retval value
std::string try_emplace(const std::vector<std::vector<std::string>>& data, const size_t& row_index,
                   const std::string&& prob_header_name, size_t&& prob_column_index);

//! @brief try to emplace data from geodetic table
//! @param[in] str - got value
//! @retval value as fix\float number
double try_atof(const std::string& str);


//! @brief sort geodetic points by length from start
//! @note It must be sort because geodetic measurments
//! have been made in different moments in differents place, so
//! for projecting on spline it depends full spline passing - too long
//! @param[in] vec - geodetic data
//! @param[in] rail_path - spline data
void sort_by_length(std::vector<Useful_from_geodetic_Data>& vec, const RailPath& rail_path);
