//! @file    geo_spline/src/get_data.cpp
//! @author  Arsentev Alexandr

#include "get_data.h"
using namespace std;
using routes::GMCSPoint;
using routes::ENPoint;
using routes::GMPoint;


ostream & operator<<(ostream & o, const Output &data)
{
  o << data.path;
  o << data.h_geodetic;
  o << data.h_srtm;
  o << data.name;
  o << data.x;
  o << data.y;
  o << data.Easting;
  o << data.Northing;
  o << data.dist2spline;
  return o;
}

namespace ar::tsv
{
  template<>
  inline ar::TsvWriter::Header header<Geodetic_data>()
  {
    return {
      // really useful
      {{"Name"}, Value::STRING},
      {{"Easting"}, Value::FLOAT64},
      {{"Northing"}, Value::FLOAT64},
      {{"Elevation"}, Value::FLOAT64},

      {{"Description"}, Value::STRING},
      {{"Easting RMS"}, Value::FLOAT64},
      {{"Northing RMS"}, Value::FLOAT64},
      {{"Elevation RMS"}, Value::FLOAT64},
      {{"Lateral RMS"}, Value::FLOAT64},
      {{"Antenna height"}, Value::FLOAT64},
      {{"Solution status"}, Value::STRING},
      {{"Averaging start"}, Value::FLOAT64},
      {{"Averaging end"}, Value::FLOAT64},
      {{"Samples"}, Value::FLOAT64},
      {{"Base easting"}, Value::FLOAT64},
      {{"Base northing"}, Value::FLOAT64},
      {{"Base elevation"}, Value::FLOAT64},
      {{"Baseline"}, Value::FLOAT64},

      // really useful
      {{"x_gm"}, Value::FLOAT64},
      {{"y_gm"}, Value::FLOAT64},
    };
  }

  template<>
  inline ar::TsvWriter::Header header<Output>()
  {
    return {
      {{"path"}, Value::FLOAT64},
      {{"h_geodetic"}, Value::FLOAT64},
      {{"h_srtm"}, Value::FLOAT64},
      {{"name"}, Value::STRING},
      {{"x"}, Value::FLOAT64},
      {{"y"}, Value::FLOAT64},
      {{"easting"}, Value::FLOAT64},
      {{"northing"}, Value::FLOAT64},
      {{"dist2spline"}, Value::FLOAT64}};
  }

  template<>
  inline std::vector<ar::Value> obj2row(const Output& o)
  {
    return {o.path, o.h_geodetic, o.h_srtm, o.name, o.x, o.y, o.Easting, o.Northing, o.dist2spline};
  }

  template<>
  inline Geodetic_data row2obj(const TsvReader::Row &row)
  {
    return {
      get_or_die<Value::STRING>(row, "Name"),
      get_or_die<Value::FLOAT64>(row, "Easting"),
      get_or_die<Value::FLOAT64>(row, "Northing"),
      get_or_die<Value::FLOAT64>(row, "Elevation"),

      get_or_die<Value::STRING>(row, "Description"),
      get_or_die<Value::FLOAT64>(row, "Easting RMS"),
      get_or_die<Value::FLOAT64>(row, "Northing RMS"),
      get_or_die<Value::FLOAT64>(row, "Elevation RMS"),
      get_or_die<Value::FLOAT64>(row, "Lateral RMS"),
      get_or_die<Value::FLOAT64>(row, "Antenna height"),
      get_or_die<Value::STRING>(row, "Solution status"),
      get_or_die<Value::FLOAT64>(row, "Averaging start"),
      get_or_die<Value::FLOAT64>(row, "Averaging end"),
      get_or_die<Value::FLOAT64>(row, "Samples"),
      get_or_die<Value::FLOAT64>(row, "Base easting"),
      get_or_die<Value::FLOAT64>(row, "Base northing"),
      get_or_die<Value::FLOAT64>(row, "Base elevation"),
      get_or_die<Value::FLOAT64>(row, "Baseline"),

      get_or_die<Value::FLOAT64>(row, "x_gm"),
      get_or_die<Value::FLOAT64>(row, "y_gm")
    };
  }
}

Data_origination::Data_origination(std::string init_spline_path,
                                   std::string init_srtm_path,
                                   std::string init_geodeticdata_path)
{
  Timer timer(__FUNCTION__);
  if(!init_spline_path.empty()) set_spline_path(init_spline_path);
  if(!init_srtm_path.empty()) set_srtm_path(init_srtm_path);
  if(!init_geodeticdata_path.empty()) set_geodeticdata_path(init_geodeticdata_path);
}

const std::optional<RailPath>& Data_origination::read_spline()
{
  Timer t(__FUNCTION__);

  spline_ready = spline_ready->initFromSpline(_default_spline_path);
  if(spline_ready->getLength() > 0.0){
    the_gmcs::initialize(spline_ready->getEnOrigin());
  }else{
    spline_ready.reset();
  }
  return spline_ready;
}

void sort_by_length(vector<Useful_from_geodetic_Data>& vec, const RailPath& rail_path)
{
  auto pre = GMPoint(ENPoint(vec[0].Easting, vec[0].Northing).gm(GMCS(rail_path.getEnOrigin())));
  for(int i = 1; i < vec.size() - 1; i++)
  {
    for(int j = i; j < vec.size() - 1; j++)
    {
      auto next = GMPoint(ENPoint(vec[j+1].Easting, vec[j+1].Northing).gm(GMCS(rail_path.getEnOrigin())));
      auto now = GMPoint(ENPoint(vec[j].Easting, vec[j].Northing).gm(GMCS(rail_path.getEnOrigin())));
      auto diff_now = cv::norm(cv::Point2d(pre.x, pre.y)- cv::Point2d(now.x, now.y));
      auto diff_next = cv::norm(cv::Point2d(pre.x, pre.y)- cv::Point2d(next.x, next.y));
      if(diff_now > diff_next)
      {
        swap(vec[j], vec[j+1]);
        pre = now;
      }
    }
  }
}

bool Data_origination::merge_data_and_write_tsv()
{
  Timer timer(__FUNCTION__);
  if(spline_ready.has_value() && geomap_ready.has_value() && geodeticdata_ready.has_value())
  {
    ar::tsv::ObjOStream<Output> writer("res_geo_spline.tsv");

    // sort_by_length(geodeticdata_ready.value(), spline_ready.value());

    cv::Point2d start_from = spline_ready->getEnOrigin();
    if(start_from == cv::Point2d{0, 0}) return false;
    double now_pos = 0.0;
    GMPoint prev_pos(0,0);
    double dist_near_point = 0;
    double dist_to_spline = 0;
    for(size_t i = 0; i < geodeticdata_ready.value().size(); i++)
    {
      auto gm_now = ENPoint(geodeticdata_ready.value()[i].Easting,
                            geodeticdata_ready.value()[i].Northing).gm(GMCS(spline_ready->getEnOrigin()));
      dist_near_point = cv::norm(cv::Point2d{gm_now.x_gm, gm_now.y_gm} - static_cast<cv::Point2d>(prev_pos));
      if(dist_near_point > 100)
      {
         // points are too far from each other (probably due to different location of measurements), so let's find
         // projection on full spline
         now_pos = spline_ready->getProjection(cv::Point2d{gm_now.x_gm, gm_now.y_gm});
      }else{
         now_pos = spline_ready->getProjection(cv::Point2d{gm_now.x_gm, gm_now.y_gm}, now_pos - 100, now_pos + 100);
      }

      // check that heights correct
      if(isnan(geodeticdata_ready.value()[i].Elevation))
      {
        bool is_valid = false;
        int j = i;
        while(j > 0 && !is_valid)
        {
          if(!isnan(geodeticdata_ready.value()[i].Elevation))
          {
            break;
          }
          j--;
        }
        geodeticdata_ready.value()[i].Elevation = geodeticdata_ready.value()[i].Elevation;
      }

      dist_to_spline = spline_ready->distToSpline(cv::Point2d{gm_now.x_gm, gm_now.y_gm}, now_pos - 50, now_pos + 50);
      // anology with @now_pos variable
      if(dist_to_spline > 50) dist_to_spline = spline_ready->distToSpline(cv::Point2d{gm_now.x_gm, gm_now.y_gm});

      // save previous pose for check distance between them
      prev_pos = gm_now;

      writer << Output{now_pos,
                geodeticdata_ready.value()[i].Elevation,
                geomap_ready.value()[i].height,
                geodeticdata_ready.value()[i].name,
                spline_ready->getPoint(now_pos).x, // on spline
                spline_ready->getPoint(now_pos).y,
                geodeticdata_ready.value()[i].Easting,
                geodeticdata_ready.value()[i].Northing,
                dist_to_spline};

      // save some data for visualization
      final_heights.emplace_back(std::move(xhh{now_pos, geodeticdata_ready.value()[i].Elevation,
                                              geomap_ready.value()[i].height}));
      dist2spline.emplace_back(std::move(spline_error{now_pos, dist_to_spline}));
      path_with_xygm.emplace_back(std::move(path_xy{now_pos, gm_now.x_gm, gm_now.y_gm}));
    }
  }else
  {
    ARLOG("geo_spline_vis", "(is_spline_ready && is_srtm_ready && is_geodeticdata_ready) = false");
    return false;
  }
  return true;
}


optional<vector<xy_h>> Data_origination::get_data_from_geomap()
{
  Timer timer(__FUNCTION__);

  elevation_map.setHeightDataFolder(_default_srtm_path);
  double example_meters = elevation_map.elevation({37.66937,55.75721}); // Moscow-center
  if(example_meters >= 4000000000){
    ARLOG("geo_spline_vis", "bad geomap data/path");
    geomap_ready.reset();
    return geomap_ready;
  }

  if(geodeticdata_ready)
  {
    for(const auto& i : geodeticdata_ready.value())
    {
      read_srtm_by_one_EN({i.Easting, i.Northing});
    }
  }else
  {
    ARLOG("geo_spline_vis", "geomap data isn't ready");
    geomap_ready.reset();
  }
  return geomap_ready;
}

void Data_origination::read_srtm_by_one_EN(const ENPoint &en_point)
{
  auto srtm_value = xy_h{en_point, elevation_map.elevation(ENPoint2d{static_cast<cv::Point2d>(en_point)})};
  if(geomap_ready.has_value()) geomap_ready.value().emplace_back(std::move(srtm_value));
  else geomap_ready = {srtm_value};
}

string try_emplace(const vector<vector<string>>& data, const size_t& row_index,
                   const string&& prob_header_name, size_t&& prob_column_index)
{
  try{
    if(data[0][prob_column_index] != prob_header_name) // "0" is header
    {
      // probably format was changes. Let's find correct column index
      auto found_col_index = std::find(data[0].begin(), data[0].end(), prob_header_name);
      if(found_col_index != data[0].end())
      {
        prob_column_index = found_col_index - data[0].begin();
      }else
      {
        ARLOG("geo_spline_vis", "bad index/name of cell - check format of file (" +
              prob_header_name + ")");
        return "";
      }
    }
    return data[row_index][prob_column_index];
  }catch(...)
  {
    ARLOG("geo_spline_vis", "bad table");
    return "";
  }
}

double try_atof(const string& str)
{
  double res = 0.0;
  if(str.empty()) return res;
  try
  {
    res = atof(str.c_str());
  }catch(...){}
  return res;
}

#include <iostream>

optional<vector<Useful_from_geodetic_Data>> Data_origination::read_geodetic_data()
{
  Timer timer(__FUNCTION__);
  sx::ifdelimstream ifs(_default_geodeticdata_path.c_str(), ",");
  vector<vector<string>> csv_data;
  ifs >> csv_data;

  for(size_t i = 1; i < csv_data.size(); i++) // "0" is header
  {
    auto useful_member = Useful_from_geodetic_Data{
                                try_emplace(csv_data, i, "Name", 1),
                                try_atof(try_emplace(csv_data, i, "Easting", 2)),
                                try_atof(try_emplace(csv_data, i, "Northing", 3)),
                                try_atof(try_emplace(csv_data, i, "Elevation", 4)),
                                try_atof(try_emplace(csv_data, i, "x_gm", 19)),
                                try_atof(try_emplace(csv_data, i, "y_gm", 20))};

    if(geodeticdata_ready.has_value()) geodeticdata_ready.value().emplace_back(std::move(useful_member));
    else geodeticdata_ready = {std::move(useful_member)};
  }
  if(geodeticdata_ready->empty()){
    geodeticdata_ready.reset();
  }
  return geodeticdata_ready;
}

optional<vector<Useful_from_geodetic_Data>> Data_origination::read_geodetic_data_as_smart()
{
  ar::tsv::ObjIStream<Geodetic_data> reader(_default_geodeticdata_path);

  while(reader.has_data())
  {
    Geodetic_data gd; reader >> gd;

    auto useful_member = std::move(Useful_from_geodetic_Data{
                                     gd.Name,
                                     gd.Easting,
                                     gd.Northing,
                                     gd.Elevation,
                                     gd.x_gm,
                                     gd.y_gm });

    if(geodeticdata_ready.has_value()) geodeticdata_ready.value().emplace_back(std::move(useful_member));
    else geodeticdata_ready = {std::move(useful_member)};
  }
  if(geodeticdata_ready.value().empty()){
    geodeticdata_ready.reset();
  }
  return geodeticdata_ready;
}

