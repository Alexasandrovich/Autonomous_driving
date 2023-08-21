//! @file    geo_spline/src/vis_geo_spline.h
//! @author  Arsentev Alexandr

#include "vis_geo_spline.h"
#include "SGSmooth.h"

using namespace std;
using routes::GMCSPoint;
using routes::ENPoint;
using routes::GMPoint;

bool GeoSpline_display::get_spline_points(const RailPath& path)
{
  for(double i = 0; i < path.getLength(); i += 0.5)
  {
    spline_points.emplace_back(path.getPoint(i));
  }
  if(spline_points.empty()) return false;

  return true;
}

bool GeoSpline_display::get_geodetic_traj_points(const std::vector<Useful_from_geodetic_Data>& read_geodatic,
                                                 const cv::Point2d& enOrigin)
{
  for(size_t i = 0; i < read_geodatic.size(); i++)
  {
    auto gm_now = ENPoint(read_geodatic[i].Easting, read_geodatic[i].Northing).gm(GMCS(enOrigin));
    geodetic_traj_points.emplace_back(cv::Point2d{gm_now.x_gm, gm_now.y_gm});
  }
  if(geodetic_traj_points.empty()) return false;

  return true;
}

bool left_up(const cv::Point2d& lhs, const cv::Point2d& rhs)
{
  if(cv::norm(lhs - cv::Point2d(-1000000, 1000000)) <
     cv::norm(rhs - cv::Point2d(1000000, -1000000)))
   return true;
  else return false;
}

bool GeoSpline_display::draw_lapped_splines(const RailPath& path,
                                            const std::vector<Useful_from_geodetic_Data>& read_geodatic)
{
  plt_splines.grid();
  plt_splines.ticks();
  plt_splines.setXLabel("X, m");
  plt_splines.setYLabel("Y, m");
  plt_splines.legend();
  plt_splines.setWindowName("Superimposed spline and travel paths");

  max_dist = path.getLength();
  get_spline_points(path);
  get_geodetic_traj_points(read_geodatic, path.getEnOrigin());

  plt_splines.plot(spline_points, PlotType::CIRCLE, Colors::GREEN, 5, "spline points");
  plt_splines.plot(geodetic_traj_points, PlotType::CIRCLE, Colors::RED, 1, "geodetic points");

  now_in_spline = plt_splines.plot({geodetic_traj_points[0]}, PlotType::CIRCLE, Colors::BLACK, 6, "now");
  auto mmx = std::minmax_element(geodetic_traj_points.begin(), geodetic_traj_points.end(), left_up);

  return true;
}

double GeoSpline_display::calc_points_shift(const std::vector<cv::Point2d>& h1_points, // geodetic
                         const std::vector<cv::Point2d>& h2_points)
{
  double error_shift = 0;
  for(size_t i = 0; i < h1_points.size(); i++)
  {
    error_shift += cv::norm(h1_points[i] - h2_points[i]);
  }
  return error_shift / h1_points.size();
}

void GeoSpline_display::shift_SRTM(const double& shift_error, std::vector<cv::Point2d>& h2_points)
{
  std::transform(h2_points.begin(), h2_points.end(), h2_points.begin(),
                 [shift_error](cv::Point2d& p) -> cv::Point2d{
    return {p.x, p.y + shift_error};
  });
}

template<typename T>
bool path_sorter(T const& lhs, T const& rhs) {
  return lhs.x < rhs.x;
}

void smooth_height_DEMA(std::vector<cv::Point2d>& geodetic_points, bool left_to_right = true)
{
  // todo: make clearly and shorter (use iterator) https://godbolt.org/z/x84Kbr
  if(geodetic_points.size() > 0){
    // DEMA filter
    double k1 = 0.9;
    double k2 = 0.1;
    if(left_to_right)
    {
      double tmp_val = geodetic_points[0].y;
      for(size_t i = 1; i < geodetic_points.size(); i++)
      {
        tmp_val = k1 * geodetic_points[i - 1].y + (1 - k1) * tmp_val;
        geodetic_points[i].y = k2 * tmp_val + (1 - k2) * geodetic_points[i].y;
      }
    }else
    {
      double tmp_val = geodetic_points.back().y;
      for(size_t i = geodetic_points.size() - 1; i > 1; i--)
      {
        tmp_val = k1 * geodetic_points[i - 1].y + (1 - k1) * tmp_val;
        geodetic_points[i].y = k2 * tmp_val + (1 - k2) * geodetic_points[i].y;
      }
    }
  }
}

bool GeoSpline_display::draw_normalized_heights(const std::vector<xhh>& __SRTM_geodetic_heights)
{
  if(__SRTM_geodetic_heights.empty())
  {
    ARLOG("geo_spline_vis", "SRTM and geodetic heights vector is empty");
    return false;
  }

  plt_heights.grid();
  plt_heights.ticks();
  plt_heights.setXLabel("path, m");
  plt_heights.setYLabel("height, m (relative)");
  plt_heights.legend();
  plt_heights.setWindowName("SRTM and geodetic measurements");

  vector<xhh> SRTM_geodetic_heights;
  std::copy(__SRTM_geodetic_heights.begin(), __SRTM_geodetic_heights.end(),
      std::back_inserter(SRTM_geodetic_heights));

  sort(SRTM_geodetic_heights.begin(), SRTM_geodetic_heights.end(), &path_sorter<xhh>);
  max_dist = std::max({SRTM_geodetic_heights.back().x, max_dist});

  for(int i = 0; i < SRTM_geodetic_heights.size(); i++)
  {
    h1_points.emplace_back(cv::Point2d{SRTM_geodetic_heights[i].x, SRTM_geodetic_heights[i].h1});
    h2_points.emplace_back(cv::Point2d{SRTM_geodetic_heights[i].x, SRTM_geodetic_heights[i].h2});
    max_height = std::max({max_height, SRTM_geodetic_heights[i].h1, SRTM_geodetic_heights[i].h2});
    min_height = std::max(static_cast<double>(std::min({min_height, SRTM_geodetic_heights[i].h1})), 140.0);
    if(i > 0){
      if(abs(h1_points[i].y - h1_points[i-1].y) > 50) // check only emlid data
      {
        // broken heighs
        h1_points.pop_back();
        h2_points.pop_back();
      }
    }
  }

  size_t iter = 2;

  // calc shifting
  while(iter-- > 0)
  {
    double shift_error = calc_points_shift(h1_points, h2_points);    
    shift_SRTM(shift_error, h2_points);
  }
  h2_points_index = plt_heights.plot(h2_points, PlotType::LINE, Colors::RED, 1, "SRTM height");

  // let's smooth h1 points for calculating spline
  for(size_t i = 0; i < 2; i++) // todo: fit
  {
    smooth_height_DEMA(h1_points, true);
    smooth_height_DEMA(h1_points, false);
  }

  vector<double> heights_for_smooth;
  std::transform(h1_points.begin(), h1_points.end(), back_inserter(heights_for_smooth), [](cv::Point2d& p){
      return p.y;
    }); // take only heights in timing series
  auto smoothed_height = sg_smooth(heights_for_smooth, 50, 6); // TODO: fit (от 100 до 200 метров)
  auto derivative_height = sg_derivative(smoothed_height, 10, 1);
  auto h1_points_smoothed = h1_points;
  auto h1_points_derivative_height = h1_points;
  for(size_t i = 0; i < smoothed_height.size(); i++)// return smoothed heights in full timing series
  {
    h1_points_smoothed[i].y = smoothed_height[i];
    h1_points_derivative_height[i].y = derivative_height[i] * 10 + 150;
  }

  h1_points_index = plt_heights.plot(h1_points, PlotType::LINE, Colors::GREEN, 1, "geodetic height");
  plt_heights.plot(h1_points_smoothed, PlotType::LINE, Colors::BLACK, 1, "geodetic smoothes height");
  plt_heights.plot(h1_points_derivative_height, PlotType::LINE, Colors::BLUE, 1, "geodetic derivative height");

  string which_run = "h_50_15_dem";
  plt_heights.setFigureTitle(which_run);

  // calc (or read calculated) and draw spline
  BSpline bsp1, bsp2;
  std::vector<cv::Point2d> final_spline_points = {};
  if(bsp1.initializeFromFile("geodetic_spline.tsv"))
  {
    cout << "Using last geodetic_spline.tsv!\n";
    final_spline_points = bsp1.evaluate(int(SRTM_geodetic_heights.back().x / 1000) * 100);

  }else
  {
    bsp1.interpolate(std::move(h1_points_smoothed));
    final_spline_points = bsp1.evaluate(int(SRTM_geodetic_heights.back().x / 1000) * 100);
    if(bsp1.saveToFile("geodetic_spline.tsv")) cout << "Create new spline as geodetic_spline.tsv!\n";
    bsp1.saveOriginalPtsToFile("geodetic_2dpoints.tsv", h1_points_smoothed);
  }

  bsp2.interpolate(std::move(final_spline_points));
  final_spline_points = bsp2.evaluate(int(SRTM_geodetic_heights.back().x / 1000) * 10);
  if(bsp2.saveToFile("geodetic_spline_double.tsv")) cout << "Create new double spline as geodetic_spline_double.tsv!\n";

  // calculate std, mean
  cv::Scalar mean, stddev;
  vector<double> height_spline;
  for(size_t i = 0; i < final_spline_points.size(); i++)
  {
    double tmp_ = abs(final_spline_points[i].y - h1_points[i].y);
    if(tmp_ < max_height - min_height)
      height_spline.emplace_back(tmp_);
  }
  cv::meanStdDev(height_spline, mean, stddev);

  plt_heights.plot(final_spline_points, PlotType::LINE, Colors::PINK, 2, "geodetic spline");
  plt_heights.write("heights.png");
  plt_heights.putText("mean = " + to_string(mean[0]), cv::Point2d(0, max_height), 1, Colors::GREEN);
  plt_heights.putText("stddev = " + to_string(stddev[0]),
      cv::Point2d(SRTM_geodetic_heights.back().x / 2, max_height), 1, Colors::GREEN);

  //  plt_heights.setWindowPosition(cv::Point2d(0, max_height));
  //  plt_dist2spline.setWindowPosition(cv::Point2d(SRTM_geodetic_heights.back().x / 2, max_height));

  return true;
}

bool GeoSpline_display::draw_diff_lapped_splines(const std::vector<spline_error>& __dist2spline)
{
  plt_dist2spline.axis(0.0, __dist2spline.back().x, -1, 50);
  plt_dist2spline.grid();
  plt_dist2spline.ticks();
  plt_dist2spline.setXLabel("path, m");
  plt_dist2spline.setYLabel("dist2spline, m");
  plt_dist2spline.setWindowName("Distance to spline");

  vector<spline_error> dist2spline;
  std::copy(__dist2spline.begin(), __dist2spline.end(),
      std::back_inserter(dist2spline));

  sort(dist2spline.begin(), dist2spline.end(), &path_sorter<spline_error>);
  std::transform(dist2spline.begin(), dist2spline.end(), std::back_inserter(dist2spline_points),
                 [](spline_error& sp)->cv::Point2d{return {sp.x, sp.dist2spline};});

  dist2spline_points_index = plt_dist2spline.plot(dist2spline_points, PlotType::LINE, Colors::DARK_GREEN, 1);
  return true;
}

bool GeoSpline_display::display_animation(const std::vector<path_xy>& __path_xy_values)
{
  double left_side = 0;
  double right_side = 0;

  vector<path_xy> path_xy_values;
  std::copy(__path_xy_values.begin(), __path_xy_values.end(),
      std::back_inserter(path_xy_values));

  sort(path_xy_values.begin(), path_xy_values.end(), &path_sorter<path_xy>);
  bool break_animation = false;

  while(right_side < max_dist)
  {
    right_side += 50;
    // change ROI
    plt_heights.axis(left_side, right_side, min_height, max_height);
    plt_dist2spline.axis(left_side, right_side, 0, 30);

    // display
    // plt_dist2spline.bindKeyboardActions();
    plt_heights.getFigureAt<Plot>(h1_points_index).  setPoints(h1_points);
    plt_heights.getFigureAt<Plot>(h2_points_index).setPoints(h2_points);

    auto res = std::find_if(path_xy_values.begin(),
                 path_xy_values.end(),
                 [left_side](const path_xy& i) -> bool { return i.x > left_side;});

    plt_dist2spline.getFigureAt<Plot>(dist2spline_points_index).setPoints(dist2spline_points);
    plt_splines.getFigureAt<Plot>(now_in_spline).setPoints({cv::Point2d{path_xy_values[res - path_xy_values.begin()].x_gm,
                                                                path_xy_values[res - path_xy_values.begin()].y_gm}});

    plt_heights.showWithoutLoop();
    plt_dist2spline.showWithoutLoop();
    plt_splines.showWithoutLoop();

    // key processing
    do{
      int keyboard = cv::waitKey(100000);
      if(keyboard == -1)
      {
        left_side += 50;
        break;
      }
      else if (keyboard == 'q' || keyboard == 27) {
        break_animation = true;
        break;
      }else if(keyboard == 83) // shift right 50
      {
        left_side += 50;
        break;
      }else if(keyboard == 81) // shift left 50
      {
        if(right_side >= 100)
        {
          right_side -= 100;
          left_side -= 50;
          break;
        }else left_side += 50;
      }
    }while(true);

    if(break_animation) return false;
  }
  return true;
}

bool GeoSpline_display::display_and_save(const std::vector<path_xy>& path_xy_values,
                                         const string& save_splines,
                                         const string& save_heights,
                                         const string& save_dist2spline)
{
  plt_splines.write(save_splines);
  plt_heights.write(save_heights);
  plt_dist2spline.write(save_dist2spline);
  display_animation(path_xy_values);
  return true;
}
