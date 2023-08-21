// этот файл в utf-8
#include "precompiled.h"
#pragma hdrstop
#include <iostream>
#include <ctime>
#include <opencv2/datasets/util.hpp>
#include "kittiodometer.h"
#include "setup_optflow.h"
#include "estimate_translation.h"
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <roadobjectpool/roadobjectview.h>
#include <localization/src/tram_path_loc_with_height.h>
#include <localization/trm_presets.h>
#include <dbw_lib/dbw_wrappers.h>
#include <arcore/senseframe.h>
#include <arcore/locstruct_from_frame.h>
#include <utility>
#include <bspline/bspline1d.h>
#include <bspline/spline_utils.h>
#include <geomap/geomap_renderer.h>
#include <geomap/geomapeditor.h>
#include <geomap/agm_objects.h>
#include <geomap/geomap.h>
#include <geomap/geomap_tools.h>
#include <ar10/arlog.h>
#include <ar10/tsvio.h>
#include <arml/libmxnetclient.h>
#include <arml/libmxnetserver.h>
#include <semanticsegm/src/drivareasegmenter.h>
#include <ar10/dashmap.h>
#include <magical_toolkit/math_functions/math_functions.h>
#include <popcorn4/src/regression.h>
#include <popcorn4/src/dema.h>
#include <localization/src/harv_aekf_gps_odo_loc.h>

using namespace std;
using namespace cv;
using namespace cv::optflow;
using namespace optflow;
using routes::GMCSPoint;
using routes::ENPoint;
using routes::GMPoint;

#define debug_speed

namespace ar::tsv
{
  template<>
  inline ar::TsvWriter::Header header<Geodetic_data>()
  {
    return {
      {{"path"}, Value::FLOAT64},
      {{"h_geodetic"}, Value::FLOAT64},
      {{"h_srtm"}, Value::FLOAT64},
      {{"x"}, Value::FLOAT64},
      {{"y"}, Value::FLOAT64},
      {{"name"}, Value::STRING},
      {{"easting"}, Value::FLOAT64},
      {{"northing"}, Value::FLOAT64},
      {{"dist2spline"}, Value::FLOAT64}};
  }

  template <>
  inline Geodetic_data
  row2obj<Geodetic_data>(const TsvReader::Row &row)
  {
    Geodetic_data g;
    g.path = tsv::get_or_die<ar::Value::FLOAT64>(row, "path");
    g.h_geodetic = tsv::get_or_die<ar::Value::FLOAT64>(row, "h_geodetic");
    g.h_srtm = tsv::get_or_die<ar::Value::FLOAT64>(row, "h_srtm");
    g.x = tsv::get_or_die<ar::Value::FLOAT64>(row, "x");
    g.y = tsv::get_or_die<ar::Value::FLOAT64>(row, "y");
    g.name = tsv::get_or_die<ar::Value::STRING>(row, "name");
    g.Easting = tsv::get_or_die<ar::Value::FLOAT64>(row, "easting");
    g.Northing = tsv::get_or_die<ar::Value::FLOAT64>(row, "northing");
    g.dist2spline = tsv::get_or_die<ar::Value::FLOAT64>(row, "dist2spline");
    return g;
  }

  template<>
  inline ar::TsvWriter::Header header<Geodetic_data_poles>()
  {
    return {
      {{"height"}, Value::FLOAT64},
      {{"easting"}, Value::FLOAT64},
      {{"northing"}, Value::FLOAT64},
      {{"x_gm"}, Value::FLOAT64},
      {{"y_gm"}, Value::FLOAT64}};
  }

  template <>
  inline Geodetic_data_poles
  row2obj<Geodetic_data_poles>(const TsvReader::Row &row)
  {
    Geodetic_data_poles g;
    g.heght = tsv::get_or_die<ar::Value::FLOAT64>(row, "height");
    g.easting = tsv::get_or_die<ar::Value::FLOAT64>(row, "easting");
    g.northing = tsv::get_or_die<ar::Value::FLOAT64>(row, "northing");
    g.x_gm = tsv::get_or_die<ar::Value::FLOAT64>(row, "x_gm");
    g.y_gm = tsv::get_or_die<ar::Value::FLOAT64>(row, "y_gm");

    return g;
  }

  template<>
  inline ar::TsvWriter::Header header<Stand_emlid_data>()
  {
    return {
      {{"grabMsec"}, Value::INT64},
      {{"nord"}, Value::FLOAT64},
      {{"east"}, Value::FLOAT64},
      {{"speed"}, Value::FLOAT64},
      {{"alt"}, Value::FLOAT64},
      {{"fixQuality"}, Value::INT64}};
  }

  template <>
  inline Stand_emlid_data
  row2obj<Stand_emlid_data>(const TsvReader::Row &row)
  {
    Stand_emlid_data g;
    g.grabMsec = tsv::get_or_die<ar::Value::INT64>(row, "grabMsec");
    g.nord = tsv::get_or_die<ar::Value::FLOAT64>(row, "nord");
    g.east = tsv::get_or_die<ar::Value::FLOAT64>(row, "east");
    g.speed = tsv::get_or_die<ar::Value::FLOAT64>(row, "speed");
    g.alt = tsv::get_or_die<ar::Value::FLOAT64>(row, "alt");
    g.fixQuality = tsv::get_or_die<ar::Value::INT64>(row, "fixQuality");
    return g;
  }
}

int KittiOdometer::estimate_translation(Snapshot& ss_pre, Snapshot& ss_cur)
{
  return estimate_translation_mono(ss_pre, ss_cur, ksdata.speedmap, setup);
}

void KittiOdometer::process_two_shots(Snapshot& ss_pre, Snapshot& ss_cur)
{
  // получаем поворот и НАПРАВЛЕНИЕ смещения, т.е. потом нужно будет найти его модуль
  find_essential(ss_pre, ss_cur);

  if(ss_cur.le.t[0] < 0.0001 && ss_cur.le.t[1] < 0.0001 && ss_cur.le.t[2] < 0.0001)
  {
    // стотим на месте (общий модуль векторо оптического потока маленький) => поза околонулевая
    cout << "No driving => waiting" << "\n";
    auto& pose_pre = ksdata.poses_re[ss_pre.ipng];

    // так как поза накапливается, то скажем, что текущая поза равна прошлой
    auto R_pre = pose_pre.rotation();     // поворот в предыдущий шот
    auto t_pre = pose_pre.translation();  // сдвиг в предыдущий шот
    auto R_cur = R_pre;
    auto t_cur = t_pre;
    ksdata.poses_re.emplace_back(Affine3d(R_cur, t_cur));
    return;
  }

  // ищем модуль смещения
  estimate_translation(ss_pre, ss_cur);

  auto& pose_pre = ksdata.poses_re[ss_pre.ipng];
  auto R_pre = pose_pre.rotation();     // поворот в предыдущий шот
  auto t_pre = pose_pre.translation();  // сдвиг в предыдущий шот
  auto R_cur = R_pre * ss_cur.le.R.t(); // Вот... новый поворот...
  auto t_cur = R_cur * ss_cur.le.t + t_pre;

  // отпишем GT и позы
  if(setup.bools["write_GT_as_XYcoord"] == true)
    GT_writing.writeRow({ t_cur[2], t_cur[0] });

  // новые позы
  ksdata.poses_re.emplace_back(Affine3d(R_cur, t_cur));

  /*if(setup.bools["use_eval_poses_cache"] == true)
  {
    ofstream cache_poses_re(dataset_folder + "cache_poses_re.txt");
    for (const auto &pose : ksdata.poses_re) cache_poses_re << pose << "\n";
  }*/
}

void KittiOdometer::process_snapshot(Snapshot& ss)
{
  Timer t("processing snapshot");
  // зарегистрируем пустую позу для нулевого кадра
  if (ss.ipng <= 1)
  {
    Affine3d empty_pose;
    ksdata.poses_re.emplace_back(empty_pose);
    ksdata.evaluate_poses_intermediate(ss.ipng, ss.ipng);
    return;
  }
  // число кадров для отмотки назад, чтобы взять snapshot как "предыдущий",
  // чтобы сдвиг между кадрами был не более, чем setup.doubles["minstep"]
  int ishot = 1;
  // текущий кадр
  auto curr_pose = ksdata.poses_re[ss.ipng - 1];
  double minstep = setup.doubles["minstep"];

  while (ishot < ksdata.snapshots.size() - 1)
  {
    // кандидат на сдвиг
    auto& pose_prev = ksdata.poses_re[ss.ipng - ishot - 1];
    auto dt = curr_pose.translation() - pose_prev.translation();
    double step = norm(dt);
    if (step > minstep) break; // уже далековато сдвинулись назад - стоп
    ishot++;
  };

  // ограничение для случая, если мы долго стоим на месте и не можем вернуться к случаю,
  // когда оптический поток считается между 2-мя ПОСЛЕДОВАТЕЛЬНЫМИ кадрами.
  // Если же стоит долго и оптический поток недостоверный (как и другие параметры) - не учитываем результат
  if(ishot > 5) { ishot = 1;}

  // запускаем алгоритм нахождения визуальной одометрии
  // ishot-предыдуший кадр, 0-текущий кадр
  process_two_shots(ksdata.snapshots[ishot], ksdata.snapshots[0]);

  // чистим очередь
  while (ksdata.snapshots.size() > min( 200, ishot + 1 )){
      // вытолкнем всех кто старше ishot -- они уже не понадобятся.
      // а ishot еще может пригодиться если медленно едем
    ksdata.snapshots.pop_back();
  }

  // отрисовка полученных углов
  ksdata.evaluate_poses_intermediate(ss.ipng, ss.ipng - ishot);
}

string get_season_folder(const string &season_pattern)
{
  auto tp = ar::TestdataPosition::fromPattern(season_pattern);
  return afs::splitLast(tp.fullPath()).first;
}

static Camera read_cam_calibs(const string &season_pattern, int icam)
{
  auto season_dir = get_season_folder(season_pattern);
  Camera cam;
  if (icam == 0) /// центральная или левая
  {
    if (
      cam.read(afs::join(season_dir, "calib/left.yml")) ||
      cam.read(afs::join(season_dir, "calib/leftImage.yml")) ||
      cam.read(afs::join(season_dir, "calib/cam_mono.yml")) ||
      cam.read(afs::join(season_dir, "calib/cam_plg_left.yml")) ||
      cam.read(afs::join(season_dir, "calib/central60.yml")) ||
      cam.read(afs::join(season_dir, "../calib/cam_mono.yml")) ||
      cam.read(afs::join(season_dir, "../calib/cam_plg_left.yml"))
      )
      return cam;
    else
      cout << "Left/mono camera calibs not found" << endl;
  }
  return cam;
}

void KittiSequenceData::make_legend(cv::Scalar color_line, std::string name_4legend, int y_level)
{
	line(local_map, Point(1700, y_level), Point(1700 + 60, y_level), color_line, 2);
	putText(local_map, "- " + name_4legend, Point(1700 + 60 + 10, y_level + 2), 1, 1, {0, 0, 0});
}

void KittiSequenceData::draw_angle(double angle_type, cv::Scalar color_line, string name_4legend, int y_level)
{
	make_legend(color_line, name_4legend, y_level);
	Point gg = Point(snapshots[0].ipng * 2 + angle_map_start.x, (-angle_type * how_timed_incr_Y + angle_map_start.y));
	// масштабирование в зависимости от количества эпизодов на ОДНОМ графике
	gg.x /= how_times_incr;
	gg.x += bias;
	rectangle(local_map, gg, gg - Point(0, 1), color_line, name_4legend == "re_yaw" ? 1 : 2 , LINE_AA);
}

void KittiSequenceData::draw_diff_angle(double angle_type_GT, double angle_type_RE, cv::Scalar color_line, string name_4legend, int y_level)
{
	make_legend(color_line, name_4legend, y_level);
	double diff = fmod( angle_type_RE - angle_type_GT + 180. + 3600, 360.) - 180.;
	Point dd = Point(snapshots[0].ipng * 2 + angle_map_start.x, (-diff * 100 * how_timed_incr_Y + angle_map_start.y)); // 0.01 градус на один пиксель
	dd.x /= how_times_incr;
	dd.x += bias;
	rectangle(local_map, dd, dd - Point(0, 1), color_line, 1, LINE_AA);
}

void KittiSequenceData::draw_speed(double speed_re, double speed_gt, int diff_png, double diff_translation)
{
	speed_re = speed_re * 100 * translation2speed / (diff_png);
	speed_gt = speed_gt * 100 * translation2speed / (diff_png);

	// полученная скорость
	Point rr = Point(snapshots[0].ipng * 2 + height_map_start.x, -speed_re * speed_ON_local_map + height_map_start.y);
	// GT-скорость
	Point gg = Point(snapshots[0].ipng * 2 + height_map_start.x, -speed_gt * speed_ON_local_map + height_map_start.y);

	Point diff = Point(snapshots[0].ipng * 2 + height_map_start.x, -diff_translation + height_map_start.y);

	// масштабирование, чтобы на графике вмещалось больше кадров
	rr.x /= how_times_incr;
	rr.x += bias;

	gg.x /= how_times_incr;
	gg.x += bias;

	diff.x /= how_times_incr;
	diff.x += bias;

	// отрисовка скоростей (чёрный цвет - GT, зелёный - вычисленная)
	rectangle(local_map, gg, gg - Point(0,1), Scalar(64, 64, 64), 2);
	rectangle(local_map, rr, rr - Point(0,1),
						snapshots[0].ipng == snapshots[0].ipng - diff_png + 1 ? Scalar(0, 255, 0) : Scalar(128, 255, 0), 2);

	rectangle(local_map, diff, diff - Point(0,1), Scalar(150, 150, 150), 2);

	make_legend(Scalar(64, 64, 64), "gt_speed", 115);
	make_legend(Scalar(0, 255, 0), "eval_speed", 130);
	make_legend(Scalar(150, 150, 150), "diff_translation", 145);

#ifdef debug_speed

	Mat local_map_debug = local_map.clone();
	string episode = "Current episode: " + to_string(snapshots[0].iseq);
	string cur_time = "Current time: " + to_string(int(snapshots[0].time));
	Point p1 = { 100, local_map.rows * 1 / 9};
	Point p2 = { 100, local_map.rows * 1 / 9 + 20};
	putText(local_map_debug, episode, p1, 1, 1, {0, 0, 0});
	putText(local_map_debug, cur_time, p2, 1, 1, {0, 0, 0});

	string speed_eval = "Speed_eval: " + to_string(speed_re);
	string speed_gt_ = "Speed_GT: " + to_string(speed_gt);
	Point p3 = { 100, local_map.rows * 1 / 9 + 40};
	Point p4 = { 100, local_map.rows * 1 / 9 + 60};
	putText(local_map_debug, speed_eval, p3, 1, 1, {0, 0, 0});
	putText(local_map_debug, speed_gt_, p4, 1, 1, {0, 0, 0});

	imshow("local_map", local_map_debug);

#else
    imshow("local_map", local_map);
#endif
}

void KittiSequenceData::draw_trajectory(cv::Vec3d re_pose_cur, int angle_rot_map)
{
	Point drawing_eval_speed = Point(re_pose_cur[0], -re_pose_cur[2]) + local_map_start;
	if(snapshots[0].ipng < 4){
			diff_start_gt_eval = common_start_gt_eval - drawing_eval_speed;
	}
	else{
		// синхронизация gt и eval_pose
		drawing_eval_speed += diff_start_gt_eval;
		drawing_eval_speed -= common_start_gt_eval;
		Vec2d point_no_rot = { static_cast<double>(drawing_eval_speed.x),
													 static_cast<double>(drawing_eval_speed.y)};
		Vec2d point_with_rot = gen_mat_rotation_map(angle_rot_map) * point_no_rot;
		drawing_eval_speed = Point(point_with_rot[0], point_with_rot[1]) + common_start_gt_eval;
	}
	// отрисовка полученного собственного движения (смещения)
	drawing_eval_speed.x = (drawing_eval_speed.x) /  20 + 300;
	drawing_eval_speed.y = (drawing_eval_speed.y) / 20 + 500;
	circle(local_map, drawing_eval_speed, 2, Scalar(0, 128, 255), 1);
}

vector<string> KittiSequenceData::gen_trk_list(const InputEntry &ie, int finishEp)
{
  vector<string> res;
  for (int i = ie.season_entry.startEpisode.as<int>(); i < finishEp; ++i)
  {
    res.push_back(cv::format(ie.trk_file.c_str(), i));
  }
  return res;
}

VehicleModel read_vehicle_model(const string &vehicle_yml)
{
  VehicleModel vm;
  cv::FileStorage fs(afs::absolutePath(vehicle_yml), cv::FileStorage::READ);
  const std::string vehicle_dict = afs::absolutePath("../data/vehicle/vehicle_dict.yml"); //местоположение нашего словарика с параметраи наших машин

  const cv::FileStorage dict(vehicle_dict, cv::FileStorage::READ);
  if (fs.isOpened())
  {
      const auto name = fs["vehicleName"];
      auto params = dict[name];
      if(!params.empty())
      {
          vm.read(params);
      }
      else
      {
          vm.read(fs.root());
      }
  }
  if(!fs["override"].empty())
  {
     vm.read(fs["override"]);
  }
  return vm;
}


KittiSequenceData::KittiSequenceData(string& season_pattern, Setup& setup)
{
  // настройка логгера
  arlog::init("./log", "./logger-config.yaml");

  // чтение данных камеры
  Camera camera_ = read_cam_calibs(season_pattern, 0);
  // инициализация интрисиков
  full_cam_opt = camera_;

  // ректификакция внутренних параметров камеры в зависимости от внешних
  rectifying = ForwardRectify(full_cam_opt);
  full_cam_opt = rectifying.getForwardCam();
  int offset = 25;
  roi = Rect(offset, offset, full_cam_opt.imageSize.width - 2 * offset,
             full_cam_opt.imageSize.height - 2 * offset);
  full_cam_opt = full_cam_opt.getCameraForSubRoi(roi);

  // инициализация экстринсиков
  auto expose = camera_.exPose;
  ext_R = expose.R * camera_.cameraAxisRemap();
  ext_t = expose.t;


  if(setup.bools["agro_localizer"])
  {
    // todo: REDO hardcode
    vm = read_vehicle_model(vehicle_config);
    ar::thelocalizer::set_thread_mode(ar::thelocalizer::TM_THREAD_UNSAFE);
    agro_loc = ar::thelocalizer::get("agro_loc", ar::LT_VELO_ODO);
    agro_loc->set_vehicle_model(vm);
  }

  if(!setup.bools["is_tram_stand"]) return; // далее идёт поднастройки трамвайного маршрута

  // инициализация сплайна траектории, если таковой имеется
  try{
    path = path.initFromSpline(gt_path_spline);
    the_gmcs::initialize(path.getEnOrigin());
  }catch(...){
    cerr << "Can't init trm spline\n";
  }

  // настроим трамвайный локалайзер
  if(path.getLength() > 0 && setup.bools["localizer_use_height"])
  {
    // настройка модели трамвая
    tvm.read(vehicle_config);

    // инициализация настроек для локалайзера
    ar::thelocalizer::set_thread_mode(ar::thelocalizer::TM_THREAD_UNSAFE);
    trmroutes::initializeRouteManager(trmroutes::RailPathBasedRoute(path),
                                      trmroutes::TramLocPolicy::TramPathLocWithHeight,
                                      tvm);
   loc_with_height = trmroutes::getLoc();
   if(!loc_with_height->initialize_from_rail_path(path))
     cerr << "Localizer-withHeight init FAILED\n";
  }else if(setup.bools["localizer_trk"]) {
    auto loc_trk = ar::thelocalizer::get("trk_loc", ar::LT_GPS_TRACK);
    InputEntry ie = {ar::SeasonEntry(season_pattern, start_ep_algo,
                     finish_ep_algo), gt_path};

    // инциализация локалайзера для использования как GT, если таковые имеются
    if (!loc_trk->initialize_from_trk_list(gen_trk_list(ie, finish_ep_algo)))
      cerr << "Localizer-trk init FAILED\n" << endl;
    else collect_GT_for_drawing_map(loc_trk);
  }
  if(setup.bools["test_localizer"])
  {
    ar::tsv::ObjIStream<Geodetic_data_poles> gd_poles(geodetic_data_poles_path);
    while(gd_poles.has_data())
    {
      Geodetic_data_poles tmp_read; gd_poles >> tmp_read;
      geod_data_poles.emplace_back(std::move(tmp_read));
    }
  }

  if(setup.bools["draw_heights"])
  {
    // инициализация данных геодезистов и Emlid
    ar::tsv::ObjIStream<Geodetic_data> gd(geodetic_data_path);
    ar::tsv::ObjIStream<Stand_emlid_data> ed(emlid_data_path);
    while(gd.has_data())
    {
      Geodetic_data tmp_read; gd >> tmp_read;
      geod_data.emplace_back(std::move(tmp_read));
    }
    while(ed.has_data())
    {
      Stand_emlid_data tmp_read; ed >> tmp_read;
      emlid_data.emplace_back(std::move(tmp_read));
    }

    // будем отрисовывать на графике все высоты со сплайном
    height_figure_draw.grid();
    height_figure_draw.fastMode(true);
    height_figure_draw.ticks();
    height_figure_draw.setXLabel("path, m");
    height_figure_draw.setYLabel("height, m");
    height_figure_draw.legend();
    height_figure_draw.setWindowName("Heights dynamic");

    // считывание сплайна, построенного по данным высот геодезистов
    auto bsp = BSpline1D::initializeWithParams(ar::readHeightSpline(geodetic_height_spline_path).value());
    if(bsp.has_value()){
      geodeitc_height_spline.reserve(10000);
      geodeitc_height_deriv_spline.reserve(10000);
      for (int i = 0; i < 10000; i++) {
        double u = (double) i / (10000 - 1);
        auto x_slope = (*bsp).slopeAt(u, 1.0 / path.getLength()) * 180.0 / M_PI;
        auto x = (*bsp)(u);
        if(x < 100000 && x_slope < 100000){
          geodeitc_height_spline.emplace_back(Point2d(u * path.getLength(), x));
          geodeitc_height_deriv_spline.emplace_back(Point2d(u * path.getLength(), (x_slope + 160)));
        }
      }
      geo_figure_index = height_figure_draw.plot(geodeitc_height_spline, PlotType::LINE, Colors::BLACK, 4, "geodetic_heights_spline");
      geo_deriv_figure_index = height_figure_draw.plot(geodeitc_height_deriv_spline, PlotType::LINE, Colors::PINK, 4, "slope geodetic_heights_spline");
    }

    // построение графика emlid
    transform(emlid_data.begin(), emlid_data.end(), back_inserter(emlid_data_figure), [this](const Stand_emlid_data &data)
    {
      auto now_gm = ENPoint(data.east, data.nord).gm(GMCS(path.getEnOrigin()));
      return Point2d(path.getProjection(Point2d(now_gm.x_gm, now_gm.y_gm)), data.alt);
    });

   sort(emlid_data_figure.begin(), emlid_data_figure.end(), [](const Point2d& lp, const Point2d& rp)
    {
      return lp.x < rp.x;
    });
    emlid_figure_index = height_figure_draw.plot((emlid_data_figure), PlotType::LINE, Colors::YELLOW, 4, "emlid_data");

    // инициализация карты SRTM
    elevation_map.setHeightDataFolder(srtm_map_path);

    now_pos = height_figure_draw.plot({}, PlotType::LINE, Colors::RED, 2, "now position");
  }

  if(setup.bools["draw_and_use_segmentation"])
  {
    drivarea = ar::createSegmenter(dnn_options_path);
    // хранить будем в радианах, но коллекционировать в градусах, чтобы регрессия была более строгой
    obs_yaw_hist = hist_yaw_poles.hist(cv::Mat(), 500, {-40, 40}, false, Colors::GREEN, "poles observation");
    hist_yaw_poles.setWindowName("Poles histogram");
    hist_yaw_poles.setWindowPosition({1920, 540});
    hist_yaw_poles.setXLabel("Angle, [angle]");
    hist_yaw_poles.setYLabel("Normalized quantity");
    hist_yaw_poles.axis(-40, 40, 0.0, 1.0);
    hist_yaw_poles.ticks(true, true, {15, 15}, {4, 2});
    hist_yaw_poles.grid();
    hist_yaw_poles.legend();

    // отрисовка регрессий и кластеров углов
    plt_yaw_clusters.setWindowName("Yaw clusters");
    plt_yaw_clusters.setWindowPosition({1920, 540});
    plt_yaw_clusters.setXLabel("time-stamp, [mc]");
    plt_yaw_clusters.setYLabel("observated yaws");
    plt_yaw_clusters.ticks(true, true, {15, 15}, {4, 2});
    plt_yaw_clusters.axis(-10000, 10000, -40, 40);
    plt_yaw_clusters.grid();
    plt_yaw_clusters.legend();
    size_t max_cluster_size = 10;
    vector<cv::Scalar> colors = {Colors::BLACK, Colors::BLUE, Colors::GREEN, Colors::GOLD, Colors::PINK,
                                 Colors::BROWN, Colors::GRAY, Colors::YELLOW, Colors::SEA_GREEN, Colors::PURPLE};
    for(size_t i = 0; i < max_cluster_size; i++)
    {
      plt_yaw_clusters_index.emplace_back(plt_yaw_clusters.plot({}, PlotType::CIRCLE, colors[i], 4, to_string(i) + " cluster"));
    }
    if(setup.bools["track_poles_as_kalman_filter"])
    {
    }
  }
}

void KittiSequenceData::change_axes_gt(Vec3d& trans_tram)
{
  swap(trans_tram[0], trans_tram[1]);
  trans_tram[1] *= -1;
  trans_tram[0] *= -1;
  swap(trans_tram[0], trans_tram[2]);
}

cv::Matx22d KittiSequenceData::gen_mat_rotation_map(double angle_rot)
{
  Matx22d rot_matrix = Mat(2, 2, CV_8S, Scalar(0));
  rot_matrix(0,0) = cos(angle_rot * CV_PI / 180);
  rot_matrix(0,1) = -sin(angle_rot * CV_PI / 180);
  rot_matrix(1,0) = sin(angle_rot * CV_PI / 180);
  rot_matrix(1,1) = cos(angle_rot * CV_PI / 180);
  return rot_matrix;
}

void KittiSequenceData::reset(){
  local_map = local_map4reset.clone();
}

void KittiSequenceData::feed_localizer(ar::SmartPtr<ar::AShot> shot, std::shared_ptr<ar::ALocalizer> loc, Setup& setup)
{
  // gps
  for(const auto& gpsDeviceName : gpsDeviceNames){
    auto sense = shot->maybeGet<ar::SenseFrame>(gpsDeviceName);
    if(sense != nullptr && loc != nullptr){
      ar::SenseFrame senseData = *sense;
      fixQuality = senseData.fixQuality; // учтём качество GPS-приёмника
      auto gps = ar::GPSStructFromSenseFrame(senseData, &gps_device);

      if(!is_init_start_position){
        loc->init_start_position(gps, 0.0, shot->grabMsec());
        cout << "init localizer start position SUCCESS" << endl;
        is_init_start_position = true;
      }else {
        if(!the_gmcs::get().initialized())
        {
          the_gmcs::initialize(Point2d(gps.east, gps.nord));
        }
        if(prev_trajectory.size() >= 1000) prev_trajectory.pop_front();
        else prev_trajectory.emplace_back(Point2d(gps.east, gps.nord));
        loc->feed_gps_data(senseData.grabMsec(), gps);
      }
    }
  }

  // dbw
  auto dbw = shot->maybeGet<ar::DBWFeedBackFrame>(odoDeviceName);
  if(dbw.isEmpty()) return;
  else
  {
    std::shared_ptr<dbw::IFeedBackData> fbd = dbw->fbd;
    if(fbd == nullptr) return;
    dbw::FeedBackDataWrapper w(fbd.get());
    if(setup.bools["is_tram_stand"]){
      auto fbd_ptr = w.getTramFeedBackData();
      if(fbd_ptr != nullptr)
      {
        double speed_mps = fbd_ptr->m_speeds_values[0];
        ar::OdoStruct odo_data{};
        odo_data.speed = speed_mps;
        loc->feed_odo_data(dbw->grabMsec(), odo_data);
      }
    }else
    {
      ar::OdoStruct odo = ar::OdoStructFromHarvesterFrame(*dbw, &vm);
      loc->feed_odo_data(dbw->grabMsec(), odo);
    }
  }
}

void KittiOdometer::init_run()
{
  string date = strdate();
  string time = strtime();
  cv::datasets::createDirectory(output_folder);
  run_number = 0;
  { ifstream(output_folder + "run_number.txt") >> run_number; }
  { ofstream(output_folder + "run_number.txt") << run_number + 1; }
  output_folder += format("run%04d", run_number);
  cv::datasets::createDirectory(output_folder);
  ofs_report.open(output_folder + format("/report.run%04d.txt", run_number));
}

bool KittiOdometer::init_gt_writer()
{
  ar::TsvWriter::Header header{{"x", ar::Value::FLOAT64}, {"y", ar::Value::FLOAT64}};
  GT_writing.setHeader(header);
  if(GT_writing.open(vo_GT)) return true;
  else return false;
}

bool KittiOdometer::init_gt_reader()
{
  ar::TsvReader GT_reader;
  // отладка полученных из ВО GT-данных
  if(GT_reader.open(vo_GT))
  {
    while (GT_reader.readNextRow())
    {
      points4vis.emplace_back(std::make_pair(GT_reader.current_line_["x"].asDouble(),
                                             GT_reader.current_line_["y"].asDouble()));
    }
    return true;
  }
  else return false;
}

void KittiSequenceData::collect_GT_for_drawing_map(std::shared_ptr<ar::ALocalizer>& loc_trk)
{
	double min_step_m = 0.2; // чтобы не отображать подряд одни и те же точки при стоянии на месте
	int64_t msec_sampling = 100; // отображаем точку каждые 100мс

	// подразумевается что есть инициализированный trk-локалайзер loc.
	int64_t start_ms = loc_trk->get_msec_bounds().first;
	int64_t finish_ms = loc_trk->get_msec_bounds().second;

	// выделим память для отрисовки поз
	poses_gt_drawing.reserve((finish_ms - start_ms) / msec_sampling + 1);
	path_gt_drawing.reserve((finish_ms - start_ms) / msec_sampling + 1);

	Pose poses4drawing;
	Vec3d t_drawing; Matx33d R_drawing;
	int ipng_drawing = 0;
	path_gt_drawing.push_back(0);

	// пройдёмся по всем временным меткам эпизодов, чтобы можно было сделать
	// predict Pose -> сохраним её для первичной отрисовки на local_map
	for(int64_t time_ms = start_ms; time_ms < finish_ms; time_ms += msec_sampling)
	{
		//VR2GM
		poses4drawing = loc_trk->model(time_ms).getPose();

		// проверка, что не стоим долго на месте. В противном случае, значение позы не запоминаем
		if (!poses_gt_drawing.empty() &&
				norm(poses4drawing.t - poses_gt_drawing.back().translation()) < min_step_m) continue;

		t_drawing = poses4drawing.t;
		R_drawing = poses4drawing.R;

		// находим такую НЕНУЛЕВУЮ позу, которую можно будет использовать как отправную точку - (0, 0, 0)
		if(norm(t_drawing) > 0 && isStartPos)
		{
			start_t = t_drawing;
			start_R = R_drawing;
			isStartPos = false;
		}

		// возьмём начало координат транспорта как (0, 0, 0) и вырожденный курс
		t_drawing -= start_t;
		R_drawing =  start_R.inv() * R_drawing;
		change_axes_gt(t_drawing);

		// расчетаем
		Affine3d pose(R_drawing, t_drawing);
		poses_gt_drawing.emplace_back(pose);
		if(ipng_drawing > 1)
		{
			Affine3d& pose0 = poses_gt_drawing[ipng_drawing - 0];
			Affine3d& pose1 = poses_gt_drawing[ipng_drawing - 1];
			double delta = norm(pose0.translation() - pose1.translation());
			double dist = path_gt_drawing.back() + delta;
			path_gt_drawing.push_back(dist);
		}
		ipng_drawing++;
	}
}

KittiOdometer::~KittiOdometer()
{
  ofs_report << "--- setup ---" << endl;
  print_user(ofs_report);
  ofs_report << setup;
  GT_writing.close();
}

void KittiSequenceData::draw_visodometry_points(Mat& src_image, vector<pair<double, double>>& points, const size_t iter)
{
  // TODO: рисовать засечки
  // TODO: сделать ограничение на 50 метров
  // TODO: аккуратнее заполнять таблицу, убрать итерации (временные метки надёжнее)
  if(!points.empty()){
    cv::Vec3d camera_point = {0, 0, 0}; // нулевая координата, от которой идёт отрисовка GT
    if(iter <= points.size()) camera_point = {points[iter].second, points[iter].first, 0};

    for (size_t i = iter; i < points.size(); i++){
      cv::Vec3d path_pt_vr{-points[i].second, points[i].first, 0};
      cv::Point2d path_pt_on_cam = full_cam_opt.projectPt(path_pt_vr - camera_point);
      cv::circle(src_image, path_pt_on_cam, 2, { 255, 255, 0 }, 5);
    }
  }
}

void KittiSequenceData::draw_geodetic_data(Mat& src_image, const ar::ModelPosition& p)
{
  // time -> localaizer -> x, y spline -> x, y geodetic
  vector<pair<Vec3d, Point2f>> close_geod_pts = {};
  pair<Point2d, double> closestEN = make_pair(Point2d(0,0), 1000000);

  for(const auto& i : geod_data)
  {
    /*// берём только те точки, которые недалеко от текущего положения робота
    auto now_gm = ENPoint(i.Easting, i.Northing).gm(GMCS(path.getEnOrigin()));
    double path_on_tram_spline = path.getProjection(Point2d(now_gm.x_gm, now_gm.y_gm), i.path, i.path + 50);
    if(i.path < path_on_tram_spline) continue;
    if(i.path > path_on_tram_spline) break;*/ // долго ищет

    auto gm2vr = p.getPose();
    cv::Point2d en_pt(i.Easting, i.Northing);

    cv::Point2d gm_pt = the_gmcs::get().en2gm2d(en_pt);
    cv::Vec3d path_pt_gm{gm_pt.x, gm_pt.y, i.h_geodetic};
    cv::Vec3d path_pt_vr = gm2vr(path_pt_gm);
    cv::Vec3d pt_on_cam = full_cam_opt.getExPose().relative(path_pt_vr);
    cv::Vec3d norm2dPt = {path_pt_vr[0], path_pt_vr[1], 0};
    double dist2cam = cv::norm(norm2dPt);
    if(dist2cam < 50 && pt_on_cam[1] > 0 && i.dist2spline <= 1)
    {
      close_geod_pts.emplace_back(make_pair(path_pt_vr, Point2f(i.Easting, i.Northing)));
      if(closestEN.second > dist2cam)  closestEN = make_pair(en_pt, dist2cam);
    }
  }
  en_now = closestEN.first;


  if(close_geod_pts.size() < 1) return;

  sort(close_geod_pts.begin(), close_geod_pts.end(), [](const pair<Vec3d, Point2f> &left_pt,
                                                        const pair<Vec3d, Point2f> &righ_pt){
    return left_pt.first[2] < righ_pt.first[2];
  });

  double now_height = close_geod_pts[0].first[2];
  // отрисуем самые ближние и пометим относительную высоту
  for(auto &pts : close_geod_pts)
  {
    auto i = pts.first;
    string height_was = std::to_string(i[2]).substr(0, 6); /* +
        " (" + to_string(pts.second.x).substr(0, 9) + ", " + to_string(pts.second.y).substr(0, 9) + ")";*/
    i[2] -= now_height - 0.5;
    cv::Point2d geo_pt_top = full_cam_opt.projectPt(i);
    cv::Point2d geo_pt_bot = full_cam_opt.projectPt({i[0], i[1], 0});
    cv::line(src_image, geo_pt_top, geo_pt_bot, {0,0,0}, 3); // столбик
    cv::putText(src_image, height_was, geo_pt_top, cv::FONT_HERSHEY_DUPLEX, 0.5, {0, 0, 255});
    cv::circle(src_image, geo_pt_top, 2, {255,0,255}, 3);
    cv::circle(src_image, geo_pt_bot, 2, {255,0,255}, 3);
  }
}

void KittiSequenceData::draw_geodetic_poles_data(Mat &src_image, const ar::ModelPosition &p)
{
  vector<Vec3d> close_geod_pts = {};

  for(const auto& i : geod_data_poles)
  {
    auto gm2vr = p.getPose();
    cv::Point2d en_pt(i.easting, i.northing);

    //cv::Point2d gm_pt = the_gmcs::get().en2gm2d(en_pt);
    cv::Vec3d path_pt_gm{i.x_gm, i.y_gm, i.heght};
    cv::Vec3d path_pt_vr = gm2vr(path_pt_gm);
    cv::Vec3d pt_on_cam = full_cam_opt.getExPose().relative(path_pt_vr);
    cv::Vec3d norm2dPt = {path_pt_vr[0], path_pt_vr[1], 0};
    double dist2cam = cv::norm(norm2dPt);
    if(dist2cam < 50 && pt_on_cam[1] > 0)
    {
      close_geod_pts.emplace_back(path_pt_vr);
    }
  }

  if(close_geod_pts.size() < 1) return;

  sort(close_geod_pts.begin(), close_geod_pts.end(), [](const Vec3d &left_pt,
                                                        const Vec3d &righ_pt){
    return left_pt[2] < righ_pt[2];
  });

  for(auto &pts : close_geod_pts)
  {
    cv::Point2d geo_pole_pt_top = full_cam_opt.projectPt({pts[0], pts[1], 0.5});
    cv::Point2d geo_pole_pt_bot = full_cam_opt.projectPt({pts[0], pts[1], 0});
    cv::line(src_image, geo_pole_pt_top, geo_pole_pt_bot, {0, 255, 0}, 3); // столбик
    cv::circle(src_image, geo_pole_pt_top, 2, {0,0,0}, 3);
    cv::circle(src_image, geo_pole_pt_bot, 2, {0,0,0}, 3);
  }
}

void KittiSequenceData::draw_emlid_data(Mat& src_image, const ar::ModelPosition& p)
{
  // отрисовывем только ближние точки
  vector<Vec3d> close_emlid_pts = {};
  vector<double> close_data_20m;

  // найдём такие данные от емлид, которых находятся в пределе 15 метров + ещё не отрисовывали
  for(size_t i = 0; i < emlid_data.size(); i++)
  {
    auto gm2vr = p.getPose();
    cv::Point2d en_pt(emlid_data[i].east, emlid_data[i].nord);
    cv::Point2d gm_pt = the_gmcs::get().en2gm2d(en_pt);
    cv::Vec3d path_pt_gm{gm_pt.x, gm_pt.y, emlid_data[i].alt};
    cv::Vec3d path_pt_vr = gm2vr(path_pt_gm);
    cv::Vec3d pt_on_cam = full_cam_opt.getExPose().relative(path_pt_vr);

    cv::Vec3d norm2dPt = {path_pt_vr[0], path_pt_vr[1], 0};
    if(cv::norm(norm2dPt) < 10 && pt_on_cam[1] > 0 && emlid_data[i].fixQuality > 0)
    {
      close_emlid_pts.emplace_back(path_pt_vr);
    }
    if(cv::norm(norm2dPt) < 20 && pt_on_cam[1] > 0 && emlid_data[i].fixQuality == 5)
    {
      close_data_20m.emplace_back(path_pt_vr[2]);
    }
  }

  // найдём среднюю высоту, чтобы отрисовывать динамику изменения относительных высот
  auto lambda = [&](double height_1, double height_2){return height_1 + height_2 / close_data_20m.size(); };
  double middle_height = accumulate(close_data_20m.begin(), close_data_20m.end(), 0.0, lambda);// для отрисовки разницы с абсолютной высотой
  if(close_emlid_pts.size() < 1) return;

  // отрисуем все найденный точки
  for(size_t i = 1; i < close_emlid_pts.size(); i++){
    cv::Point2d emlid_pt_top = full_cam_opt.projectPt({close_emlid_pts[i][0], close_emlid_pts[i][1],
                                                       close_emlid_pts[i][2] - middle_height + 0.5});
    cv::Point2d emlid_pt_bot = full_cam_opt.projectPt({close_emlid_pts[i][0], close_emlid_pts[i][1], 0});
    cv::Point2d emlid_prev_pt_bot = full_cam_opt.projectPt({close_emlid_pts[i-1][0], close_emlid_pts[i-1][1], 0});
    cv::line(src_image, emlid_pt_top, emlid_pt_bot, {0,0,0}, 3); // столбик
    cv::line(src_image, emlid_pt_bot, emlid_prev_pt_bot, {0,0,0}, 2);
    cv::putText(src_image, to_string(close_emlid_pts[i][2]).substr(0, 6), emlid_pt_top - Point2d(70, 0), cv::FONT_HERSHEY_DUPLEX, 0.5, {0, 0, 255});
    cv::circle(src_image, emlid_pt_top, 2, {0,255,255}, 3);
    cv::circle(src_image, emlid_pt_bot, 2, {0,255,255}, 3);
  }
}

void KittiSequenceData::draw_height_figure(const ar::ModelPosition& p)
{
  // к этому месту все графики должны быть отрисованы
  // и мы только переставляем границы для локального просмотра
  auto gm2vr = p.getPose();
  cv::Point2d current_pos(gm2vr.getX(), gm2vr.getY());
  double current_path_meter = path.getProjection(current_pos);

  height_figure_draw.axis(current_path_meter - 50, current_path_meter + 50, 140, 170);
  height_figure_draw.getFigureAt<Plot>(geo_figure_index).setPoints(geodeitc_height_spline);
  height_figure_draw.getFigureAt<Plot>(emlid_figure_index).setPoints(emlid_data_figure);
  height_figure_draw.getFigureAt<Plot>(geo_deriv_figure_index).setPoints(geodeitc_height_deriv_spline);
  height_figure_draw.getFigureAt<Plot>(now_pos).setPoints({Point2d(current_path_meter, 0), Point2d(current_path_meter, 1000)});
  height_figure_draw.showWithoutLoop();
}

void KittiSequenceData::draw_railpath_data(cv::Mat& src_image, const ar::ModelPosition& p)
{
  auto gm2vr = p.getPose();
  cv::Point2d current_pos(gm2vr.getX(), gm2vr.getY());
  double current_path_meter = path.getProjection(current_pos);
  drawRailPath(src_image, full_cam_opt, gm2vr, path, current_path_meter, current_path_meter + 30);
}

void KittiSequenceData::draw_tram_trajectory_on_google_map(const ar::ModelPosition& p, GeomapRenderer& gmr, Setup& s,
																													 std::pair<std::vector<Point2d>, std::vector<Point2d> > filtered_poles_gm)

{
	// отрисовка текущего положения
	auto gm2vr = p.getPose();
	cv::Point2d current_pos(gm2vr.getX(), gm2vr.getY());
	if(cv::norm(current_pos) < 0.000001) return;// не прогрелись

	double now_height = 0; //((((*p.getGeodeticHeightsSpline())(path.getParamFromLen(current_path_meter)))))[1];
	auto now_en = the_gmcs::get().gm2en({current_pos.x, current_pos.y, now_height});
	gmr.moveTo(now_en);
	gmr.setZoomStrict(0.2);
	Mat render = gmr.render();

	if(s.bools["is_tram_stand"])
	{
		double current_path_meter = path.getProjection(current_pos);

		optional<Point2d> prev_pnt;
		for(double i = current_path_meter - 150 > 0 ? current_path_meter - 150: 0.0;
			i < current_path_meter + 150; i += 1) // отрисовка каждого метра сплайна
		{
			auto spline_pt = path.getPoint(i);
			auto spline_pt_en = the_gmcs::get().gm2en({spline_pt.x, spline_pt.y, now_height});

			auto ptWnd_spline = gmr.en2wnd().convert(spline_pt_en);
			if(prev_pnt.has_value()) cv::line(render, ptWnd_spline, prev_pnt.value(), {0, 255, 0}, 2);
			else cv::circle(render, ptWnd_spline, 5, {0, 0, 255}, 5);

			prev_pnt = ptWnd_spline;
		}

		// отрисуем весь трамвай
		if(current_path_meter >= 20)
		{
			optional<Point2d> prev_tram_part;
			for(double i =  current_path_meter - 20; i <= current_path_meter; i += 1)
			{
				auto spline_pt = path.getPoint(i);
				auto spline_pt_en = the_gmcs::get().gm2en({spline_pt.x, spline_pt.y, now_height});

				auto ptWnd_spline = gmr.en2wnd().convert(spline_pt_en);
				if(prev_tram_part.has_value()) cv::line(render, ptWnd_spline, prev_tram_part.value(), {255, 100, 50}, 6);

				prev_tram_part = ptWnd_spline;
			}
		}
	}

	// отрисуем все распознанные и отфильтрованные столбы
	if(!filtered_poles_gm.first.empty()){
		for(const auto& pole : filtered_poles_gm.first)
		{
			auto predict_en_pole = the_gmcs::get().gm2en({pole.x, pole.y, 0});
			auto ptWnd_pole = gmr.en2wnd().convert(static_cast<Point2d>(predict_en_pole));
			cv::circle(render, ptWnd_pole, 5, {0, 255, 0}, 5);
		}
		for(const auto& pole : filtered_poles_gm.second)
		{
			auto raw_en_pole = the_gmcs::get().gm2en({pole.x, pole.y, 0});
			auto ptWnd_pole = gmr.en2wnd().convert(static_cast<Point2d>(raw_en_pole));
			cv::circle(render, ptWnd_pole, 5, {0, 0, 255}, 5);
		}
	}
	auto ptWnd = gmr.en2wnd().convert(now_en); // мы

  // отрисуем наше поле видимости
  //Point2d L(cos(-p.getPose().getYaw() + 0.5585054), sin(-p.getPose().getYaw() + 0.5585054));
  //Point2d R(cos(-p.getPose().getYaw() - 0.5585054), sin(-p.getPose().getYaw() - 0.5585054));
  Point2d L(cos(p.getPose().getYaw() + M_PI_2 + 0.5585), sin(p.getPose().getYaw() + M_PI_2 + 0.5585));
  Point2d R(cos(p.getPose().getYaw() + M_PI_2 - 0.5585), sin(p.getPose().getYaw() + M_PI_2 - 0.5585));

	L *= 75;
	R *= 75;
	Point2d LShifted = L + Point2d(p.getPose().getX(), p.getPose().getY());
	Point2d RShifted = R + Point2d(p.getPose().getX(), p.getPose().getY());
	Point2d LShiftedEN = the_gmcs::get().gm2en({LShifted.x, LShifted.y, 0.0});
	Point2d RShiftedEN = the_gmcs::get().gm2en({RShifted.x, RShifted.y, 0.0});
	auto LPtWnd = gmr.en2wnd().convert(LShiftedEN);
	auto RPtWnd = gmr.en2wnd().convert(RShiftedEN);
	cv::line(render, LPtWnd, RPtWnd, {0, 255, 0}, 1);
	cv::line(render, ptWnd, LPtWnd, {255, 0, 0}, 1);
	cv::line(render, ptWnd, RPtWnd, {0, 0, 255}, 1);

	// наш след
	for(const auto& step : prev_trajectory)
	{
		cv::circle(render, gmr.en2wnd().convert(step), 2, {0, 0, 100}, 2);
	}
	cv::circle(render, ptWnd, 6, {0, 0, 255}, 6); // мы

	//putText(render, cv::format("[%1f, %1f]", now_en.x, now_en.y), ptWnd, 1, 1, Scalar(255, 255, 255));
	//putText(render, cv::format("[global_yaw = %1f]", p.getPose().getYaw()), ptWnd, 1, 1, Scalar(255, 255, 255));
	imshow("Google_map", render);
}

double KittiSequenceData::get_poles_reproject_error(Mat &src_image, const ar::ModelPosition &p,
                                                    eobj::Reading<GMObjectContainer> r_objects, double radius)
{
  // todo: разобраться с высотой
  GMPoint2d gm = {p.getPose().getX(), p.getPose().getY()};
  GMPoint2d gm1 = gm - Point2d(radius, radius);
  GMPoint2d gm2 = gm + Point2d(radius, radius);
  ENPoint2d en1 = the_gmcs::get().gm2en({gm1.x, gm1.y, 0});
  ENPoint2d en2 = the_gmcs::get().gm2en({gm2.x, gm2.y, 0});
  ENRect2d enrc(en1, en2); // область-квадрат со сторонами radius

  selected_poles.clear();
  selected_poles_idx.clear();

  // отберём столбы, которые внутри квадрата
  findObjectsInside<AGM_Pole>(r_objects.get(), enrc, selected_poles_idx);
  for (int gmidx : selected_poles_idx)
  {
    auto agm_pole = std::dynamic_pointer_cast<const AGM_Pole>(r_objects.get().obj(gmidx));
    if (agm_pole == nullptr)
      continue;
    ENPoint2d ept = agm_pole->pt(0);
    GMPoint2d gpt = the_gmcs::get().en2gm2d(ept);
    selected_poles.emplace_back(Selected_geomap_pole(gmidx, ept, gpt));
  }

  // для визуализации отрисуем столбы и их сегментацию
  for(const auto& pole : selected_poles)
  {
    bool must_insert = true;
    auto en2vr = the_gmcs::get().en2vr(pole.ept_, p.getPose(), 0);
    cv::Vec3d pt_on_cam = full_cam_opt.getExPose().relative({en2vr.x, en2vr.y, en2vr.z});
    if(pt_on_cam[1] > 0){ // отрисовка только впередистоящих столбов
      for(size_t in_container = max(0, int(poles_geomap_gm_coords.size()) - 5);
          in_container < poles_geomap_gm_coords.size(); in_container++)
      {
        if(cv::norm(poles_geomap_gm_coords[in_container] - pole.gpt_) < 1) must_insert = false;
      }
      if(must_insert)
        poles_geomap_gm_coords.emplace_back(static_cast<Point2d>(pole.gpt_));
      auto pole_bot = full_cam_opt.projectPt(en2vr);
      cv::line(src_image, pole_bot, Point(pole_bot.x, pole_bot.y - 50), {255, 0, 0}, 4);
    }
  }
  return 0;
}

double KittiSequenceData::get_predict_error(const ar::ModelPosition& p, const int64_t &now_time)
{
  return 0;
}

double get_yaw_of_pixel(const Camera& cam, cv::Point2d pixel)
{
  cv::Vec3d direction_of_pixel = cam.reprojectPtWithDist(pixel, 1.0);
  cv::Vec3d vr_dir = direction_of_pixel - cam.getExPose().t;

  return atan2(vr_dir(0), vr_dir(1));
}

double get_pixel_column_of_yaw(const Camera& cam, double yaw)
{
  return cam.principalPoint().x + cam.projection(0, 0) * tan(yaw / 180.0 * M_PI);
}

tuple<Mat, vector<Rect2d>> get_strong_poles(Mat& raw_segmented_poles)
{
  DashMap correct_poles;
  correct_poles.compute(raw_segmented_poles);
  std::vector<cv::Rect2d> poles_bb;
  for(auto& dash : correct_poles.dashes)
  {
    double dash_height = dash.runs.size();
    double dash_thickness = dash.thickness();
    double ratio = dash_thickness / dash_height;
    if ((dash.value == 0) || (dash_height < raw_segmented_poles.rows / 10 || dash_thickness > 50 || ratio > 2.))
    {
      continue;
    }
    Rect bb = dash_bb(dash);
    poles_bb.emplace_back(bb);
  }
  Mat final_poles = Mat::zeros(raw_segmented_poles.rows,
                               raw_segmented_poles.cols,
                               raw_segmented_poles.type());
  for(const auto& bb : poles_bb)
  {
    for(int i = bb.tl().x; i < bb.br().x; i++)
    {
      for(int j = bb.br().y; j > bb.tl().y; j--)
      {
        final_poles.ptr<uchar>(j)[i] = 255;
      }
    }
  }
  return make_tuple(move(final_poles), move(poles_bb));
}

Mat get_diff_between_mats(const Mat& img_1, const Mat& img_2)
{
  //Mat diffImage;
  //cv::absdiff(img_1, img_2, diffImage);
  return img_2 - img_1;
}

int KittiSequenceData::where_is_closer(double yaw_value)
{
  // todo: брать что-то умнее, чем просто мат. ожидание, так как
  // если, например, долго стоим, а потом сдвинулись, то кластер
  // будет очень медленно сдвигаться

  // todo: использовать min, max

  double min_angle_for_close = 3; // todo: подобрать экспериментально
  double min_pixels_dist = 5; // минимальное расстояние между двумя кластерами в пикселях
  int result = -1;
  double how_close = 1000000;

  // середина
  for(size_t ind = 0; ind < yaws_clusters.size(); ind++)
  {
    // условно говорим, что если между мат. ожиданием (~центром) кластера
    // и входным углом не более, чем min_angle_for_close градуса, то эти замеры скорее всего от одного столба
    double maybe_close_1 = abs(yaws_clusters[ind].math_expectation() - yaw_value);
    double maybe_close_2 = abs(yaws_clusters[ind].last_value.y - yaw_value); // для случая простаивания (костыль!)
    if(maybe_close_1 < min_angle_for_close || maybe_close_2 < min_angle_for_close)
    {
      // дальние столбы могут быть очень близки друг к другу,
      // поэтому нужно выбрать самый ближний
      if(maybe_close_1 < how_close || maybe_close_2 < how_close)
      {
        result = ind;
        how_close = maybe_close_1;
      }
    }
    // проверка на то, что между углом и соседним класетром достаточное кол-во пикслей, либо же сливаем в один
    if(yaws_clusters[ind].max_on_frame > 0 && yaws_clusters[ind].min_on_frame < 0){
      auto pix_x = static_cast<int>(get_pixel_column_of_yaw(full_cam_opt, yaw_value));
      if(abs(pix_x - static_cast<int>(get_pixel_column_of_yaw(full_cam_opt, yaws_clusters[ind].min_on_frame))) > min_pixels_dist &&
         abs(pix_x - static_cast<int>(get_pixel_column_of_yaw(full_cam_opt, yaws_clusters[ind].max_on_frame))) > min_angle_for_close)
      {
        result = -1;
        how_close = 1000000;
      }
    }
  }
  return result;
}

void KittiSequenceData::update_yaws_clusters(double yaw_value, int64_t time_stamp)
{
  // попытаемся соотнести угол к кластеру, либо же построим новый кластер
  int closest_cluster_index = where_is_closer(yaw_value);
  if(closest_cluster_index == -1) // == угол далёк от центров кластеров
  {
    int new_index = -1;
    //! @note каждый кадр происходит переиндексирование класетров слева направо
    if(yaws_clusters.size() > 0) new_index = yaws_clusters.back().index + 1;
    else new_index = 0;
    yaws_clusters.emplace_back(Yaws_cluster(new_index));
    yaws_clusters.back().add(yaw_value, time_stamp);
    return;
  }

  // обновим подходящий кластер
  yaws_clusters[closest_cluster_index].add(yaw_value, time_stamp);
}

vector<Point2d> KittiSequenceData::predict_poles_yaws(const int64_t& time_stamp)
{
  // TODO: задать max_path_len + удалять кластеры из дэка кластера аккуратно
  // TODO: посократить кол-во данных в кластерах
  // TODO: линейной регрессии очень мало (либо нужно брать полином > 1 степени, либо ограничивать окно)
  // TODO: брать последние N значений
  // TODO: если столб далеко, то делать регрессию чуть круче
  vector<Point2d> predicted_yaws_for_clusters;
  for(auto& cluster : yaws_clusters)
  {
    // reg: k10*x + k01*y + k00 = 0
    auto last_N_elements = next(cluster.pinched_yaws.begin(), std::max(0, // берём только последние N (30) кадров
                                                                       static_cast<int>(cluster.pinched_yaws.size()) - 1000));
    auto part_of_cluster = deque<Point2d>{last_N_elements, cluster.pinched_yaws.end()};
    auto reg = regress_to_line2d(part_of_cluster);
    double predict_yaw = (-reg.k00 - reg.k10 * time_stamp) / reg.k01;
    predicted_yaws_for_clusters.emplace_back(Point2d(time_stamp, predict_yaw));
  }
  return predicted_yaws_for_clusters;
}

pair<vector<Point2d>, vector<Point2d>>
KittiSequenceData::process_poles(const Mat& image, Mat& forward_visual, const ar::ModelPosition& p,
                                 const int64_t& time_stamp, Setup& setup,
                                 ar::MultiObjectFilter<ar::SingleObjectFilterRPhiFixedWithPoorGPS>& pole_tracker)
{
  // получим сегментацию столбов
  std::vector<std::vector<cv::Mat>> result_mask;
  drivarea->processAsyncStart(image);
  drivarea->processMulti(image, result_mask);
  Mat raw_poles_segment = move(result_mask[0][3]); // сегментация без постпроцессинга

  // подготовка столбов к постобработке
  raw_poles_segment.convertTo(raw_poles_segment, CV_8U);
  raw_poles_segment *= 255;
  // выделим bbox и дозальём "сырую" сегментацию до bbox
  auto [poles_segment, bound_rect] = get_strong_poles(raw_poles_segment);

	// сделаем мэпинг из пиксельного пространства в yaw => получим кластеры столбов + вероятную нижнюю точку столба
	obs_yaws.clear();
	for (int j = 0; j < poles_segment.cols; j++)
	{
		for (int i = 0; i < poles_segment.rows; i++)
		{
			int pix = poles_segment.ptr<uchar>(i)[j];
			if (pix > 0) // берём пиксели только от столбов
			{
				Point2d pixel(j, i);
				// отображение пикселя в yaw (в грудасах)
				double yaw = get_yaw_of_pixel(full_cam_opt, pixel) * 180.0 / M_PI;
				update_yaws_clusters(yaw, time_stamp); // для формирования кластеров
				obs_yaws.emplace_back(yaw); // для отрисовки гистограммы
			}
		}
	}
	for(size_t cluster_ind = 0; cluster_ind < yaws_clusters.size(); cluster_ind++)
	{
		yaws_clusters[cluster_ind].index = cluster_ind;
	}

	// присвоим каждому кластеру условную нижнюю точку столба и его высоту
	// для начала отсортируем
	sort(bound_rect.begin(), bound_rect.end(), [](const Rect2d& l_box, const Rect2d& r_box)
	{
		return l_box.tl().x < r_box.tl().x;
	});
	if(bound_rect.size() == yaws_clusters.size())
	{
		for(size_t cluster_ind = 0; cluster_ind < yaws_clusters.size(); cluster_ind++)
		{
			yaws_clusters[cluster_ind].height = abs(bound_rect[cluster_ind].tl().y - bound_rect[cluster_ind].br().y);
			yaws_clusters[cluster_ind].width = abs(bound_rect[cluster_ind].tl().x - bound_rect[cluster_ind].br().x);
			yaws_clusters[cluster_ind].mid_on_bot = Point2d(bound_rect[cluster_ind].br().x - bound_rect[cluster_ind].tl().x,
																											bound_rect[cluster_ind].br().y);

		}
	}else
	{
		// TODO: смёрджить bbox и кластеры, получаемые из сегментации
	}

  size_t index = 0;
  pair<vector<Point2d>, vector<Point2d>> filtered_poles;  // predict and measurements
  // предскажем положение всех видимых столбов на случай их заслонения
  if(setup.bools["track_poles_as_line_reg"]){
    ///////////////////////////////// REG-FILTER ////////////////////////////////////////
    auto prob_poles_yaws = predict_poles_yaws(time_stamp);
    for(size_t pred_cluster_id = 0; pred_cluster_id < yaws_clusters.size(); pred_cluster_id++){
      // постарались предсказать следующее положение кластера и если столб
      // не был отсегментирован на кадре, то испольуем предсказание
      yaws_clusters[pred_cluster_id].predicted_last_value = prob_poles_yaws[pred_cluster_id];
    }

    // отсортируем по yaw слева-направо [-40, ..., 40] и переиндексируем кластеры углов в порядке возрастания, чтобы быстро потом удалять
    // крайние, выходящие из области видимости
    sort(yaws_clusters.begin(), yaws_clusters.end(), [](Yaws_cluster& l, Yaws_cluster& r)
    {
       return l.math_expectation() < r.math_expectation();
    });
    for(size_t iter = 0; iter < yaws_clusters.size(); iter++) yaws_clusters[iter].index = iter;

    vector<size_t> indexes_to_delete_clusters; // соберём индексы кластеров, которые вышли из области видимости
    for(size_t i = 0; i < yaws_clusters.size(); i++)
    {
      // возьмём среднее (по среднему) кластеру и последнему значению (мб экспофильтр?)
      // todo: мерджить среднее по кластерку, которое мб не объективным и последнее значение, попавшее в кластер
      if(yaws_clusters[i].last_value.x < time_stamp && abs(yaws_clusters[i].last_value.y) < 30)
      {
        yaws_clusters[i].add_as_pinched_yaw(yaws_clusters[i].predicted_last_value);
        yaws_clusters[i].last_value = yaws_clusters[i].predicted_last_value;
        cout << cv::format("cluster #%i hasn't new value => let's predict...", i).c_str();
      }
      else {
        yaws_clusters[i].add_as_pinched_yaw(Point2d(yaws_clusters[i].yaws_collection[0].x,
                                            (/*yaws_clusters[i].math_expectation()*/ + yaws_clusters[i].last_value.y)));
      }


      // избавление от столба, который уже вышел за камеру
      if(abs(yaws_clusters[i].pinched_yaws.back().y) > 30 || // общее ограничение по градусам
         abs(get_pixel_column_of_yaw(full_cam_opt, yaws_clusters[i].last_value.y) - raw_poles_segment.cols) < 10 ||
         abs(get_pixel_column_of_yaw(full_cam_opt, yaws_clusters[i].last_value.y)) < 10
         ) // ограничением по пикселям от границ
      {
        indexes_to_delete_clusters.emplace_back(i);
      }

      cout << cv::format("cluster #%i has yaw = %.4f", i, yaws_clusters[i].pinched_yaws.back().y).c_str() << endl;
      // зачистим все "сырые" углы кластера, так как они больше не нужны
      yaws_clusters[i].yaws_collection.clear();

    }cout << endl;

    // удалим кластеры, которые вышли из области видимости
    size_t shift = 0;
    for(size_t iter = 0; iter < indexes_to_delete_clusters.size(); iter++)
    {
      yaws_clusters.erase(yaws_clusters.begin() + indexes_to_delete_clusters[iter] + shift++);
    }
    // отрисуем валидные кластеры, предварительно сбросив все графики
    for(size_t plt = 0; plt < 10; plt++)
      plt_yaw_clusters.getFigureAt<Plot>(plt_yaw_clusters_index[plt]).setPoints({});
    for(size_t i = 0; i < yaws_clusters.size() && i < plt_yaw_clusters_index.size(); i++)
    {
     plt_yaw_clusters.getFigureAt<Plot>(plt_yaw_clusters_index[yaws_clusters[i].index]).setPoints({yaws_clusters[i].pinched_yaws.begin(),
                                                                              yaws_clusters[i].pinched_yaws.end()});
    }

    // смерджим кластеры, которые могли случайно разъединиться из-за выбросов и других проблем (дерево как столб)
    // todo: сделать нормально
    for(size_t i = 1; i < yaws_clusters.size() && yaws_clusters.size() > 1; i++)
    {
      if(abs(yaws_clusters[i].last_value.y - yaws_clusters[i-1].last_value.y) < 3)
      {
        if(yaws_clusters[i].den > yaws_clusters[i-1].den) // кластер выбросов будет иметь меньше углов
        {
          yaws_clusters[i].num += yaws_clusters[i-1].num;
          yaws_clusters[i].den += yaws_clusters[i-1].den;
          yaws_clusters.erase(yaws_clusters.begin() + i - 1);

        }else{
          yaws_clusters[i-1].num += yaws_clusters[i].num;
          yaws_clusters[i-1].den += yaws_clusters[i].den;
          yaws_clusters.erase(yaws_clusters.begin() + i);
        }
      }
    }
    for(size_t iter = 0; iter < yaws_clusters.size(); iter++) yaws_clusters[iter].index = iter;

    // удалим кластеры, котоные не обновлялись 3 секунда (todo: fit)
    /*for(size_t i = 0; i < yaws_clusters.size(); i++)
    {
      if(yaws_clusters[i].yaws_collection.back().x > 3000)
      {
        yaws_clusters.erase(yaws_clusters.begin() + i);
        i--;
      }
    }*/

    // построим гистограмму по всем пикселям, переведённым в yaw
    auto obs_hist_mat = mth::createHistogram(obs_yaws, 500, {-40, 40}, true);
    hist_yaw_poles.getFigureAt<Hist>(obs_yaw_hist).setHistMat(obs_hist_mat);

    // построим собранные кластеры во временном ряду на 10 секунд назад в вперёд
    plt_yaw_clusters.axis(time_stamp - 10000, time_stamp + 10000, -40, 40);
    plt_yaw_clusters.showWithoutLoop();
    hist_yaw_poles.showWithoutLoop();
    // отрисуем все кластеры столбов, в том числе и предиктнутые
    for(const auto& cluster : yaws_clusters){
      double pix_x = get_pixel_column_of_yaw(full_cam_opt, cluster.last_value.y);
      line(forward_visual, Point(pix_x, 0), Point(pix_x, forward_visual.rows), {0, 0, 255}, 3);
    }
  }  ///////////////////////////////// KALMAN-FILTER ///////////////////////////////////
  else if(setup.bools["track_poles_as_kalman_filter"])
  {
    if(fixQuality > 1){ // примерное условие, когда локалайзер валидный
      vector<ar::MeasurementPoint> meas_pnt;
      for(auto& rect : bound_rect)
      {
        ar::MeasurementPoint pnt;
        pnt.height_pix = abs(rect.br().y - rect.tl().y);
        pnt.id = index++; // dEBUG
        pnt.input_id = pnt.id;
        pnt.msec = time_stamp;
        pnt.phi_vr = get_yaw_of_pixel(full_cam_opt, Point2d((rect.br().x + rect.tl().x) / 2, rect.br().y));

				auto projected_pole_bot = full_cam_opt.reprojectPtWithHeight(Point2d((rect.br().x + rect.tl().x) / 2, rect.br().y), 0.0);
				pnt.r_vr = cv::norm(projected_pole_bot);
				pnt.width_pix = abs(rect.br().x - rect.tl().x);
				pnt.fixQuality = fixQuality;

				//Point3d gm_poles = the_gmcs::get().vr2gm(Point3d(projected_pole_bot.x, projected_pole_bot.y, 0.0), p.getPose());
				pnt.pt_gm = {p.getPose().getX(), p.getPose().getY()};
				pnt.yaw_gm = p.getPose().getYaw();


				double alpha = pnt.yaw_gm + M_PI_2 - pnt.phi_vr;
				Point2d poleRot(pnt.r_vr * cos(alpha), pnt.r_vr * sin(alpha));
				Point2d finalPoleGM(poleRot.x + pnt.pt_gm.x, poleRot.y + pnt.pt_gm.y);


				//double x = pnt.pt_gm.x + pnt.r_vr * cos(pnt.phi_vr + pnt.yaw_gm); // GM-object
				//double y = pnt.pt_gm.y + pnt.r_vr * sin(pnt.phi_vr + pnt.yaw_gm);

				//auto gmPole = the_gmcs::get().vr2gm(projected_pole_bot, p.getPose());
				pnt.obj_gm = {finalPoleGM.x, finalPoleGM.y};
				meas_pnt.emplace_back(pnt);

				// save raw poles
				filtered_poles.second.emplace_back(pnt.obj_gm);
			}
			auto res = pole_tracker.processTracking(meas_pnt);
			for(const auto& gotten_pole_predict : res) // predicted poles
			{
				Pose pred_pose = gotten_pole_predict.pose;
				filtered_poles.first.emplace_back(Point2d(pred_pose.getX(), pred_pose.getY()));
			}
		}
	}

  // отрисуем столбы на forward-view изображении
  Mat poles_brg;
  cvtColor(poles_segment, poles_brg, COLOR_GRAY2BGR);
  for(size_t idx = 0; idx < bound_rect.size(); idx++)
  {
    {
      // сохраним EN координаты столбов как от сегментации, так и от их гугл-карт
      //circle(not_clear_image, bound_rect[idx].br(), 5, {255, 0, 0}, 5);
      //auto pt_to_pole = full_cam_opt.reprojectPtWithHeight(bound_rect[idx].br());

      //if(pt_to_pole.y <= 20 && pt_to_pole.y > 0)
      {
        rectangle(forward_visual, bound_rect[idx].tl(), bound_rect[idx].br(), {0, 255, 0}, 2);
        //auto gm_segm_pole_3d = the_gmcs::get().vr2gm(pt_to_pole, p.getPose());
        //Point2d gm_segm_pole(gm_segm_pole_3d.x, gm_segm_pole_3d.y);

        double pole_yaw_bot = get_yaw_of_pixel(full_cam_opt, Point2d((bound_rect[idx].br().x +
                                                             bound_rect[idx].tl().x) / 2, bound_rect[idx].br().y)) * 180.0 / M_PI;

        //double pole_yaw_top = get_yaw_of_pixel(full_cam_opt, Point2d((bound_rect[idx].br().x +
        //                                                     bound_rect[idx].tl().x) / 2, bound_rect[idx].tl().y)) * 180.0 / M_PI;

        putText(forward_visual, format("[yaw = %.1f  yaw_global = %.1f]", pole_yaw_bot, p.getPose().getYaw() * 180.0 / M_PI), bound_rect[idx].br(), 1, 1, {0, 255, 0}, 2);

        //putText(forward_visual, format("[yaw = %.1f]", pole_yaw_top),
        //                                bound_rect[idx].tl(), 1, 1, {0, 255, 0}, 2);

        /*// todo: сделать условие нового столбы жёстче
        bool must_insert = true;
        for(size_t in_container = max(0, int(poles_gm_coords.size()) - 5);
            in_container < poles_gm_coords.size(); in_container++)
        {
          if(cv::norm(poles_gm_coords[in_container][0] - gm_segm_pole) < 5)
          {
            poles_gm_coords[in_container].emplace_back(gm_segm_pole);
            must_insert = false;
          }
        }
        if(must_insert) // новый столб
          poles_gm_coords.emplace_back(vector<Point2d>({static_cast<Point2d>(gm_segm_pole)}));*/
      }
    }
  }
  return filtered_poles;
}

int KittiOdometer::process_avi(ar::SeasonShooter& season_shooter, string &season_pattern)
{
  Timer timer(__FUNCTION__);
  ksdata = std::move(KittiSequenceData(season_pattern, setup));

  // преднастройка Шотов
  ar::SmartPtr<ar::AShot> shot;
  ar::ImageFrame* ifr_le = nullptr;
  ar::TMapTypeFrames shot_frames = {};
  ksdata.curr_ep_algo = season_shooter.getCurrentEpisode(); // эпизод, по которому получаем VO
  ksdata.start_ep_algo = season_shooter.getStartEpisode();
  ksdata.finish_ep_algo = season_shooter.getFinishEpisode();

  // инициализация отписывания GT (пригодится для обучения сеток для сегментации ЖД путей)
  if(setup.bools["write_GT_as_XYcoord"]){
    if(!init_gt_reader()) cerr << "can't read GT VO previous running\n";
    if(!init_gt_writer()) cerr << "can't init GT writer\n";
  }

  // инициализация GeoMap
  GeomapRenderer gmr;
  gmr.loadMap(ksdata.geomap_objects_path);
  const auto& geomap_objects = (gmr.geomap())->objects;
  eobj::Reading<GMObjectContainer> r_objects(geomap_objects);
  ar::MultiObjectFilter<ar::SingleObjectFilterRPhiFixedWithPoorGPS> pole_tracker_kalman;
  if(setup.bools["draw_google_map"])
  {
    gmr.loadMap(ksdata.google_map_path);
    gmr.setDestImageSize({600, 600});
  }

  // будем использовать такой-то локалайзер
  std::shared_ptr<ar::ALocalizer> cur_loc;
  if(setup.bools["localizer_use_height"]) cur_loc = ksdata.loc_with_height;
  if(setup.bools["agro_localizer"]) cur_loc = ksdata.agro_loc;

  // итерация по шотам
  while (!(shot = season_shooter.shoot()).isEmpty())
  {
    ksdata.seq_num = season_shooter.getCurrentEpisode();
    // собрали шот
    shot_frames = shot->inputFrames;
    if(shot_frames.find("central60") != shot_frames.end() ||
       shot_frames.find("leftImage") != shot_frames.end())
    {
      // пока используем central60, либо leftImage
      auto frame_le = shot_frames["central60"];
      if(frame_le.isEmpty()) frame_le = shot_frames["leftImage"];

      // вычленим из шота изображения - левое и правое (болванчик)
      ifr_le = frame_le.dynamicCast<ar::ImageFrame>();
      Mat3b left_image = ksdata.rectifying.process(ifr_le->src1, cv::INTER_NEAREST);
      left_image = left_image(ksdata.roi);

      if (left_image.empty()) {
        cout << "left-image EMPTY" << endl;
        return 1;
      }

      // дадим актуальную информацию локалайзеру
      ksdata.feed_localizer(shot, cur_loc, setup);

      // временная метка изображения, полученного из шота
      auto msec_frame = ifr_le->grabMsec();

      // узнаем положение локалайзера на текущий момент
      auto cur_loc_ans = cur_loc != nullptr ? cur_loc->model(msec_frame) :
                                              ar::ModelPosition();

      Mat left4draw_map = left_image.clone(); // forward_view для различных отрисовок

      if(setup.bools["is_tram_stand"]){
        // отрисуем рельсы
        ksdata.draw_railpath_data(left4draw_map, cur_loc_ans);
      }

      if(setup.bools["test_localizer"])
      {
        //ksdata.draw_geodetic_poles_data(left4draw_map, cur_loc_ans);
        // ошибку проекции карты-в-кадр можно сделать, например, на основе столбов и
        // сегментации через нахождение минимального суммарного расстояния между центрами столбов по ДП

        //Mat segmantation = left_image.clone();
        //auto filtered_poles = ksdata.process_poles(segmantation, left4draw_map, cur_loc_ans, msec_frame, setup, pole_tracker_kalman);
        //ksdata.get_poles_reproject_error(left4draw_map, cur_loc_ans, r_objects, 20 /* радиус в метрах */);
        //ksdata.get_predict_error(cur_loc_ans, msec_frame);

        if(setup.bools["draw_heights"] && cur_loc != nullptr){
          // отрисуем столбцы геодезических замеров с нормализованной высотой
          ksdata.draw_geodetic_data(left4draw_map, cur_loc_ans);

          // отрисуем столбцы Emlid устройства
          ksdata.draw_emlid_data(left4draw_map, cur_loc_ans);

          // отрисуем график высот
          ksdata.draw_height_figure(cur_loc_ans);
        }
        if(setup.bools["draw_google_map"]) {
          ksdata.draw_tram_trajectory_on_google_map(cur_loc_ans, gmr, setup);
        }
      }

			if(setup.bools["draw_XYcoord_prev_start"])
			{
				Mat prev_start_odometr = left_image.clone();
				ksdata.draw_visodometry_points(prev_start_odometr, points4vis, iter_prev_run++);
			}

			if(!setup.bools["only_play_stand"]){
				// делаем предикт позы локалайзера по временной метке
				auto pose_ = ksdata.loc_trk->model(msec_frame).getPose();
				Vec3d& t = pose_.t;
				Matx33d& R = pose_.R;

				if(ksdata.isStartPos && norm(t) > 0)
				{
					// синхронизация локалайзера и одометра
					ksdata.start_t = t;
					ksdata.start_R = R;
					ksdata.isStartPos = false;
				}
				if(ksdata.isStartAngle)
				{
					ksdata.start_pitch_gt = ksdata.loc_trk->pose(msec_frame).getPitch();
					ksdata.start_roll_gt = ksdata.loc_trk->pose(msec_frame).getRoll();
					ksdata.start_yaw_gt = ksdata.loc_trk->pose(msec_frame).getYaw();
					ksdata.isStartAngle = false;
				}

				// возьмём начало координат транспорта как (0, 0, 0) и вырожденный курс
				t -= ksdata.start_t;
				R =  R * ksdata.start_R.inv();
				ksdata.change_axes_gt(t);

				// собираем аффинное преобразование из вращения и сдвига
				Affine3d pose(R, t);

				// проверяем, что вектора сдвига (из локалайзера) изначально околонулевой и положительный
				if(abs(pose.translation()[2]) > 100 && ksdata.ipng < 5){
					pose = Affine3d();
				}

				// заполняем вектор поз из локалайзера
				ksdata.poses_gt.emplace_back(pose);

				// ищем норму сдвига между кадрами, чтобы можно отрисовать всю карту
				if(ksdata.ipng > 0)
				{
					Affine3d& pose0 = ksdata.poses_gt[ksdata.ipng - 0];
					Affine3d& pose1 = ksdata.poses_gt[ksdata.ipng - 1];
					double delta = norm(pose0.translation() - pose1.translation());
					double dist = ksdata.path_gt.back() + delta;
					ksdata.path_gt.emplace_back(dist);
				}

				Snapshot ss(ksdata.ipng,
										ifr_le->grabMsec(),
										ksdata.full_cam_opt,
										ksdata.poses_gt[ksdata.ipng],
										season_shooter.getCurrentEpisode(),
										R, t, left_image,
										(ksdata.loc_trk->pose(msec_frame).getYaw() - ksdata.start_yaw_gt) * 180. / CV_PI,
										(ksdata.loc_trk->pose(msec_frame).getRoll() - ksdata.start_roll_gt) * 180. / CV_PI,
										(ksdata.loc_trk->pose(msec_frame).getPitch() - ksdata.start_pitch_gt) * 180. / CV_PI);
				ksdata.ipng++;
				ksdata.snapshots.emplace_front(ss);

				// обработка шота и получение хар-ик смещения, поворота
				process_snapshot(ss);
			}else ksdata.ipng++;

     // отрисовка окон камеры
     imshow("Forward View", left4draw_map);

			// обработка клавиш
			int key = waitKey(setup.ints["delay"]);
			if(setup.bools["iter_by_image"])
			{
				while(true)
				{
					key = waitKey(setup.ints["delay"]);
					if(key == key_Enter) break;
					if(key == 27) return 0;
					if(key == 32) {
						setup.bools["iter_by_image"] = !setup.bools["iter_by_image"];
						break;
					}
				}
			}
			if (key == 32) // <space>
			{
				while(true)
				{
					key = waitKey(setup.ints["delay"]);
					if(key == key_Enter){
						setup.bools["iter_by_image"] = !setup.bools["iter_by_image"];
						break;
					}
					if(key == 32) break;
					if(key == 27) return 0;
				}
			}
			if(key == 110) /* <n> */season_shooter.forceNextEpisode();
			if (key == 27) /* <esc >*/ break;
     }

     if(season_shooter.getCurrentEpisode() != ksdata.curr_ep_algo && false && setup.bools["write_statistics"])
     {
       // формирование отчёта
       ksdata.curr_ep_algo = season_shooter.getCurrentEpisode();
       evaluate_poses_final(output_folder, setup);
       if (ksdata.report.step_num > 0)
       {
         ksdata.report.duration = timer.msec();
         seq_num2report[ksdata.curr_ep_algo] = ksdata.report;
         ksdata.ofs_output << "Processing speed (msec per frame): \t" <<
                              ksdata.report.duration / (ksdata.report.step_num) << endl;
       }
       ksdata.speedmap.write( output_folder, ksdata.seq_num );
     }
  }
  return 0;
}
