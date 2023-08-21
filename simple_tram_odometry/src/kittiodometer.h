// этот файл в utf-8
//! @file kittiodometer.h
//! @brief набор функций для рассчёта визуальной одометрии
//! @author Арсентьев А.
//! @date 2020

#pragma once
#include <fstream>
#include <map>

#include <string>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/datasets/util.hpp>

//#include <ar10/src/svnversion.h>
#include <localization/localizer.h>
#include <ar10/idx.h>
#include <ar10/smartptr.h>
#include <ar10/testdata.h>
#include <ar10/filesystem.h>
#include <arcore/offlineshooter.h>
#include <coords/device.h>
#include <warping/warping.h>
#include <ar10/tsvio.h>
#include <arcore/imageframe.h>
#include <uplotlib/uplotlib.h>
#include <utility>
#include <geomap/geomap_renderer.h>
#include <geomap/elevation.h>
#include <semanticsegm/src/drivareasegmenter.h>
#include <arml/mxnetplayer.h>
#include <popcorn4/src/dema.h>
#include <roadobjectpool/gm_object_filter.h>

typedef std::tuple<std::vector<cv::Vec2f>, std::vector<cv::Vec2f>,
                    std::vector<cv::Vec2f>, std::vector<cv::Vec2f>> Coor_points;

#include "snapshot.h"

//! @brief базовая структура для расчёта результатов прогона
//! каждая величина считается как отношение суммы всех ошибок кинематических хар-ик
//! к количеству кадров прогон
struct KittiSequenceReport
{
  double step_num = 0.; //!<  число переходов. double для мягкости в случае нулевых

  /// результаты и статистика по прогону
  double speed_err_sum = 0; //!< сумма относительных отклонений длины вектора покадрового смещения (== скорости) от граунтруса
  double rotat_err_sum = 0; //!< сумма абсолютных отклонений угла поворота в градусах
  double angle_dif_sum = 0.; //!< суммарное отклонение знаковых ошибок угла поворота в градусах
  double angle_tot_err = 0.; //!< суммарное отклонение угла поворота в градусах
  double duration = 0.; //!<  время обработки сиквенса в миллисекундах
};

//! @brief структура KITTI (A.Geiger) для расчёта ошибок результатов прогона
//! каждая величина считается по окну в [100, 200, ..., 800].
struct KittiErrors {

  //! @brief тривиальный конструктор
  KittiErrors() = default;

  //! @brief конструктор
  //! @param[in] first_frame номер первого изображения определённого окна
  //! @param[in] r_err ошибка поворота на определённом шаге
  //! @param[in] t_err ошибка сдвига на определённом шаге
  //! @param[in] len длина окна
  //! @param[in] speed скосрость
  KittiErrors(int32_t& first_frame, float& r_err, float& t_err, float& len, float& speed) :
    first_frame(first_frame), r_err(r_err), t_err(t_err), len(len), speed(speed){}

  //! @brief конструктор c инициализацией только первого кадра
  //! @param[in] first_frame номер первого изображения определённого окна
  KittiErrors(int32_t first_frame) :
    first_frame(first_frame){}

  int32_t first_frame = 0; //!< первый кадр последовательности
  int32_t num_steps = 0; //!< расширение -- число ребер на цепочке. фиксирует диапазон расчета.
  float   r_err = 0.; //!< на шаге окна. в конце нормируется на длину окна len (а не реальную длину gt-цепочки!!!)
  float   t_err = 0.; //!< на шаге окна. в конце нормируется на длину окна len (а не реальную длину gt-цепочки!!!)
  float   len = 0; //!< по схеме использования тут значения РОВНО(!)= [100, 200, ... 800]. а не реальные длины цепочек :(
  float   gt_len = 0; //!< настоящая длина последовательности по GT
  float   speed = 0.; //!< пока не используем
};

//! @brief структура KITTI (A.Geiger) для апдейта статистики проезда по окну
//! [1,...,8] содержат статистику по окнам 100..800, [0] содержит суммарную статистику
struct KittiOdometryStatistics
{
  KittiErrors windows[9]; //!< индексы окон [100, 200, ..., 800] метров которые отстают но обновляются по мере сдвига конца расчитанного участка
  double alpha_sum[9] = { 0. }; //!< суммарная ошибка alpha = path_error/sqrt(path)
  double t_err_sum[9] = { 0. }; //!< суммарная ошибка трансляции по окнам
  double r_err_sum[9] = { 0. }; //!< суммарная ошибка поворота по окнам
  int num_err[9] = { 0 }; //!< количество просуммированных ошибок

  double t_err_common = 0; //!< финальная ошибка по смещению за весь эпизод - оценка невязки
  double r_err_common = 0; //!< финальная ошибка по повороту за весь эпизод - оценка невязки
};


//! @brief структура для формирования шаблона файлов, подаваемые на вход локалайзеру
struct InputEntry
{
  ar::SeasonEntry season_entry;
  std::string trk_file;
  bool operator==(const InputEntry &rhs){
      return trk_file == rhs.trk_file;
  }
};

//! @brief данные геодезистов по высотам
struct Geodetic_data
{
  double path;
  double h_geodetic;
  double h_srtm;
  double x;
  double y;
  std::string name;
  double Easting;
  double Northing;
  double dist2spline;
};

struct Geodetic_data_poles
{
  double heght;
  double easting;
  double northing;
  double x_gm;
  double y_gm;
};

//! @brief данные с Emlid
struct Stand_emlid_data
{
  long long grabMsec;
  double nord;
  double east;
  double speed;
  double alt;
  double fixQuality;
};

//! @brief данные с SRTM
struct STRM_data
{
  long long grabMsec;
  double nord;
  double east;
  double alt;
  double path;
};

//! @brief ключевая информация по AGM_Pole, выгруженная из геомапа для быстрого итерирования
struct Selected_geomap_pole {
  Selected_geomap_pole( int agm_index, const ENPoint2d& ept, const GMPoint2d& gpt):
    agm_index_(agm_index),
    ept_(ept), gpt_(gpt) {}

  ENPoint2d ept_; // центр столба в EN координатах
  GMPoint2d gpt_; // центр столба в GM координатах
  int agm_index_ = -1;
};

//! @brief кластеры углов одного столба
//! @note берётся все сегментация столба и каждый пиксель переводится
//! в угол, после чего имеем нормальное распределение угла. Чем ближе столб
//! тем "шире" распределение
struct Yaws_cluster{
  Yaws_cluster(int index){ this->index = index; }
  double num = 0; // для расчёта среднего
  double den = 0;
  double height = 0.0; // в пикселях
  double width = 0.0; // в пикселях
  cv::Point2d mid_on_bot; // нижняя точка середины столба для репроекции из пикселей в GM

  DEMA smoothed_pinched_yaws{0.9, 0.1}; // будем сглаживать получаемые углы, чтобы не было скачков резких

  std::deque<cv::Point2d> yaws_collection;
  std::deque<cv::Point2d> pinched_yaws; //! для регрессии, где x - время, y - угол

  cv::Point2d min = cv::Point2d(-1, 90);
  cv::Point2d max = cv::Point2d(-1, -90);

  double min_on_frame = 90;
  double max_on_frame = -90;

  cv::Point2d last_value = cv::Point2d(-1, -90);
  cv::Point2d predicted_last_value = {-1, -1};

  double math_expectation(){ return num / den; }
  void add(double yaw_value, int64_t time_stamp){
    num += yaw_value;
    den += 1;
    yaws_collection.emplace_back(cv::Point2d(time_stamp, yaw_value));

    // будем хранить также mix, min, last_value
    if(yaw_value > max.y) max = cv::Point2d(time_stamp, yaw_value);
    if(yaw_value <= min.y) min = cv::Point2d(time_stamp, yaw_value);

    if(yaw_value > max_on_frame) max_on_frame = yaw_value;
    if(yaw_value <= min_on_frame) min_on_frame = yaw_value;

    last_value = cv::Point2d(time_stamp, yaw_value);
  }

  void add_as_pinched_yaw(const cv::Point2d& last_value)
  {
    smoothed_pinched_yaws.update(last_value.y);
    pinched_yaws.emplace_back(cv::Point2d(last_value.x, smoothed_pinched_yaws.value()));
  }

  void reset(){
    num = 0;
    den = 0;
    if(!yaws_collection.empty()) yaws_collection.clear();
    if(!pinched_yaws.empty()) pinched_yaws.clear();
  }
  int index;
};

//! @brief данные по прогону одного sequence или его фрагмента
struct KittiSequenceData // данные по прогону одного sequence или его фрагмента
{
	//! @brief тривиальный конструктор
	KittiSequenceData() = default;

	//! @brief констуктор для инициализации интрисиков и экстринсиков.
	//! @param [in] season_pattern шаблон сезона
	KittiSequenceData(std::string &season_pattern, Setup &setup);

	//! @brief обновление poses_re[] + отрисовка на первом кадре карту GT + отрисовка полученной позы
	//! @param [in] ipng номер текущего кадра
	//! @param [in] iprev номер предыдущего кадра
	void evaluate_poses_intermediate(int ipng, int iprev);

	//! @brief обновляет окна расчета статистик при появлении новой позы ipng
	//! @param [in] ipng номер текущего кадра
	void update_kitti_windows(int ipng);

	//! @brief обновляет невязку между GT- и EVAL Pose по сдвигу и повороту
	//! @param [in] ipng номер текущего кадра
	void update_discrepancy(int ipng);

	//! @brief обновляет окна расчета статистик при появлении новой позы ipng
	//! @param [in] ipng номер текущего кадра
	//! @param [in] KittiErrorsструктура KITTI (A.Geiger) для расчёта ошибок результатов прогона
	//! @param [in] iwnd номер окна от 0 до 8
	void register_sequence_window_error(int& ipng, const KittiErrors wnd, int iwnd);

	//! @brief до начала прогона собирает данные с локалайзера (первая итерация) и отрисовывает на карте (local_map) от стендов
	void collect_GT_for_drawing_map(std::shared_ptr<ar::ALocalizer>& loc_trk);

	//! @brief поворот СК трамвая (GT) в стандартную СК KITTI
	//! @param [in -> out] trans_tram - x, y, z
	void change_axes_gt(cv::Vec3d& trans_tram);

	//! @brief формирование списка данных от локалайзера
	//! @param [in] InputEntry список эпизодов
	//! @param [in] finishEp последний эпизод, который ограничивает список
	std::vector<std::string> gen_trk_list(const InputEntry& ie, int finishEp);

	//! @brief обновляем проезд (график) после смены эпизода. сброс настроек - понадобится, когда будем наблюдать за каждым эпизодом в отдельности
	void reset();

	//! @brief поворот карты GT (из GM-координаты в стандартные от КИТТИ (едем изначально вверх))
	//! @param [in] angle_rot угол поворота
	//! @retval cv::Matx22d матрица поворота для 2D
	cv::Matx22d gen_mat_rotation_map(double angle_rot = -31);

	//! @brief отрисовка одного из углов на local_map
	//! @param [in] angle_type угол поворота: yaw, pitch, roll
	//! @param [in] color_line цвет линии угла
	//! @param [in] name_4legend наименование линии для легенды
	//! @param [in] y_level расположение отрисовки
	void draw_angle(double angle_type, cv::Scalar color_line, std::string name_4legend, int y_level);

	//! @brief отрисовка одного из углов на local_map
	//! @param [in] angle_type_GT угол поворота из GT: yaw, pitch, roll
	//! @param [in] angle_type_RE такой же угол поворота, но полученный из VisOdom: yaw, pitch, roll
	//! @param [in] color_line цвет линии разницы углов
	//! @param [in] name_4legend наименование линии для легенды
	//! @param [in] y_level расположение отрисовки
	void draw_diff_angle(double angle_type_GT, double angle_type_RE, cv::Scalar color_line, std::string name_4legend, int y_level);

	//! @brief отрисовка скоростей GT и полученную на local_map
	//! @param [in] speed_eval скорость, полученная из выявленного смещения
	//! @param [in] speed_gt скорость, полученная от локалайзера
	//! @param [in] diff_png сколько изображений между парами кадров, для которых расчитывали смещение (== скорость)
	//! @param [in] diff_translation norm(gt) - norm(eval)
	void draw_speed(double speed_re, double speed_gt, int diff_png, double diff_translation);

	//! @brief отрисовка полученной траектории. Можно менять направление через angle_rot_map в зависимости от направления GM
	//! @param [in] re_pose_cur полученая поза
	//! @param [in] gt_pose_cur поза их локалйзера
	//! @param [in] angle_rot_map угол поворота ПОЛУЧЕННОЙ траектории
	void draw_trajectory(cv::Vec3d re_pose_cur, int angle_rot_map = 0);

	//! @brief отрисовка дегенды local_map
	//! @param [in] color_line цвет линии разницы углов
	//! @param [in] name_4legend наименование линии для легенды
	//! @param [in] y_level расположение отрисовки
	void make_legend(cv::Scalar color_line, std::string name_4legend, int y_level);

	//! @brief Calculates rotation matrix given euler angles.
	//! @param [in] theta углы Эйлера
	//! @param [out] матрица поворота
	cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta);

	//! @brief Сглаживание точки схода методом скользящего окна
	//! @param [in] last_FOE последнее полученное значение FOE
	//! @param [out] сглаженное значение FOE с учитыванием истории
	cv::Point2d return_smoothedFOE(cv::Point2d last_FOE);

	//! @brief получение среднего между (x,y) точками
	//! @param [out] среднее значение
	cv::Point2d get_middle_point();

	//! @brief получение среднего между средними значениями оптического потока (независимыми между собой)
	//! @param [in] среднее значение векторов оптическоего потока для текущего кадра
	//! @param [out] среднее значение
	float get_middle_float(float curr_value);

	//! @brief сглаживает полученное среднее значение норм векторов оптического потока, чтобы не было большого разброса +
	//! гарантированное выделение момента остановки
	//! @param [in] mean_optflow среднее значение оптического потока (нормы векторов) для текущего кадра
	//! @param [out] среднее значение оптического потока для скользящего окно
	float return_smoothedOptFlow(float mean_optflow);

	//! @brief функция нормализует local_map с точки зрения масштаба, чтобы весь график собственного движеия влезал в верхнюю часть local_map,
	//! а также синхронизировал угол поворота в отрисовке, чтобы мы всегда двигались вверх в начале проезда
	void normalize_local_map();

	//! @brief подача локалайзеру актуальной информации по положению на маршруте
	//! @param [in] текущий шот
	void feed_localizer(ar::SmartPtr<ar::AShot> shot, std::shared_ptr<ar::ALocalizer> loc, Setup &setup);

	//! @brief отрисовка столбцов геодезических замеров с нормализованной высотой
	//! @param [in/out] src_image изображение Forward View
	//! @param [in] p текущее положение робота на маршруте
	void draw_geodetic_data(cv::Mat& src_image, const ar::ModelPosition& p);

	//! @brief отрисовка физических столбов по геодезическим замерам
	//! @param [in/out] src_image изображение Forward View
	//! @param [in] p текущее положение робота на маршруте
	void draw_geodetic_poles_data(cv::Mat& src_image, const ar::ModelPosition& p);

	//! @brief отрисовка высот Emlid-утройства
	//! @param [in/out] src_image изображение Forward View
	//! @param [in] p текущее положение робота на маршруте
	void draw_emlid_data(cv::Mat& src_image, const ar::ModelPosition& p);

	//! @brief отрисовка рельс по сплайну маршрута
	//! @param [in/out] src_image изображение Forward View
	//! @param [in] p - текущее положнеие робота на маршруте
	void draw_railpath_data(cv::Mat& src_image, const ar::ModelPosition& p);

	//! @brief отрисовка траектории проезда, полученной с прошлого прогона
	//! @param [in/out] src_image изображение Forward View
	//! @param [in] points X, Y - координаты
	//! @param [in] iter - метка для итерации по данным с синхронизации
	void draw_visodometry_points(cv::Mat& src_image, std::vector<std::pair<double, double>>& points, const size_t iter);

	//! @brief отрисовка сплайна на гугл карте
	//! @param [in] p - текущее положение робота на маршруте
	//! @param [in] gmr - объекты GeoMap
	void draw_tram_trajectory_on_google_map(const ar::ModelPosition& p, GeomapRenderer &gmr, Setup &s,
																					std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d> > filtered_poles_gm = {});

	//! @brief отрисовка высотных графиков
	//! @param [in] p - текущее положение робота на маршруте
	void draw_height_figure(const ar::ModelPosition &p);

	//! @brief получение ошибки репроекции сегментируемых столбов и их реальным положением на GeoMap
	//! @param [in/out] src_image изображение Forward View
	//! @param [in] p - текущее положение робота на маршруте
	//! @param [in] radius - в какой области смотрим радиус
	//! @retval double - ошибка репроекции в метрах
	double get_poles_reproject_error(cv::Mat& src_image, const ar::ModelPosition &p, eobj::Reading<GMObjectContainer> r_objects,
																	 double radius);

	//! @brief получение ошибки предсказывания пути на 0.25, 0.5, 1 и 2 секунды относительно соответствующих не-предиктнутых измерений
	//! @param [in] p - текущее положение робота на маршруте
	double get_predict_error(const ar::ModelPosition& p, const int64_t& now_time);

	//! @brief локализация по столбам, используя сегментацию, google-map и прочее
	//! @param image - изображение для сегментации
	//! @param forward_visual - отрисовка сегментации, высотных столбов и прочее
	//! @param p - текущее положение робота на маршруте
	//! @param time_stamp - временная метка текущего шота
	std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d> > process_poles(const cv::Mat &image, cv::Mat &forward_visual, const ar::ModelPosition &p,
																																							 const int64_t &time_stamp, Setup &setup, ar::MultiObjectFilter<ar::SingleObjectFilterRPhiFixedWithPoorGPS> &pole_tracker);

	//! @brief получив угол столба, пытаемся отнести его к определённому кластеру
	//! @param [in] angle_value - угол столба
	//! @retval индекс подходящего кластера или же -1 в другом случае
	int where_is_closer(double angle_value);

	//! @brief обновление кластеров yaws сегментированных столбов
	//! @param yaw_value - полувенный yaw от пикселя
	//! @param time_stamp - временная метка, когда был получен угол
	void update_yaws_clusters(double yaw_value, int64_t time_stamp);

	//! @brief расчёт регрессии для всех собранных кластеров углов столбов
	//! @param time_stamp - временная метка для предсказывания положения угла (для случая, если временной метки нет)
	//! @note если мы предсказали угол определённого кластера +- верно, то нужно искусственно увеличить кол-во таких углов,
	//! чтобы при последующих предикций новый угол не затерялся
	//! @retval vector<Point2d> предсказанные углы и временные метки для каждого кластера
	std::vector<cv::Point2d> predict_poles_yaws(const int64_t& time_stamp);

	// *** данные для отрисовки local_map ***
	cv::Mat3b local_map = cv::Mat3b(1100, 2400, cv::Vec3b(255, 255, 255)); //!< карта прогона с графиками скорости, собств. движения и прочее. Ос
																																				 //!< ось абсцисс - смешение в метрах, ось ординат - полученная скорость
	cv::Mat3b local_map4reset = cv::Mat3b(1100, 2400, cv::Vec3b(255, 255, 255)); //!< карта прогона для сброса данных после каждого эпизода
	cv::Point local_map_start; //!< начальная точка отрисовки всех кинем. хар-ик на local_map
	std::vector< double >       path_re;  //!< path_re[i] == длина re траектории от старта до i-го фрейма, рассчитанная как сумма вычисленных смещений
	std::vector< double >       path_gt_drawing; //!< смещение транспорта по GT
	std::vector< double >       path_gt{0.0};  //!< path_gt[i] == длина gt траектории от старта до i-го фрейма, рассчитанная по граундтрус как сумма смещений
	double tag100 = 0; //!< зарубка о прохождении очередных 100 метров
	double tag005 = 0; //!< зарубка о прохождении очередных 5 метров
	cv::Point diff_start_gt_eval = {0,0}; //!< разница между начало отрисовки GT и полученных данных из визуальной одометрии
	cv::Point common_start_gt_eval = {0,0}; //!< подбор общего начала для отрисовки
	int seq_num = -1; //!< номер эпизода, по умолчанию -1
	bool isStartPos = true; //!< переменная, отвечающая за нахождение НЕНУЛЕВЫХ GT данных локалайзера
													//!< нужно, чтобы делать поправку на это значение

	bool isStartAngle = true;
	double strightAngle4GT = 0; //!< такой угол GT, который выравнивает первые 3 метра вверх (КИТТИ по дефолту движется вверх)
	bool bad_locilizer = true; //!< если был сдвиг yaw на 360 градусов (на trm.109.013)
	int diff4bad = 0; //!<  если был сдвиг yaw на 360 градусов (на trm.109.013)
	int bias = 50; //!< смещение начала координат в точку (0,0) для графика
	int how_times_incr = 150; //!< множитель сжатия общего графика в local_map,  когда хотим на одном графике вместить все эпизоды
	int how_timed_incr_Y = 2; //!< множитель расширения линий углов в local_map для улучшения качества восприятия
	bool isEnough = true; //!< ключ, показывающий, хватит ли отрисовки на одном картинке (920 метров)
	cv::Point height_map_start = {50, local_map.rows - 400}; //!< точка отрисовки боковых пометок (0, 10, ..., 150)
	cv::Mat height_map = local_map;

	double height_pix_per_meter = 2.; //!< нормировочный множитель для масштабирования разметки оси ординат
	double path_pix_per_meter = 2.; //!< нормировочный множитель для перевода полученного смещения в путь (ось абсции)
	double translation2speed = 0.360073; //!< нормировочный множитель для перевода смещения в скорость
	double speed_ON_local_map = 10; //!< нормировочный множитель для отрисовка полученной скорости на local_map
	cv::Point angle_map_start = {50, local_map.rows - 400}; //!< точка отрисовки углов (yaw, pitch, roll)
// ***

// *** данные для расчёта визуальной одометрии ***
	std::deque< Snapshot > snapshots; //!< статистика по кадру
	int ipng = 0; //!< номер кадра
	int curr_ep_algo; //!< текущий эпизод
	int start_ep_algo; //!< первичный эпизод
	int finish_ep_algo; //!< последний эпизод
	std::vector< cv::Affine3d > poses_re; //!< позы, вычисленные с помощью визуальной одометрии
	std::vector< cv::Affine3d > poses_gt; //!< позы, полученные как граундтрус (GT)
	std::vector< cv::Affine3d > poses_gt_drawing; //!< позы из локалайзера для отрисовки карты на первом кадре
	std::vector<int64_t>        timestamps; //!< временные метки для синхронизации
	SpeedMap speedmap;  //!< гистограмма оценок скоростей, полученных к i-му кадру
	Camera full_cam_opt; //!< калибы
	ForwardRectify rectifying; //!< ректификация кадра
	cv::Ptr<cv::optflow::DISOpticalFlow>   optflower_DIS_motion; //!< инструмент для оптического потока
	cv::Ptr<cv::optflow::DISOpticalFlow>   optflower_DIS_stereo; //!< не используется
	std::ofstream ofs_output; //!< поток для вывода таблицы разностей с gt
	std::ofstream ofs_poses;  //!< поток для вывода расчитанных интегральных поз
	cv::Vec3d start_t; //!< начальная позиция траспорта в GT. Нужно для синхронизации отрисовки gt и полученных значений
	cv::Matx33d start_R; //!< начальный угол поворотав GT. Нужно для синхронизации отрисовки gt и полученных значений
	double start_yaw_gt; //!< эйлеровый угол, полученный из локалайзера
	double start_pitch_gt; //!< эйлеровый угол, полученный из локалайзера
	double start_roll_gt; //!< эйлеровый угол, полученный из локалайзера

	cv::Matx33d ext_R; //!< матрица поворота В ДАННУЮ С.К.(rotation)
								 //!< Не стоит путать матрицу поворта с матрицей перехода
								 //!< (которая вращает координатные оси и является обратной к данной)

	cv::Vec3d ext_t; //!< вектор переноса ДАННОЙ С.К. (translation)
								 //!< То есть это координаты центра данной С.К. в исходной!!!
								 //!   std::deque< Snapshot > snapshots;
	std::deque<cv::Point2d> FOE_window;
	cv::Rect roi;
	std::deque<float> mean_opt_flow; //!< вектор средних значений оптического потока, чтобы найти границу,
																		//!< которая бы отвечала за возможное стоячее движение и активное (speed > 0).
																		//! Требуется для избавления от моментов, когда мы стоим, но перед нами проезжает транспорт
																		//! и из-за этого происходит накопление ошибки поворота
	// ***

// *** отписывание данных с визуальной одометрии ***
	KittiSequenceReport report; //! базовая структура для расчёта результатов прогона
	KittiOdometryStatistics kitstat; //! накапливаемая статистика по окнам [100, 200, ..., 800]
// ***
// *** данные для работы с локалайзером в случае трамвайных стендов
	std::string gt_path =                     "/testdata/trm/out.trk.crisploc/trm.168/trm.168.%03d.odo.trk.tsv"; //!< папка с данными trk-локалайзера
	std::string gt_path_spline =              "/testdata/trm/spline/17.1/route.spline.tsv"; //!< сплайн маршрута трамвая
	std::string geodetic_data_path =          "/testdata/geosurvey/v36/res_geo_spline.tsv"; //!< путь к файлу с геодезическими замерами
	std::string emlid_data_path =             "/testdata/trm/inp/trm.168/trm.168.info.tsv"; //!< данные емлида (можно и онлайн получать)
	std::string geodetic_data_poles_path =    "/testdata/geosurvey/v36/res_geo_poles.tsv"; //!< замеры столбов от геодезистов
	std::string srtm_data_path =              "/testdata/geosurvey/v36/srtm_data.tsv"; //!< данные SRTM
	std::string vehicle_config =              "/testdata/soy/inp/soy.304/calib/vehicle.yml"; //!< настройки трамвая как физического объекта
	std::string geodetic_height_spline_path = "/testdata/trm/spline/17.1/route.height_profile.tsv";
	std::string google_map_path =             "/testdata/maps/samplemap";
	std::string geomap_objects_path =         "/testdata/maps/trm";
	std::string srtm_map_path =               "/testdata/maps/elevation/srtm/mosregion";
	std::string dnn_options_path =            "/testdata/models/mxnet/tramway_bisenet/sgm.005/bisenet_opt.yaml";
	bool player_is_set_up = false;
	bool is_init_start_position = false;      //!< проверка инициализации локалайзер
	ar::SmartPtr<ar::DrivareaSegmenter> drivarea;

	std::vector<std::string> gpsDeviceNames = {"emlidBack", "!agrodroidGps"}; //! имя GPS-датчика
	std::string odoDeviceName = "dbwFbVehicleCan"; //! имя девайса для считывания колесной одометрии
	std::string sensePosesGpsDeviceName = "emlidLeftGPS"; //! имя поля в конфиге sense_poses.yml для gps девайса
	GPSDevice gps_device; //! устройство gps (emlid)
	int fixQuality = 1; //! GPGGA
	std::vector<Geodetic_data> geod_data;
	std::vector<Geodetic_data_poles> geod_data_poles;
	std::vector<Stand_emlid_data> emlid_data;
	std::vector<STRM_data> strm_data;
	MapElevation elevation_map; //!< all SRTM heights in geomap
	std::vector<std::vector<cv::Point2d>> poles_gm_coords; //!< GM координаты (один кадр - один замер столба) сегментированных столбов
	std::vector<cv::Point2d> poles_geomap_gm_coords; //!< GM координаты столбов из геомап

	RailPath path; //! сплайн траектории трамвая, если таковой имеется
	TramVehicleModel tvm; //! параметры трамвая как физ. объекта (частный случай VehicleModel)
	VehicleModel vm; //! параметры робота
	std::shared_ptr<ar::ALocalizer> loc_with_height; //! трамвайный локализатор, который учитывает высоту при проекциях
	std::shared_ptr<ar::ALocalizer> loc_trk; //! трамвайный локализатор, работающий с TRK-файламши
	std::shared_ptr<ar::ALocalizer> agro_loc; //! агро-локалайзер
	CPlot height_figure_draw; //! график с высотами
	CPlot predict_error_figure_draw; //! график с ошибками предикции
	CPlot hist_yaw_poles; //! распределение наблюдений углов видимых столбов
	CPlot plt_yaw_clusters; //!< отрисовка кластеров и построенных регрессий
	std::vector<double> obs_yaws; //! набор всех углов пикселей, полученных из сегментации
	int obs_yaw_hist = -1;//! индексы для перерисовки графиков
	int geo_figure_index = -1;
	int geo_deriv_figure_index = -1;
	int emlid_figure_index = -1;
	int now_pos = -1;
	std::vector<int> plt_yaw_clusters_index;

	std::vector<cv::Point2d> geodeitc_height_spline; //! данные геодезических высот по трамвайному маршруту
	std::vector<cv::Point2d> geodeitc_height_deriv_spline; //! производые по данным геодезических высот по трамвайному маршруту
	std::vector<cv::Point2d> emlid_data_figure; //! оффлайн данные со стенда
	std::vector<cv::Point2d> row_geodetic_height_data_figure; //! оффлайн данные со стенда

	cv::Point2d en_now; //! текущее положение робота

  std::vector< Selected_geomap_pole > selected_poles; //! кэшированная ключевая информация по близлежащим столбам
  std::deque<cv::Point2d> prev_trajectory; // последние сто точек по GPS
  std::vector< int > selected_poles_idx; // индексы близлежащих столбов

  cv::Mat prev_pole_segm; //! сегментация столбов прошлого с прошлого кадра

  cv::Ptr<ar::DrivareaOptions> options;
  std::vector<Yaws_cluster> yaws_clusters;
  std::vector<std::pair<double, int>> gotten_poles_yaws; // накопленные углы от сегментированных столбов

  int64_t prev_time_stamp = -1;
  // ***
};



//! @brief функция для генерации текущей даты.
//! Требуется для отписывания результатов прогона в правильной хронологии
inline std::string strdate() {
  time_t t = time(0);
  struct tm * timeStruct = localtime(&t);
  std::string str = cv::format("%d-%02d-%02d", timeStruct->tm_year + 1900, timeStruct->tm_mon + 1, timeStruct->tm_mday);
  std::cout << str << std::endl;
  return str;
}

//! @brief функция для генерации текущего времени.
//! Требуется для отписывания результатов прогона в правильной хронологии
inline std::string strtime() {
  time_t t = time(0);
  struct tm * timeStruct = localtime(&t);
  std::string str = cv::format("%02d-%02d-%02d", timeStruct->tm_hour, timeStruct->tm_min, timeStruct->tm_sec);
  std::cout << str << std::endl;
  return str;
}

class KittiOdometer
{
public:

  //! @brief констуктор инициализации параметров прогона
  //! @note TODO: передавать список config-параметров
  KittiOdometer(bool use_config);

  //! @brief деструктор для отписывания времени прогона
  ~KittiOdometer();

  //! @brief логгирование параметров прогона в файл "/report.run%04d.txt"
  void init_run();

  //! @brief нахождение визуальной одометрии на картинках. В наших стендах не используется
  //! @param [in] seq_num номер проезда
  //! @param [in] num_frames кол-во изображений
  //! @param [in] start_frame с какого изображения стартуем
  int process_sequence(int& seq_num, int num_frames = 0, int start_frame = 0);

  //! @brief запуск нахождения визуальной одометрии на .avi.
  //! @param [in] season_shooter Класс, сшивающий различные эпизоды при съемке
  //! @param [in] season_pattern шаблон сезона
  //! @param [in] number_start_frame с какого изображения стартуем
  //! @param [in] final_episode используется, если мы хотим доехать до определённого эпизода
  int process_avi(ar::SeasonShooter& season_shooter, std::string& season_pattern);

  //! @brief выгрузка snapshot в общую последовательность данных для расчёта визуальной одометрии
  //! @param [in] ss полноценные данные о текущем кадре - GT, расчитанный оптический поток и прочее
  void process_snapshot(Snapshot& ss);

   //! @brief расчёт поворота и смещения между двумя snapshot
   //! @param [in -> out] ss_pre достаточные данные о прошлом кадре
   //! @param [in -> out] ss_cur достаточные данные о текущем кадре
  void process_two_shots(Snapshot& ss_pre, Snapshot& ss_cur);

public:
  // *** Параметры для сохранения отчёта ***
      std::map< int, KittiSequenceReport > seq_num2report; //!< сводная таблица по статистике прогона
      const char* revision;// = SVNVERSION; //!< номер ревизии для отчёта
      Setup setup; //!< настройки алгоритмов
      int run_number = 1; //!< номер прогона для текущего даты (= дня, месяца и т.д.)
      std::string run_date = strdate(); //!< текущая дата для отчёта
      std::string run_time = strtime(); //!< текущее время для отчёта
      std::string output_folder =  "/testdata/trm/out." + run_date + '/'; //!< место для сохранения отчёта о прогоне
      std::ofstream ofs_report; //!< поток для вывода сводного отчета
      KittiSequenceData ksdata; //!< данные по прогону одного sequence или его фрагмента
  // ***
      ar::TsvWriter GT_writing; //!< отписывание (x, y) координат, полученных через ВО
      std::vector<std::pair<double, double>> points4vis; //!< считанные GT-точки, полученные на предыдущем прогоне
  // *** пути к искомым данным проезда
      std::string dataset_folder = "/testdata/trm/inp/"; //!< папка с файлам прогона
      std::string pose_folder = ""; //!< путь к файлу с полученным позами (кэш)
      std::string vo_GT = "wrote_GT.tsv"; //!< путь к файлу, в который будут записываться GT
  // ***

  size_t iter_prev_run = 0; //!< итерация по GT, полученных из предыдущего прогона
  int din_change_pp = 0; //!< костыль, который динамично сдвигает PP (точку схода) влево (!) в случае расчёта поворота


private:

  //! @brief поиск основной матрицы через эпиполярную геометрию. Матрицы описывает motion между двумя кадрами (смещение + поворот)
  //! проблема в том, что смещение ищется только в виде единичного вектора и его модуль не явлется достоверным с точки зрения земных координат
  //! поэтому используется дополнительная функция estimate_translation для расчёта модуля смещения
  //! @param [in] ss_pre достаточные данные на прошлом кадре
  //! @param [in] ss_cur достаточные данные на текущем кадре
  void find_essential(Snapshot& ss_pre, Snapshot& ss_cur);

  //! @brief поиск модуля смещения между двумя кадрами
  //! @param [in] ss_pre достаточные данные на прошлом кадре
  //! @param [in -> out] ss_cur достаточные данные на текущем кадре
  //! @retval int кол-во пригодных точек, по которым производился расчёт смещения
  int estimate_translation(Snapshot& ss_pre, Snapshot& ss_cur);

  //! @brief финальный расчет данных для сиквенса после окончания нахождения поз
  //! @param [in] output_folder путь к папке для сохранения отчёта
  //! @param [in] setup настройки, при которых был запущен алгоритм
  void evaluate_poses_final(const std::string& output_folder, const Setup& setup);

   //! @brief Кэширования оптического потока для увеличения скорости расчёта поз
   //! @param [in] optflow расчитанный оптический поток
   //! @param [in] prepng номер кадра, от которого считался ОП
   //! @param [in] curpng номер кадра, к которому считался ОП
   //! @param [in] prefix префикс к именованию кэша ОП
  void write_optflow(cv::Mat2f& optflow, int prepng, int curpng, const char* prefix);

  //! @brief Чтение закэшированного оптическего потока
  //! @param [out] optflow переменная для сохранения считанного ОП
  //! @param [in] prepng номер кадра, от которого считался ОП
  //! @param [in] curpng номер кадра, к которому считался ОП
  //! @param [in] prefix префикс к именованию кэша ОП
  //! @retval bool (true/false), если чтение произошло успешно
  bool read_optflow(cv::Mat2f& optflow, int prepng, int curpng, const char* prefix);

  //! @brief подготовка оптического потока между смежными кадрами
  //! @param [in] ss_pre данные по предыдущем кадру
  //! @param [in] ss_cur данные по текущему кадру
  void optflow_prepare(Snapshot &ss_pre, Snapshot &ss_cur);

  //! @brief подготовка оптического потока между смежными кадрами
  //! @param [in] flow рассчитанный оптический поток
  //! @param [in] grid сетка сжатого оптического потока
  //! @param [out] pps корр. точки из предыдущего кадра
  //! @param [out] qqs корр. точки из текущего кадра
  void flow2gridpoints(const cv::Mat2f flow, cv::Size& grid, cv::Mat2f& pps, cv::Mat2f& qqs);

  //! @brief подготовка корреспондирующих точек из векторов оптического потока
  //! @param [in] pre данные по предыдущем кадру
  //! @param [in] cur данные по текущему кадру
  //! @param [in] PrP точка схода
  //! @param [in] focal фокальная длина
  //! @retval вектор корреспондирующих точек
  Coor_points create_corr_points(const SFrame &pre, const SFrame &cur,
                                 const cv::Mat1b &mask_low_sigma_grid,
                                 const cv::Point2d &PrP, const double focal);

  //! @brief Отписывание GT (пригодится для обучения сеток для сегментации ЖД путей)
  //! @retval cтатус
  bool init_gt_writer();

  //! @brief для отладки полученных GT
  //! @retval cтатус
  bool init_gt_reader();
};

//! @brief функция для дополения отчёта данными пользователя
//! @param [out] ofs поток вывода, который будем выведен в отчёт
inline void print_user(std::ofstream& ofs)
{
  if (const char* env_p = std::getenv("LOGNAME"))
    ofs << "USER: " << env_p << '\n';
  if (const char* env_p = std::getenv("USERNAME"))
    ofs << "USER: " << env_p << '\n';
}

//! @brief функция для разделния отчёта на части
//! @param [in] counter количество готовых столбов
//! @param [in] columns желательное количество столбов в файле отчёта
inline char setup_delimeter(int counter, int columns) { return counter % columns ? '\n' : '\t'; }

//! @brief переопределённый оператор вывода для записи настроек алгоритма в отчёт
//! @param [out] ofs поток вывода, который будем выведен в отчёт
//! @param [in] setup настройки запуска алгоритма
//! @retval std::ofstream& ссылка на поток вывода
inline std::ofstream& operator << (std::ofstream& ofs, const Setup& setup)
{
  int counter = 0;
  int ncolumns = 3;
  for (auto& xx : setup.strings)
    ofs << xx.first << '\t' << xx.second << setup_delimeter(++counter, ncolumns);
  for (auto& xx : setup.sizes)
    ofs << xx.first << '\t' << xx.second << setup_delimeter(++counter, ncolumns);
  for (auto& xx : setup.doubles)
    ofs << xx.first << '\t' << xx.second << setup_delimeter(++counter, ncolumns);
  for (auto& xx : setup.ints)
    ofs << xx.first << '\t' << xx.second << setup_delimeter(++counter, ncolumns);
  for (auto& xx : setup.bools)
    ofs << xx.first << '\t' << xx.second << setup_delimeter(++counter, ncolumns);
  return ofs;
}

//! @brief переопределённый оператор вывода для кэширования поз
//! @param [out] ofs поток вывода, который будем выведен в отчёт
//! @param [in] t_R поза, полученная в результате извлечения из основной матрицы (essential matrix)
//! @retval std::ofstream& ссылка на поток вывода
inline std::ofstream& operator << (std::ofstream& ofs, const cv::Affine3d& t_R)
{
    ofs << t_R.translation()[0]  << " " << t_R.translation()[1] << " "<< t_R.translation()[2] << " "
       << " " << t_R.rotation()(0,0) << " " << t_R.rotation()(0,1)<< " " << t_R.rotation()(0,2)
            << " " << t_R.rotation()(1,0) << " " << t_R.rotation()(1,1) << " " << t_R.rotation()(1,2)
            << " " << t_R.rotation()(2,0) << " " << t_R.rotation()(2,1) << " " << t_R.rotation()(2,2);
    return ofs;
}

//! @brief переопределённый оператор ввода для кэширования поз
//! @param [in -> out] is, поток вывода для формирования вектора полученных поз
//! @param [in] pose кэшировання поза
//! @retval std::ifstream& ссылка на входной поток
inline std::ifstream& operator >> (std::ifstream& is, cv::Affine3d& pose)
{
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 4; j++)
      is >> pose.matrix(i, j);
  return is;
}
