//! @file    prj.sandbox/stabilizer/stabilizer.h
//! @author  Александр Арсентьев
//! @brief   Класс для стабилизации видео
//! @date    15.04.2021

#include <mutex>
#include <optional>
#include <deque>
#include <vector>
#include <future>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <opencv2/optflow.hpp>

#include "rtdb/rtdb.hpp"

#pragma once

//! @brief класс со стабилизированным изображением, его оценкой и временной меткой
//! @note изначально инициализируем первоначальным изображением и если PSNR не слишком высокие (<25),
//! то переинициализируем stabImg, в противном случае в StabImg лежит исходное изображение
class StabSnapshot
{
public:
  StabSnapshot() = default;
  StabSnapshot(const cv::Mat& img, long long cur_msec) : stabImg(img.clone()), msec(cur_msec) {}
  void operator = (const StabSnapshot& ss) {
    this->msec = ss.msec;
    this->stabImg = (ss.stabImg.value_or(cv::Mat()).clone());
    this->PSNR = ss.PSNR;
  }

  bool empty()
  {
    return stabImg.value_or(cv::Mat()).empty();
  }
public:
  std::optional<cv::Mat> stabImg;
  long long msec = 0.0;
  double PSNR = -1; // похожесть с прошлым кадром, если таковой имеется
};

//! @brief индекс изображений для rtdb
struct MsecIndex {
  int64_t grabMsec = 0;
};

inline bool operator<(const MsecIndex& mi1, const MsecIndex& mi2) { return mi1.grabMsec < mi2.grabMsec; }
inline bool operator==(const MsecIndex& mi1, const MsecIndex& mi2) {return mi1.grabMsec == mi2.grabMsec;}
inline MsecIndex operator+(const MsecIndex& mi1, const int64_t diff) {
  return {mi1.grabMsec + diff};
}
template<>
inline int64_t ar::diff<MsecIndex>(const MsecIndex &l, const MsecIndex &r)
{
  return l.grabMsec - r.grabMsec;
}
template<>
inline std::string ar::getTableName<MsecIndex>(const MsecIndex&) {
  return "forStab";
}


//! @brief структура для инициализации параметров алгоритма
struct Setup
{
  std::map< std::string, std::string >  strings;
  std::map< std::string, cv::Size >     sizes;
  std::map< std::string, double >       doubles;
  std::map< std::string, int >          ints;
  std::map< std::string, bool >         bools;
};

//! @brief структура для параметров межкадровых трансформаций
struct TransformParam
{
  TransformParam() {}
  TransformParam(double _dx, double _dy, double _da, double _ds_x, double _ds_y) :
    dx(_dx), dy(_dy), da(_da), ds_x(_ds_x), ds_y(_ds_y) {}
  double dx;
  double dy;
  double da;
  double ds_x;
  double ds_y;
};

//! @brief возможные трансформации между кадрами
enum transformationMethod {
  RIGID,
  AFFINE,
  NOT_RECOGNIZED
};

enum smoothing_method {
  KALMAN,
  MOVING_AVERAGE_WINDOW,
  CUSTOM,
  NOT_SET
};

class Kalman_filter
{
  // todo: использовать наследование от фильтра из core
public:
  void update(const TransformParam& obs);
  void predict();
  bool ready() {return updates >= 0;}
  bool correct_motion(const TransformParam& tp);
  cv::Mat get_oldest_predict();
private:

  // params
  double dx = 0.0;
  double dy = 0.0;
  double da = 0.0;
  double ds_x = 0.0;
  double ds_y = 0.0;

  double sx = 0.0;
  double sy = 0.0;

  double diff_scale_X = 0.0;
  double diff_scale_Y = 0.0;
  double diff_trans_X = 0.0;
  double diff_trans_Y = 0.0;
  double diff_thetha ;

  double errscale_X = 1.0;
  double errscale_Y= 1.0 ;
  double errthetha= 1.0 ;
  double errtrans_X = 1.0;
  double errtrans_Y = 1.0;

  double Q_scale_X = 0.004;
  double Q_scale_Y = 0.004;
  double Q_thetha = 0.004;
  double Q_trans_X = 0.004;
  double Q_trans_Y = 0.004;

  double R_scale_X = 0.5;
  double R_scale_Y = 0.5;
  double R_thetha = 0.5;
  double R_trans_X = 0.5;
  double R_trans_Y = 0.5;

  double sum_scale_X = 0.0;
  double sum_scale_Y = 0.0;
  double sum_thetha = 0.0;
  double sum_trans_X = 0.0;
  double sum_trans_Y = 0.0;

  // state
  double scale_X = 0.0;
  double scale_Y = 0.0;
  double thetha = 0.0;
  double trans_X = 0.0;
  double trans_Y = 0.0;

  int updates = -1;

  // prediction
  std::deque<cv::Mat> smoothed_mat;

};

//! @brief стабилизатор видеопоследовательности
class Stabilizer
{
public:
  //! @brief конструктор с инициализацией потоков с дефолтными настройками
  Stabilizer(smoothing_method sm);

  //! @brief инициализация потоков с заданными настройками
  Stabilizer(const Setup& s);

  //! @brief скармливание изображеня и если Стабилизатор сошёлся,
  //! то выдача нового кадра
  //! @param img - изображения для стабилизации
  //! @param msec - временная метка изображения (если видео никак не привязан к нашим стендам, то можно не указывать)
  StabSnapshot feed_image_and_try_get_stab_it(const cv::Mat& img, long long msec = 0);

  //! @brief внутренняя оценка алгоритма, которая показывает похожесть двух последовательных кадров
  //! @note при качественной стабилизации два кадра должны быть крайне сильно похожи
  //! @param mat - два последовательных кадра
  double getPSNR(const StabSnapshot &I1, const StabSnapshot &I2);

  //! @brief деструктор для финального вывода накопленных статистик
  ~Stabilizer()
  {
    std::cout << "Final ITF = " << ITF / ITFMeasCount << std::endl;
  }

private:
  //! @brief метод, который каждые __switch_frame будем менять transformationMethod
  //! в зависимости от того, какой лучше подходит
  //! @param rigid - полученная матрица rigid преобразования на последнем кадре
  //! @param partial_affine - аналогично матрица аффинного преобразования
  //! @param prev_keypoints - ключевые точки с предыдущего кадра
  //! @retval transformationMethod индекс подходящего метода
  transformationMethod rigid_or_affine(cv::Mat rigid, cv::Mat partial_affine,
                                       std::vector<cv::Point2f> prev_keypoints);

  [[noreturn]] void motion_estimation(); //!< получения параметров движения камеры
  [[noreturn]] void motion_compensation(); //!< компенсации движения
  [[noreturn]] void generate_stab_image(); //!< генерации стабилизированного изображения

  Kalman_filter kf;
  smoothing_method smooth_method = smoothing_method::NOT_SET;
  bool init_ORB(); //!< инициализация ORB-детектора
  bool detect_ORB_kpts(const cv::Mat& img_for_detect, std::vector<cv::Point2f> &keypoints_res);

  std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
  match_found_ORB_kpts(const cv::Mat &descriptor_res_prev, const cv::Mat &descriptor_res_cur,
                            std::vector<cv::KeyPoint>& full_keypoints_data_prev,
                            std::vector<cv::KeyPoint>& full_keypoints_data_cur);

  //! @brief сбор статистики для оценок метрик
  //! @param ss - стабилизированное изображение
  //! @note оценка идёт по стабилизированному изображению и если она плохая, то ставим исходное изображение
  void collect_stat(StabSnapshot& ss);

  std::optional<cv::Ptr<cv::Feature2D>> __orb;
  size_t __switch_frame = 20; //!< кол-во кадров, после которых идёт переключение трансформаций (affine <-> rigid)
  size_t __smoothing_radius = 30; //!< радиус сглаживания (в кадрах)
  size_t __horizontal_border_crop = 20; //!< обрезка границы (в пикселях), чтобы уменьшить черные границы от стабилизации
  double __rigid_scale = 1; //!< при rigid трансформации говорим, что масштаб = 1
  double __keypoint_threshold = 20; //!< максимальная дистанция в пикселях между keypoints на двух кадрах
  size_t __keypoint_retrack_frames = 20; //!< через сколько кадров будет пересчитывать ключевые точки
  size_t __max_keypoints = 100;
  double __keypoints_quality = 0.01; //!< требуемая точность
  double __min_keypoint_dist = 20; //!< минимально возможная евкл. дистанция между ключевыми точками
  int64_t __img_counter = 1;

  std::mutex __transform_param; //!< Мьютекс для критической зоны "Transform Parameters"
  std::mutex __transform_new_param; //!< Мьютекс для критической зоны "New Transform Parameters"
  std::mutex __wait_picture; //!< Мьютекс, которые блокирует потоки, если нет новых картинок
  bool __new_pict_notif = false; //!< переменная, отвечающая за появление новых картинок (от ложных пробуждений)
  std::condition_variable __new_pict_check; //!< условная переменная для пробуждения потоков, когда приходит новая картинка

  std::vector<std::future<void>> __threads; //!< потоки для получения параметров движения камеры,
                                      //!< для компенсации движения
                                      //!< и для генерации стабилизированного изображения
  bool __start_work = true;
  bool __start_return_stab = false;
  std::deque<TransformParam> __prev_to_cur_transform = {}; //!< буфферы параметров межкадровых трансформаций для обмена между потоками
  std::deque<TransformParam> __new_prev_to_cur_transform = {};

  std::deque<StabSnapshot> __last_gotten_snapshot; //!< последнее переданное Стабилизатору изображение
  std::deque<StabSnapshot> __for_stab_snapshot; //!< кадры для стабилизации
  StabSnapshot __snapshot_for_stab;
  int64_t __shooter_iter = 0;

  cv::Mat __last_grey_img;
  cv::Mat __pre_last_grey_img;
  StabSnapshot __ready_snapshot; //!< стабилизированное последнее изображение
  StabSnapshot __pre_snapshot; //!< прошлое стабилизированное изображение для оценки алгоритма

  double ITF = 0.0; //!< финальная накопленная оценка стабилизации
  int ITFMeasCount = 0; //!< количество замеров
};
