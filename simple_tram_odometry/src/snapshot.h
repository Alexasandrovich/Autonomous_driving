// этот файл в utf-8
//! @file snapshot.h
//! @brief набор структур для привязки к кадру всех необходимых
//! для расчёта визуальной одометрии данных
//! @author Арсентьев А.
//! @date 2020
#pragma once
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "precompiled.h"
#include <coords/device.h>

//! @brief расчитанные данные по одному кадру с помощью визуальной одометрии
struct SFrame
{
    //! @brief тривиальный конструктор
    SFrame() = default;

    //! @brief констуктор для инициализации параметров камеры и BRG-изображения
    SFrame(Camera& cam_, cv::Mat3b& bgr_): cam(cam_), bgr(bgr_){
        gre = grey(bgr);
    }

    Camera cam; //!< параметры камеры
    cv::Mat3b     bgr; //!< source bgr image
    cv::Mat1b     gre; //!< source grey image
    cv::Mat1f     sigma;
    cv::Mat1b     mask_low_sigma; //!< 0 заполнены пиксели с низким уровнем сигнала

    // previous-to-current ~ current-to-next
    int           ipre = -1; //!< индекс фрейма, из которого был расчитан pre2cur_optflow
    cv::Mat2f     pre2cur_optflow; //!< previous-to-current frame optical flow:  pre[xy] => cur[xy+flow[xy]]
    cv::Matx33f   pre2cur_ecc_homography = cv::Matx33f::eye(); //!< previous-to-current frame homography  cv::Matx23f
    cv::Matx33d E;  //!< essential matrix (pre=>cur)
    cv::Matx33d R = cv::Matx33d::eye(); //!< rotation matrix  (pre=>cur) Xcur = R*Xpre+t  (this=>next) camera pose
    cv::Vec3d t = {};  //!< смещение (pre=>cur) X_cur = R*X_pre+t  (this=>next)

    // t = t_step * t_unit
    cv::Vec3d t_unit = {}; //!< направление translation  (pre=>cur) X_cur = R*X_pre + t_step*t_unit  (this=>next)
    double    t_step = 0; //!< "длина шага" translation  (pre=>cur) X_cur = R*X_pre + t_step*t_unit  (this=>next)
};

//! @brief заготовленные данные по одному кадру для расчёта визуальной одометрии
struct Snapshot
{
    //! @brief тривиальный конструктор
    Snapshot() = default;

    //! @brief констуктор для инициализации кадра
    //! @param [in] ipng_ номер кадра в контексте перечня всех эпизодов одного сезона
    //! @param [in] time_ временная метка для текущего кадра
    //! @param [in] cam_ параметры камеры
    //! @param [in] gt_pose_ GT-поза из локалайзера
    //! @param [in] episode_ номер эпизода текущего сезона
    //! @param [in] R_gt_ GT поворот
    //! @param [in] t_gt_ внешние параметры камеры - сдвиг
    //! @param [in] bgr_ кадра в формета BGR
    //! @param [in] yaw_gt угол поворота, полученный из локалайзера
    //! @param [in] roll_gt угол поворота, полученный из локалайзера
    //! @param [in] pitch_roll угол поворота, полученный из локалайзера
    Snapshot(int ipng_, int32_t time_, Camera& cam_, cv::Affine3d& gt_pose_, int episode_,
             cv::Matx33d& R_gt_, cv::Vec3d& t_gt_, cv::Mat3b& bgr_, double yaw_gt_, double roll_gt_, double pitch_gt_):
        ipng(ipng_), time(time_), le(cam_, bgr_), gt_pose(gt_pose_), iseq(episode_),
        ex_R(R_gt_), ex_t(t_gt_), yaw_gt(yaw_gt_), roll_gt(roll_gt_), pitch_gt(pitch_gt_){}

    //! @brief проверка на пустоту кадра
    bool empty() { return time < 0.; }

    int ipng = 0; //!< номер кадра в контексте перечня всех эпизодов одного сезона
    double time = -1.; //!< временная метка для текущего кадра
    SFrame le; //!< расчитанные данные по одному кадру с помощью визуальной одометрии
    SFrame ri; //!< не используется
    cv::Mat2f le2ri; //!< Не используется. left-to-right оптический поток (0-channel is disparity)
    cv::Mat2f ri2le; //!< Не используется right-to-left оптический поток (0-channel is disparity)
    cv::Affine3d gt_pose; //!< GT-поза из локалайзера
    cv::Matx33d ex_R; //!< GT - поворот
    cv::Vec3d ex_t; //!< GT - сдвиг
    double translation = 0; //!< расчитанное смещение (длина вектора) из предыдущего кадра
    int iseq = 0; //!< номер эпизода текущего сезона
    std::string sequences_folder; //!< путь к файлам
    double yaw_gt; //!< угол поворота, полученный из локалайзера
    double roll_gt; //!< угол поворота, полученный из локалайзера
    double pitch_gt; //!< угол поворота, полученный из локалайзера

    double yaw_eval = 0; //!< угол поворота, полученный в ходе алгоритма
    double roll_eval = 0; //!< угол поворота, полученный в ходе алгоритма
    double pitch_eval = 0; //!< угол поворота, полученный в ходе алгоритма

};

//! @brief глобальная бинаризация изображение на пиксели с низким уровнем сигнала
//! @param [in] sfr расчитанные данные по одному кадру с помощью визуальной одометрии
inline cv::Mat1b binarize_low_sigma_zones(SFrame& sfr)
{
  cv::Mat1f gref = sfr.gre;
  cv::Mat1f avef; GaussianBlur(gref, avef, cv::Size(5, 5), 0);
  cv::Mat1f diff; absdiff(gref, avef, diff);
  cv::Mat1f sigf; GaussianBlur(diff, sigf, cv::Size(5, 5), 0);
  GaussianBlur(diff, sfr.sigma, cv::Size(5, 5), 0);

  cv::Mat1b sigb = sigf * 10;
  cv::Mat1b mask_low_sigma;
  threshold(sigb, mask_low_sigma, 0, 255, cv::THRESH_OTSU);
  return mask_low_sigma;
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

//! @brief отписывание файлов cv::Mat1f
//! @param [in] filename имя файла
//! @param [in] img одноканальное изображение
inline bool imwrite1f(std::string& filename, cv::Mat1f& img)
{
    cv::Mat1b m(img.rows, img.cols * 4, img.data);
    std::vector<int> parms;    parms.push_back(cv::IMWRITE_PNG_COMPRESSION); parms.push_back(0);
    return cv::imwrite(filename, m, parms);
}

//! @brief циклическая гистограмма оценок скоростей, полученных к i-му кадру
class SpeedMap
{
public:
    //! @brief отписывание гистограммы скоростей для текущего эпизода
    //! @param [in] output_folder путь к папке для сохранениия гистограммы
    //! @param [in] seq_num номер эпизода
    bool write(std::string output_folder, int seq_num)
    {
      std::string s1 = output_folder + cv::format("/hist_orig.seq%02d.1f.png", seq_num);
      bool res1 = imwrite1f(s1, hist_orig);
      std::string s2 = output_folder + cv::format("/hist_ema1.seq%02d.1f.png", seq_num);
      bool res2 = imwrite1f(s2, hist_ema1);
      return res1 && res2;
    }

public:
  int max_displacement = 1024; //!< ~51.2 метра в секунду, больше пока не рассматриваем
  int cyckle = 1200; //!< 120 секунд памяти в циклическом буфере фреймов
  cv::Mat1f hist_orig =
          cv::Mat1f(cv::Size(max_displacement, cyckle), 0.);  //!< строка -- номер фрейма,
                                                              //!< столбец -- оценка скорости в единице измерения 1 см за 0.1 секунды
  cv::Mat1f hist_ema1 = cv::Mat1f(cv::Size(max_displacement, cyckle), 0.);  //!< фильтрованная гистограмма по EMA
};

