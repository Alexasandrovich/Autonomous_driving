// этот файл в utf-8
//! @file precompiled.h
//! @brief набор функций для вычисления рассчёта визуальной одометрии
//! @author Постников В.
//! @date 2020
#pragma once

#include <cassert>
#include <iostream>
#include <iomanip>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/optflow.hpp>


#if (CV_VERSION_MAJOR < 4)
namespace cv_dis = cv::optflow;
#else
namespace cv_dis = cv;
namespace cv {
inline cv::Ptr<cv_dis::DISOpticalFlow> createOptFlow_DIS(int preset)
{
  return cv_dis::DISOpticalFlow::create(preset);
}
};
#endif


//! @brief структура для замера времени работы функций и методовы
struct Timer
{
  long long start = 0;
  std::string title;
  double msec() { return (1000 * (cv::getTickCount() - start)) / cv::getTickFrequency(); }
  Timer(std::string title): title(title) { start = cv::getTickCount(); }
  ~Timer() { std::cout << title << " " << msec() << " msec" << std::endl << std::endl; }
};

//! @brief функция для упрощения использования cv::resize()
//! @param [in] src изображение на входе
//! @param [in] ratio параметры изменения размеров изображения
//! @retval Mat изменённое изображение
inline cv::Mat resize(cv::Mat& src, double ratio) {
    cv::Mat res;
    cv::resize(src, res, cv::Size(), ratio, ratio);
    return res;
}

//! @brief функция для упрощения использования cv::cvtColor()
//! @param [in] src цветное изображение на входе
//! @retval Mat изменённое серое изображение
inline cv::Mat grey(cv::Mat& src) {
    cv::Mat res;
    cvtColor(src, res, cv::COLOR_BGR2GRAY);
    return res;
}

//! @brief функция для упрощения использования cv::minMaxIdx()
//! @param [in] src одноканальное изображение на входе
//! @retval double максимальное значение
inline double get_max(cv::Mat1f& src) {
    double mi = 0;
    double ma = 0;
    cv::minMaxIdx(src, &mi, &ma);
    return ma;
}

//! @brief функция для упрощения использования cv::minMaxLoc()
//! @param [in] src одноканальное изображение на входе
//! @retval cv::Point точка с максимальным значением
inline cv::Point get_max_pos(cv::Mat1f& src)
{
  cv::Point ptmi, ptma;
  double mi = 0; double ma = 0;
  minMaxLoc(src, &mi, &ma, &ptmi, &ptma);
  return ptma;
}
