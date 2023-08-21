//! @file    prj.sandbox/stabilizer/feature_match.h
//! @author  Александр Арсентьев
//! @brief   функции для работа с ORB-фичами
//! @date    16.04.2021
#include <opencv2/core/core.hpp>
#include <string>
#pragma once

//! @brief сопоставление ключевых точек с двух последовательных изображений
void match_features(cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches,
                    std::string descriptorType, std::string matcherType, std::string selectorType);

//! @brief расчёт дескрипторов для уникального сопоставления ключевых точек
void desc_keypoints(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, cv::Mat &descriptors, std::string descriptorType);
