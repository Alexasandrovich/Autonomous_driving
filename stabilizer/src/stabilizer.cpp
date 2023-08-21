//! @file    prj.sandbox/stabilizer/stabilizer.h
//! @author  Алексадндр Арсентьев

#include <iostream>
#include <cassert>
#include <cmath>
#include <thread>
#include <chrono>

#include <opencv2/features2d.hpp>

#include "stabilizer/feature_match.h"
#include "stabilizer/stabilizer.h"
#include "ar10/ticker.h"

using namespace std;
using namespace cv;
using chrono::milliseconds;
using chrono::duration;
using chrono::duration_cast;
using Ltp = ar::lifepolicies::FIFO<1000, 20>;
namespace tags {struct StabImgTag;}
using StabImg = ar::TaggedObj<Mat, tags::StabImgTag>;
//#define UseRTDB

Stabilizer::Stabilizer(smoothing_method sm)
{
  smooth_method = sm;
	init_ORB();
}

double Stabilizer::getPSNR(const StabSnapshot& I1, const StabSnapshot& I2)
{
  Mat s1;
  absdiff(I1.stabImg.value(), I2.stabImg.value(), s1);       // |I1 - I2|
  s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
  s1 = s1.mul(s1);           // |I1 - I2|^2
  Scalar s = sum(s1);        // sum elements per channel
  double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels
  if( sse <= 1e-10) // for small values return zero
    return 0;
  else{
    double mse  = sse / (double)(I1.stabImg->channels() * I1.stabImg->total());
    double psnr = 10.0 * log10((255 * 255) / mse);
    return psnr;
  }
}


void Stabilizer::collect_stat(StabSnapshot& curSs)
{
  if(!__pre_snapshot.empty()) // сделаем оценку полученного алгоритма
  {
    double PSNRNow = getPSNR(__pre_snapshot, curSs);
    ITF += PSNRNow;
    ITFMeasCount++;
    curSs.PSNR = PSNRNow;
  }

  // сохраним в историю стабилизаци
  __pre_snapshot = curSs;
}

bool Stabilizer::init_ORB()
{
	const size_t MAX_FEATURES = 100;

	//  scaleFactor==2 означает классическую пирамиду масштабов, где каждый следующий уровень имеет в 4 раза меньше пикселей,
	//  чем предыдущий, но такой большой масштабный коэффициент значительно ухудшит показатели matching характеристик.
	//  С другой стороны, слишком близкий к 1 масштабный фактор будет означать,
	//  что для покрытия определенного диапазона масштаба понадобится больше уровней пирамиды,
	//  и поэтому скорость будет хуже. В общем, либо будет страдать скорость, либо качество matching
	const float SCALE_FACTOR = 1.2f;
	const size_t nLEVELS = 4; // кол-во уровней в пирамиде масштабов

	//  Это размер границы, на которой признаки не обнаруживаются. Он должен приблизительно соответствовать параметру patchSize.
	const size_t EDGE_THRHD = 0;
	const size_t FirstLEVEL = 0; // если не будет хватать скорости, то можно будет начать с 1 уровня
	//   Количество точек, которые используются для выделения ориентированного BRIEF дескриптора
	//  берём WTA_K рандомных точек около предполагаемого дискриптора (seed фиксированный)
	const size_t WTA_K = 3;
	const int ScoreTYPE = ORB::HARRIS_SCORE;// алгоритм оценивания (для сортировки) признаков (альтернатива - FAST_SCORE: быстрее, но менее стабильно)
	const size_t PatchSize = 71;  //  размер патча, используемого ориентированным BRIEF дескриптором.
	const int fast_Treshold = 20; // можно будет использовать для увеличения скорости при использовании FAST_SCORE

	try{
		__orb = ORB::create(MAX_FEATURES, SCALE_FACTOR, nLEVELS,
							EDGE_THRHD, FirstLEVEL, WTA_K,
							ScoreTYPE, PatchSize, fast_Treshold);
	} catch(...)
	{
		cerr << "Can't init ORB-detector" << "\n";
		return false;
	}
	return true;
}

bool Stabilizer::detect_ORB_kpts(const Mat& img_for_detect, vector<Point2f>& keypoints_res)
{
  if(!__orb.has_value()) return false;
  if(keypoints_res.size() > 0) keypoints_res.clear();
  std::vector<KeyPoint> full_keypoints_res;
  (__orb.value())->detect(img_for_detect, full_keypoints_res);
  for(const auto& pt : full_keypoints_res)
  {
    keypoints_res.emplace_back(move(pt.pt));
  }

  return true;
}

void Kalman_filter::update(const TransformParam& obs)
{
  ds_x = obs.ds_x;
  ds_y = obs.ds_y;
  dx = obs.dx;
  dy = obs.dy;
  da = obs.da;

  sx = obs.ds_x;
  sy = obs.ds_y;

  sum_trans_X += dx;
  sum_trans_Y += dy;
  sum_thetha += da;
  sum_scale_X += ds_x;
  sum_scale_Y += ds_y;

	if(updates == -1){ // не будем обновлять на первой итерации
		double frame_1_scale_X = scale_X;
		double frame_1_scale_Y = scale_Y;
		double frame_1_thetha = thetha;
		double frame_1_trans_X = trans_X;
		double frame_1_trans_Y = trans_Y;

		double frame_1_errscale_X = errscale_X + Q_scale_X;
		double frame_1_errscale_Y = errscale_Y + Q_scale_Y;
		double frame_1_errthetha = errthetha + Q_thetha;
		double frame_1_errtrans_X = errtrans_X + Q_trans_X;
		double frame_1_errtrans_Y = errtrans_Y + Q_trans_Y;

		double gain_scale_X = frame_1_errscale_X / (frame_1_errscale_X + R_scale_X);
		double gain_scale_Y = frame_1_errscale_Y / (frame_1_errscale_Y + R_scale_Y);
		double gain_thetha = frame_1_errthetha / (frame_1_errthetha + R_thetha);
		double gain_trans_X = frame_1_errtrans_X / (frame_1_errtrans_X + R_trans_X);
		double gain_trans_Y = frame_1_errtrans_Y / (frame_1_errtrans_Y + R_trans_Y);

		errscale_X = ( 1 - gain_scale_X ) * frame_1_errscale_X;
		errscale_Y = ( 1 - gain_scale_Y ) * frame_1_errscale_X;
		errthetha = ( 1 - gain_thetha ) * frame_1_errthetha;
		errtrans_X = ( 1 - gain_trans_X ) * frame_1_errtrans_X;
		errtrans_Y = ( 1 - gain_trans_Y ) * frame_1_errtrans_Y;

		scale_X = frame_1_scale_X + gain_scale_X * (sum_scale_X - frame_1_scale_X);
		scale_Y = frame_1_scale_Y + gain_scale_Y * (sum_scale_Y - frame_1_scale_Y);
		thetha = frame_1_thetha + gain_thetha * (sum_thetha - frame_1_thetha);
		trans_X = frame_1_trans_X + gain_trans_X * (sum_trans_X - frame_1_trans_X);
		trans_Y = frame_1_trans_Y + gain_trans_Y * (sum_trans_Y - frame_1_trans_Y);
	}

  diff_scale_X = scale_X - sum_scale_X;
  diff_scale_Y = scale_Y - sum_scale_Y;
  diff_trans_X = trans_X - sum_trans_X;
  diff_trans_Y = trans_Y - sum_trans_Y;
  diff_thetha = thetha - sum_thetha;
  updates++;
}

void Kalman_filter::predict()
{
  auto new_smoothed_mat = Mat(2,3,CV_64F);

  ds_x = ds_x + diff_scale_X;
  ds_y = ds_y + diff_scale_Y;
  dx = dx + diff_trans_X;
  dy = dy + diff_trans_Y;
  da = da + diff_thetha;

  new_smoothed_mat.at<double>(0,0) = sx * cos(da);
  new_smoothed_mat.at<double>(0,1) = sx * -sin(da);
  new_smoothed_mat.at<double>(1,0) = sy * sin(da);
  new_smoothed_mat.at<double>(1,1) = sy * cos(da);
  new_smoothed_mat.at<double>(0,2) = dx;
  new_smoothed_mat.at<double>(1,2) = dy;

  smoothed_mat.emplace_back(new_smoothed_mat);
}

bool Kalman_filter::correct_motion(const TransformParam& tp)
{
  if(abs(tp.dx) < 40 && abs(tp.dy) < 40 &&  tp.ds_x > 0.9 && tp.ds_x < 1.2 &&
     tp.ds_y > 0.9 && tp.ds_y < 1.2) return true;
  else return false;
}

Mat Kalman_filter::get_oldest_predict()
{
  updates--;
  predict();
  auto oldest_predict = smoothed_mat[0];
  smoothed_mat.pop_front();
  return oldest_predict;
}

pair<vector<Point2f>, vector<Point2f>> Stabilizer::match_found_ORB_kpts(const Mat &prev_img,
                                                                        const Mat &cur_img,
                                                                        vector<KeyPoint> &full_keypoints_data_prev,
                                                                        vector<KeyPoint> &full_keypoints_data_cur)
{
  vector<DMatch> matches;
  string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
  string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
  string selectorType = "SEL_KNN";      // SEL_NN, SEL_KNN

  Mat desc_pre = Mat(), desc_cur = Mat();
  desc_keypoints(full_keypoints_data_prev, prev_img, desc_pre, "ORB");
  desc_keypoints(full_keypoints_data_cur, cur_img, desc_cur, "ORB");
  match_features(desc_pre, desc_cur, matches, descriptorType, matcherType, selectorType);

  vector<Point2f> keypoints_prev, keypoints_cur; // получившиеся совпадения
  for(size_t i = 0; i < matches.size(); i++)
  {
    keypoints_cur.emplace_back(full_keypoints_data_cur[matches[i].trainIdx].pt);
    keypoints_prev.emplace_back(full_keypoints_data_prev[matches[i].queryIdx].pt);
  }
  /*drawMatches(prev_img, full_keypoints_data_prev, cur_img, full_keypoints_data_cur, matches, res,
              cv::Scalar::all(-1), cv::Scalar::all(-1),
              vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);*/
  return make_pair(move(keypoints_cur), move(keypoints_prev));
}

transformationMethod Stabilizer::rigid_or_affine(Mat rigid, Mat partial_affine, vector<Point2f> prev_keypoints)
{
  // TODO: аккуратнее переделать перерасчёт новых точек
  vector<Point2f> new_points_rigid;
  vector<Point2f> new_points_partial_affine;

  for (int i = 0; i < prev_keypoints.size(); i++){
    double _xR = rigid.at<double>(0, 0) * prev_keypoints[i].x + rigid.at<double>(0, 1) * prev_keypoints[i].y + rigid.at<double>(0, 2);
    double _yR = rigid.at<double>(1, 0) * prev_keypoints[i].x + rigid.at<double>(1, 1) * prev_keypoints[i].y + rigid.at<double>(1, 2);
    double _wR = rigid.at<double>(2, 0) * prev_keypoints[i].x + rigid.at<double>(2, 1) * prev_keypoints[i].y + 1;
    double _xA = partial_affine.at<double>(0, 0) * prev_keypoints[i].x + partial_affine.at<double>(0, 1) * prev_keypoints[i].y + partial_affine.at<double>(0, 2);
    double _yA = partial_affine.at<double>(1, 0) * prev_keypoints[i].x + partial_affine.at<double>(1, 1) * prev_keypoints[i].y + partial_affine.at<double>(1, 2);
    double _wA = partial_affine.at<double>(2, 0) * prev_keypoints[i].x + partial_affine.at<double>(2, 1) * prev_keypoints[i].y + 1;

    // переход к однородным координатам ключевых точек
    double xR = _xR / _wR;
    double yR = _yR / _wR;
    double xA = _xA / _wA;
    double yA = _yA / _wA;
    Point2f ptR(xR, yR);
    Point2f ptA(xA, yA);
    new_points_rigid.emplace_back(ptR);
    new_points_partial_affine.emplace_back(ptA);
  }

  // сравнение и выбор подходящего преобразования
  double avg_diff_rigid = 0;
  double avg_diff_affine = 0;
  for(int i = 0; i < prev_keypoints.size(); i++){
    avg_diff_rigid += sqrt((pow((prev_keypoints[i].x - new_points_rigid[i].x), 2) + pow((prev_keypoints[i].y - new_points_rigid[i].y), 2)));
    avg_diff_affine += sqrt((pow((prev_keypoints[i].x - new_points_partial_affine[i].x), 2) + pow((prev_keypoints[i].y - new_points_partial_affine[i].y), 2)));
  }
  avg_diff_rigid /= prev_keypoints.size();
  avg_diff_affine /= prev_keypoints.size();

  if (avg_diff_rigid > avg_diff_affine){
    return transformationMethod::AFFINE;
  }else if (avg_diff_rigid < avg_diff_affine){
    return transformationMethod::RIGID;
  }else
    return transformationMethod::NOT_RECOGNIZED;
}

void Stabilizer::motion_estimation(){
  // TODO: покрутить ROI

  Mat prev_grey;
  Mat cur_grey;
  Mat I = (Mat_<double>(3, 3) << (1, 0, 0, 0, 1, 0, 0, 0, 1)); // вырожденная трансформация
  Mat last_T = I;

  vector <uchar> status;
  vector <float> err;

  vector <Point2f> prev_keypoints, cur_keypoints;
  vector<Point2f> pre_final_kpts, cur_final_kpts;

  int countRigidTransform = 0, countAffineTransform = 0, counter = 0;
  transformationMethod currentMethod = RIGID;

  cvtColor(__last_gotten_snapshot.back().stabImg.value(), prev_grey, COLOR_BGR2GRAY); // самый первый кадр
  __last_gotten_snapshot.pop_front();

  while (true) {
    if(__last_gotten_snapshot.empty()){
      // если пусто, то останавливаем поток на некоторые время и пускаем заново
      unique_lock<mutex> locker(__wait_picture);
      while(!__new_pict_notif) __new_pict_check.wait(locker);
      continue;
    }

    Mat R, pA, T; // матрица поворота, аффинная матрица, сдвиг
    cvtColor(__last_gotten_snapshot.front().stabImg.value(), cur_grey, COLOR_BGR2GRAY);

    //Rect ROI(150, 0, 600, 200);

    Mat prev_grey_roi = prev_grey;//(ROI);
    Mat cur_grey_roi = cur_grey;//(ROI);
    if (__img_counter % __keypoint_retrack_frames == 0){
      // обновление признаков, по которым идёт трекинг
      calcOpticalFlowPyrLK(prev_grey_roi, cur_grey_roi, prev_keypoints, cur_keypoints, status, err);
      detect_ORB_kpts(prev_grey_roi, prev_keypoints);
    }
    // Если выше определенного порога, то выполнить вычисление OF,
    // если нет, то вставить матрицу идентичности в вектор transformParams, указывающий на отсутствие изменений
    if(prev_keypoints.size() > __keypoint_threshold){
      cur_keypoints.clear();
      calcOpticalFlowPyrLK(prev_grey_roi, cur_grey_roi, prev_keypoints, cur_keypoints, status, err);
      pre_final_kpts.clear();
      cur_final_kpts.clear();
      for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
          pre_final_kpts.emplace_back(prev_keypoints[i]);
          cur_final_kpts.emplace_back(cur_keypoints[i]);
        }
      }
      // выберем подходящее преобразование
      try{
        if (currentMethod == RIGID)
          R = estimateRigidTransform(pre_final_kpts, cur_final_kpts, false);
        else if (currentMethod == AFFINE)
          pA = estimateRigidTransform(pre_final_kpts, cur_final_kpts, true);

        // запомним все признаки с текущего кадра
        prev_keypoints = cur_keypoints;
        counter++;
      }
      catch (...){
        R = I;
        pA = I;
        currentMethod = RIGID;
        cout << "R or pA is invalid" << endl;
      }
      if (__img_counter % __switch_frame == 0){
        R = estimateRigidTransform(pre_final_kpts, cur_final_kpts, false);
        pA = estimateRigidTransform(pre_final_kpts, cur_final_kpts, true);
        if (!R.empty() && !pA.empty() && rigid_or_affine(R, pA, pre_final_kpts)
                == transformationMethod::AFFINE){
          currentMethod = AFFINE;
          countAffineTransform++;
        }
        else{
          currentMethod = RIGID;
          countRigidTransform++;
        }
      }
      if (currentMethod == RIGID){
        T = R; // false = rigid transform, no scaling/shearing
      }else if (currentMethod == AFFINE){
        T = pA;
      }
      // в редких случаях трансформация выходит вырожденной. В таком случае используем последнее известное хорошее преобразование
      if (T.data == NULL){
        last_T.copyTo(T);
      }
      // декомпозиция T
      double dx = T.at<double>(0, 2);
      double dy = T.at<double>(1, 2);
      double da = atan2(T.at<double>(1, 0), T.at<double>(0, 0)); // угол поворота

      // проверка на выбросы
      if (dx < -100 || dx > 100){ // валидация смещения
        dx = 0;
      }else
        last_T.at<double>(0, 2) = T.at<double>(0, 2);

      if (dy < -100 || dy > 100){
        dy = 0;
      }else
        last_T.at<double>(1, 2) = T.at<double>(1, 2);

      if (da > 0.7853 || da < -0.7853 /* == +- 90 градусов */){
          da = 0;
      }else
        T.copyTo(last_T);

      if (currentMethod == RIGID){
        std::unique_lock<mutex> _keyToTF(__transform_param);
        __prev_to_cur_transform.push_back(TransformParam(dx, dy, da, __rigid_scale, __rigid_scale));
      }else if (currentMethod == AFFINE){
        double ds_x = T.at<double>(0, 0) / cos(da);
        double ds_y = T.at<double>(1, 1) / cos(da);
        std::unique_lock<mutex> _keyToTF(__transform_param);
        __prev_to_cur_transform.push_back(TransformParam(dx, dy, da, ds_x, ds_y));
      }
    }else{
      // первый кадр
      std::unique_lock<mutex> _keyToTF(__transform_param);
      __prev_to_cur_transform.push_back(TransformParam(0, 0, 0, 1, 1));
      detect_ORB_kpts(prev_grey_roi, prev_keypoints);
      __img_counter++;

      __snapshot_for_stab = __last_gotten_snapshot.front();
      __last_gotten_snapshot.pop_front();
      prev_grey = cur_grey;
      cur_grey.release();
      continue;
    }

    __snapshot_for_stab = __last_gotten_snapshot.front();
    __last_gotten_snapshot.pop_front();
    prev_grey = cur_grey;
    cur_grey.release();

    if (!pre_final_kpts.empty())
      pre_final_kpts.clear();
    if (!cur_final_kpts.empty())
      cur_final_kpts.clear();

    __img_counter++;
    __new_pict_notif = false;
  }
}

void Stabilizer::motion_compensation(){
  while (true){
      switch (smooth_method){
        case smoothing_method::KALMAN:
          if(__prev_to_cur_transform.size() > 0)
          { // будем считать внутреннее состояние только после пары кадров
            std::unique_lock<mutex> _keyToTF(__transform_param); //Locks unprocessed parameters
            if(kf.correct_motion(__prev_to_cur_transform.back()))
            {
              kf.update(__prev_to_cur_transform.back());
              __prev_to_cur_transform.pop_front();
            }
          }
          break;
        case smoothing_method::MOVING_AVERAGE_WINDOW:
          if (__prev_to_cur_transform.size() ==  __smoothing_radius * 2){
            deque<TransformParam> Trajectory; // траектория всех кадров
            deque<TransformParam> smoothed_Trajectory;
            std::unique_lock<mutex> _keyToTF(__transform_param); //Locks unprocessed parameters
            // усреднение скользящим окном
            double a = 0.0;
            double x = 0.0;
            double y = 0.0;
            double s = 0.0;
            for (size_t i = 0; i < __smoothing_radius * 2; i++) {
              x += __prev_to_cur_transform[i].dx;
              y += __prev_to_cur_transform[i].dy;
              a += __prev_to_cur_transform[i].da;
              s += __prev_to_cur_transform[i].ds_x;
              Trajectory.emplace_back(TransformParam(x, y, a, s, s));
            }
            for (size_t i = 0; i < Trajectory.size(); i++) {
              double sum_x = 0.0;
              double sum_y = 0.0;
              double sum_a = 0.0;
              double sum_s = 0.0;
              int count = 0;
              for (int j = -15; j < 15; j++) {
                if (i + j >= 0 && i + j < Trajectory.size()) {
                  sum_x += Trajectory[i + j].dx;
                  sum_y += Trajectory[i + j].dy;
                  sum_a += Trajectory[i + j].da;
                  sum_s += Trajectory[i + j].ds_x;
                  count++;
                }
              }
              double avg_a = sum_a / count;
              double avg_x = sum_x / count;
              double avg_y = sum_y / count;
              double avg_s = sum_s / count;
              smoothed_Trajectory.push_back(TransformParam(avg_x, avg_y, avg_a, avg_s, avg_s));
              }
            // Будем коммулятативно учитывать все параметры трансформаций от кадра к кадру
            double da = 0.0;
            double dx = 0.0;
            double dy = 0.0;
            double ds = 0.0;

						for (size_t i = 0; i < __smoothing_radius * 2; i++) {
								double diff_x = smoothed_Trajectory[i].dx - Trajectory[i].dx;
								double diff_y = smoothed_Trajectory[i].dy - Trajectory[i].dy;
								double diff_a = smoothed_Trajectory[i].da - Trajectory[i].da;
								double diff_s = smoothed_Trajectory[i].ds_x - Trajectory[i].ds_x;

								dx = __prev_to_cur_transform[i].dx + diff_x;
								dy = __prev_to_cur_transform[i].dy + diff_y;
								da = __prev_to_cur_transform[i].da + diff_a;
								ds = __prev_to_cur_transform[i].ds_x + diff_s;

								// проверка на выбросы
								if (ds < 0.9 || ds > 1.2)
									ds = 1;
								if (i == __smoothing_radius)
									break;
							}
						std::unique_lock<mutex> _keyToNewTF(__transform_new_param);
						__new_prev_to_cur_transform.emplace_back(TransformParam(dx, dy, da, ds, ds));
						_keyToNewTF.unlock();
						__prev_to_cur_transform.pop_front();
						_keyToTF.unlock();
					}
					break;
			case smoothing_method::CUSTOM:
        break; // todo: set custom probility
      case smoothing_method::NOT_SET:
				break;
			default:
				break;
		}
	}
}

void Stabilizer::generate_stab_image(){
  while (true){
    switch (smooth_method) {
      case smoothing_method::MOVING_AVERAGE_WINDOW:
        if (!__new_prev_to_cur_transform.empty() && !__for_stab_snapshot.front().empty()){
          if(!__start_return_stab) {
              __for_stab_snapshot.erase(__for_stab_snapshot.begin(),
                                   __for_stab_snapshot.begin() + __smoothing_radius);
              __start_return_stab = true;
          }
          Mat T(2, 3, CV_64F);
          std::unique_lock<mutex> _keyToNewTP(__transform_new_param);
          TransformParam P = __new_prev_to_cur_transform.front();
          __new_prev_to_cur_transform.pop_front();
          _keyToNewTP.unlock();

          if (P.ds_x != __rigid_scale){
            T.at<double>(0, 0) = P.ds_x * cos(P.da);
            T.at<double>(0, 1) = P.ds_x * -sin(P.da);
            T.at<double>(1, 0) = P.ds_x * sin(P.da);
            T.at<double>(1, 1) = P.ds_x * cos(P.da);
          }
          else{
            T.at<double>(0, 0) = cos(P.da);
            T.at<double>(0, 1) = -sin(P.da);
            T.at<double>(1, 0) = sin(P.da);
            T.at<double>(1, 1) = cos(P.da);
          }
          T.at<double>(0, 2) = P.dx;
          T.at<double>(1, 2) = P.dy;

          Mat res;
          warpAffine(__for_stab_snapshot.front().stabImg.value(), res, T,
                     __for_stab_snapshot.front().stabImg->size());
          __ready_snapshot.stabImg = move(res);
         __for_stab_snapshot.pop_front();
        }

        break;
      case smoothing_method::KALMAN:
        if(kf.ready()) {
          std::unique_lock<mutex> _keyToNewTP(__transform_new_param);
          auto predict_motion = kf.get_oldest_predict();
          Mat res;
          warpAffine(__for_stab_snapshot.front().stabImg.value(), res, predict_motion,
                     __for_stab_snapshot.front().stabImg->size());
          __ready_snapshot.stabImg = move(res);
          __for_stab_snapshot.pop_front();
        }
        break;
      case smoothing_method::CUSTOM:
        break;
      case smoothing_method::NOT_SET:
        if(!__for_stab_snapshot.empty()){
          __ready_snapshot = __for_stab_snapshot.front();
          __for_stab_snapshot.pop_front();
        }
        break;
      default:
        break;
     }
  }
}

StabSnapshot Stabilizer::feed_image_and_try_get_stab_it(const Mat &img, long long msec)
{
  Ticker t("stabilizer", nullptr, 0, arlog::DEST_DEFAULT);
  // TODO: синхронизировать __last_gotten_img и __for_stab_img
  // TODO: будить потоки по надобности
  // TODO: убрать прыжки по кадрам, когда происходит обновление keypoints
  // TODO: добавить временные индексы к кадрам

#ifdef UseRTDB
  using db = ar::DB<Ltp, MsecIndex>;
  MsecIndex current_index = {msec};

  // будем обрабатывать не менее 10 кадров за последнюю секунду взятых с интервалами не менее 70 мс
  auto& table = db::table<StabImg>("forStab");
  db::insert(current_index, (img)); // добавим новый кадр с временной меткой

  auto res = table
    .filter([current_index](auto cr) { // фильтруем по "за последнюю секунду"
      const auto& index = cr.first;
      return index.grabMsec > (current_index.grabMsec - 1000) && index < current_index;
    })
    .filter([current_index](auto cr) { // фильтруем по "интервал не менее 70 мс"
      static MsecIndex prev;
      const auto& index = cr.first;
      bool res = false;
      if(index.grabMsec - prev.grabMsec > 70) {
        res = true;
        prev = current_index;
      }
      return res;
    })
    .collect();

  //if(res.size() < 1) cout << "waring: empty table!" << endl;
#endif

  // если всё ок, то берём кадр в использование
  StabSnapshot preStabSnapshot(img, msec);
  if(!img.empty())
  {
    if(smooth_method != smoothing_method::NOT_SET){
      __last_gotten_snapshot.emplace_back(preStabSnapshot);
      __for_stab_snapshot.emplace_back(preStabSnapshot);
      __shooter_iter++;
      if(__start_work){
        // запуск потоков
        __threads.emplace_back(std::async(std::launch::async, &Stabilizer::motion_estimation, this));
        __threads.emplace_back(std::async(std::launch::async, &Stabilizer::motion_compensation, this));
        __threads.emplace_back(std::async(std::launch::async, &Stabilizer::generate_stab_image, this));
        __start_work = false;

        return preStabSnapshot; // нужно хотя бы два изображения для стабилизации
      }

      __new_pict_notif = true;
      __new_pict_check.notify_one();

      if(!__ready_snapshot.empty())
      {
        collect_stat(__ready_snapshot);
        preStabSnapshot = __ready_snapshot; // готово -> можно выдавать
      }

    }else // вырожденный случай сделаем отдельно
    {
      collect_stat(preStabSnapshot);
    }
  }

  int offset = 20; // уберём чёрные полосы после стабилизации
  if(!preStabSnapshot.empty())
  {
    auto roi = Rect(offset, offset,
                    preStabSnapshot.stabImg->cols - 2 * offset,
                    preStabSnapshot.stabImg->rows - 2 * offset);
    preStabSnapshot.stabImg = (preStabSnapshot.stabImg.value())(roi);
  }else{ // если нет стабилизации, то всё равно обрежем края, чтобы размер всегда был постоянный
    auto roi = Rect(offset, offset,
                    img.cols - 2 * offset,
                    img.rows - 2 * offset);

    preStabSnapshot.stabImg = (preStabSnapshot.stabImg.value())(roi);
  }
  return preStabSnapshot;
}
