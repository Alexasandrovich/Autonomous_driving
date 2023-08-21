//// этот файл в utf-8
////! @file tram_filter.h
////! @brief Фильтр движения для трамваев, основанный на расширенном фильтре Калмана
////! @author Арсентьев А.
////! @date 2020

//#pragma once
//#include <coords/kalman_filter.h>
//#include <worldobject/worldobject.h>
//#include <worldobjectpool/motionfilter.h>
//#include <worldobjectpool/filter_state.h>
//#include <worldobjectpool/objectbehaviour.h>

//struct TramFilterState : public AFilterState
//{
//  double yaw = 0.0;
//  double v = 0.0;

//  Eigen::VectorXd toVector() override;
//};

////! @brief Мультигипотезный фильтр Калмана для Трамвая
////! Три модели двежия - стоим, едем по прямолинейному участку
////! и двигаемся по дуге. Переключение между моделями
////! основанно на тригерах - пороги изменения матрицы поворота,
////! обственные значения матрицы дисперсии (когда стоим на месте
////! то облако точек x, y +- равномерное и маленькое)
//class MultiHypothTramFilter : public MotionFilter{
//private:
//  struct Measurement {
//    double r = 0.0; //!< радиус путей движения
//    double yaw = 0.0; //!< угол направления

//    // измерения визуальной одометрии
//    double x_ego = 0.0;
//    double y_ego = 0.0;
//    double yaw_ego = 0.0;
//    long msec = 0;
//  };
//public:
//  MultiHypothTramFilter();
//  static std::unique_ptr<MultiHypothTramFilter> create();
//  TramFilterState getState() override;
//  TramFilterState processObservation(const ObjectObservation& obs, ObjectAction action,
//                                     int64_t locationId) override;
//  Measurement prepareMeasurement(const ObjectObservation& obs);
//  void processMeasurement(const Measurement& meas);


//private:
//  // TODO: подобрать кол-во параметров
//  int nStateParams = 5; // ???
//  int nMeasurementParams = 3; // ???
//  int nExtParams = 3; // ??/

//  ExtendedKalmanFilter filter;
//  ObjectAction currentAction = ObjectAction::UNPREDICTABLE_BEHAVIOUR;

//  // сброс состояния фильтра
//  void resetState(const Measurement& meas);
//}




