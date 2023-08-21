//#include "tram_filter.h"
//#include <memory>

//MultiHypothTramFilter::MultiHypothTramFilter()
//{
//  filter.x_ = Eigen::VectorXd::Zero(nStateParams);
//  filter.P_ = Eigen::MatrixXd::Identity(nStateParams, nStateParams);
//  filter.F_ = Eigen::MatrixXd::Identity(nStateParams, nStateParams);
//  filter.Q_ = Eigen::MatrixXd::Identity(nStateParams, nStateParams);

//  filter.R_ = Eigen::MatrixXd::Identity(nMeasurementParams, nMeasurementParams);
//  filter.R_ << r_rr_, 0, 0, 0, r_phiphi_, 0, 0, 0, r_yawyaw_;

//  filter.H_ = Eigen::MatrixXd::Identity(nMeasurementParams, nStateParams);

//  filter.has_heading_angle = true;

//  aliveTime = 0;
//  didInit = false;
//}

//static std::unique_ptr<MultiHypothTramFilter> MultiHypothTramFilter::create()
//{
//  return std::unique_ptr<MultiHypothTramFilter>();
//}

//Eigen::VectorXd TramFilterState::toVector()
//{
//  Eigen::VectorXd vec(4);
//  vec << x, y, yaw, v;

//  return vec;
//}

//TramFilterState MultiHypothTramFilter::getState()
//{
//  return {}; // todo: return state correctly
//}

//TramFilterState MultiHypothTramFilter::processObservaction(const ObjectObservation& obs,
//                                                         ObjectAction action,
//                                                         int64_t locationId)
//{
//  /* параметры трамвая - длина, положение колесной пары и прочее
//   *
//   *
//   */
//  MultiHypothTramFilter::Measurement meas = prepareMeasurement(obs);
//  if(currentAction != action) // триггер переключения режима фильтра
//  {
//    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(nStateParams, nStateParams);
//    filter.P_ = P;
//    reset_state(meas);
//    filteringTime = 0;
//  }

//  currentAction = action;
//  processMeasurement(meas);

//  return getState();
//}

//MultiHypothTramFilter::Measurement
//MultiHypothTramFilter::prepareMeasurement(const ObjectObservation& obs)
//{
//  Measurement meas;
//  meas.msec = obs.grabMsec;
//  if(/* одометр заработал */)
//  {
//    /* Affine3d (???) VO_meas = ...;
//     * meas.x_ego = VO_meas.x;
//     * meas.y_ego = VO_meas.y;
//     * meas.yaw_ego = VO_meas.yaw;     *
//     * meas.r = cv::norm(obs.<кривизна траектории>);
//     */
//  }
//  return meas;
//}

//void MultiHypothTramFilter::resetState(const Measurement &meas)
//{
//  filter.x_(0) = 0.0; // meas.x_ego + meas.r * cos(meas.phi + meas.yaw_ego);
//  filter.x_(1) = 0.0; // meas.y_ego + meas.r * sin(meas.phi + meas.yaw_ego);
//  filter.x_(2) = 0.0; // meas.yaw;
//  filter.x_(3) = 0.0;
//  filter.x_(4) = 0.0;

//  /*th::fixYaw(filter.x_[2]);*/
//}

//void MultiHypothTramFilter::processMeasurement(const Measurement& meas)
//{

//}
