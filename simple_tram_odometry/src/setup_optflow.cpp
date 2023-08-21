// этот файл в utf-8
#include "precompiled.h"
#pragma hdrstop
#include "setup_optflow.h"

using namespace std;
using namespace cv;
using namespace cv::optflow;

cv::Ptr<cv_dis::DISOpticalFlow> setup_dis_optflow()
{
  auto optflower_DIS = createOptFlow_DIS(DISOpticalFlow::PRESET_ULTRAFAST);
  optflower_DIS->setFinestScale(0);
  return optflower_DIS;
}

