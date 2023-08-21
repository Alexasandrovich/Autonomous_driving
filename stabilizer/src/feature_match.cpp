#include "stabilizer/feature_match.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
using namespace std;
using namespace cv;

void match_features(
    cv::Mat &descSource, cv::Mat &descRef,
    std::vector<cv::DMatch> &matches, std::string descriptorType,
    std::string matcherType, std::string selectorType)
{
  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  if (matcherType.compare("MAT_BF") == 0) {
      int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
      matcher = cv::BFMatcher::create(normType, crossCheck);
  }
  else if (matcherType.compare("MAT_FLANN") == 0) {
      if (descSource.type() != CV_32F) {
          // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
          descSource.convertTo(descSource, CV_32F);
          descRef.convertTo(descRef, CV_32F);
      }
      // FLANN matching
      matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  }

  // perform matching task
  if (selectorType.compare("SEL_NN") == 0) {
      // nearest neighbor (best match)
      matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
  }
  else if (selectorType.compare("SEL_KNN") == 0) {
      // k nearest neighbors (k=2)
      vector<vector<cv::DMatch>> knn_matches;
      matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches

      // filter matches using descriptor distance ratio test
      double minDescDistRatio = 0.8;
      for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it) {

          if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance) {
              matches.push_back((*it)[0]);
          }
      }
  }
}

void desc_keypoints(vector<cv::KeyPoint> &keypoints, const cv::Mat &img, cv::Mat &descriptors, string descriptorType) {
	// select appropriate descriptor
	cv::Ptr<cv::DescriptorExtractor> extractor;
	if (descriptorType.compare("BRISK") == 0){
			int threshold = 30;        // FAST/AGAST detection threshold score.
			int octaves = 3;           // detection octaves (use 0 to do single scale)
			float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

			extractor = cv::BRISK::create(threshold, octaves, patternScale);
	}
	else if (descriptorType.compare("BRIEF") == 0) {
			extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
	}
	else if (descriptorType.compare("ORB") == 0) {
			extractor = cv::ORB::create();
	}
	else if (descriptorType.compare("FREAK") == 0) {
			extractor = cv::xfeatures2d::FREAK::create();
	}
	else if (descriptorType.compare("AKAZE") == 0) {
			extractor = cv::AKAZE::create();
	}
	else if (descriptorType.compare("SIFT") == 0) {
			extractor = cv::xfeatures2d::SiftDescriptorExtractor::create();
	}
	else {
			throw invalid_argument(descriptorType + "is not a valid type!!");
	}
	// perform feature description
	extractor->compute(img, keypoints, descriptors);
}
