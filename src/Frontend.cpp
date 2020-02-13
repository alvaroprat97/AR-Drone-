/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2016
 *      Author: sleutene
 */

#include <arp/Frontend.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace arp {

Frontend::~Frontend() {
  if(camera_ != nullptr) {
    delete camera_;
  }
}

void Frontend::setCameraParameters(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2)
{
  if(camera_ != nullptr) {
    delete camera_;
  }
  camera_ = new arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
          imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV, arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2));
  camera_->initialiseUndistortMaps();
}

// the undistorted camera model used for the estimator (later)
arp::cameras::PinholeCamera<arp::cameras::NoDistortion>
    Frontend::undistortedCameraModel() const {
  assert(camera_);
  return camera_->undistortedPinholeCamera();
}

int Frontend::detect(const cv::Mat& image, DetectionVec & detections)
{

  detections.clear();
	cv::Mat grayImage; // Create a dummy variable for the new image
  cv::Mat correctedImage
  cv::cvtColor(image, grayImage, CV_BGR2GRAY); // Convert 3 grayScale
	camera_->undistortImage(grayImage, correctedImage); // Undistort the image and pass it to correctedImage variable

  // Retreive tags
  std::vector<AprilTags::TagDetection> aprilTags = tagDetector_.extractTags(correctedImage); // Extract tags

  // Loop over tags and populate detections vector

	for (auto const& detection_raw: detections_raw) {

            int detectedID = detection_raw.id;
            if (idToSize_.find(detection_raw.id) == idToSize_.end()){
            continue; // ID is not found
            } else {

            // Create a Detection Instance
            Detection detection;
            detection.ID = detectedID;

            Eigen::Matrix4d transform = detection_raw.getRelativeTransform(idToSize_[detectedID],
                        camera_->focalLengthU(), camera_->focalLengthU(),
                        camera_->imageCenterU(), camera_->imageCenterV());

            detection.T_CT = kinematics::Transformation(transform);

            const std::pair<float ,float> *pair = detection_raw.p;
            Eigen::Matrix<double, 2, 4> point_matx;

            for (int j = 0; j<3; j++){
                point_matx(0,j) = pair[j].first;
                point_matx(1,j) = pair[j].second;
            }

           detection.points = point_matx;
           detections.push_back(detection);
        }
      }

  return detections.size();
}
/*
  // Clear detections
detections.clear();
//undistort the input image and transform to grayscale
cv::Mat undistorted_grey_image;
cv::Mat distorted_grey_image;
cv::cvtColor(image, distorted_grey_image, CV_BGR2GRAY);
camera_->undistortImage(distorted_grey_image,undistorted_grey_image);
double fu = camera_->undistortedPinholeCamera().focalLengthU();
double fv = camera_->undistortedPinholeCamera().focalLengthV();
double cu = camera_->undistortedPinholeCamera().imageCenterU();
double cv = camera_->undistortedPinholeCamera().imageCenterV();

//Extract AprilTags ie. get raw detactions using tagDetector_
std::vector<AprilTags::TagDetection> rawdetections = tagDetector_.extractTags(undistorted_grey_image);


for (auto const& rawdetection: rawdetections)
{
    int detected_ID = rawdetection.id;
    if (idToSize_.find(detected_ID) == idToSize_.end())
    {
      // id not found
      continue;
    } else {
      float targetSize = idToSize_[detected_ID];
      Eigen::Matrix4d transform = rawdetection.getRelativeTransform(targetSize,fu, fv, cu, cv);
      Detection detection; // maybe add arp:: etc.
      //std::cout << "transform T_CT:" << transform;
      detection.T_CT = kinematics::Transformation(transform);
      //std::cout << "deection T_CT:" << kinematics::Transformation(transform).T();

      const std::pair<float, float> *pi = rawdetection.p;
      Eigen::Matrix<double, 2, 4> matrix;

      matrix(0,0) = pi[0].first;
      matrix(0,1) = pi[1].first;
      matrix(0,2) = pi[2].first;
      matrix(0,3) = pi[3].first;
      matrix(1,0) = pi[0].second;
      matrix(1,1) = pi[1].second;
      matrix(1,2) = pi[2].second;
      matrix(1,3) = pi[3].second;
      //std::cout<< "Matrix: " << matrix;
      detection.points = matrix;
      detection.id = detected_ID;
      detections.push_back(detection);
    }
}
return detections.size();
}
*/
bool Frontend::setTarget(unsigned int id, double targetSizeMeters) {
  idToSize_[id] = targetSizeMeters;
  return true;
}

}  // namespace arp
