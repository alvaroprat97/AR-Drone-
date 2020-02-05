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

	cv::Mat correctedImage; // Create a dummy variable for the new image 
	
	camera_->undistortImage(image, correctedImage); // Undistort the image and pass it to correctedImage variable 
	
	cv::cvtColor(image, correctedImage, CV_BGR2GRAY); // Convert 3 grayScale
	
	std::vector<AprilTags::TagDetection> aprilTags = tagDetector_.extractTags(correctedImage); // Extract tags
	
	// Loop over tags and populate detections vector

        int counter = 0;

	for (int i=0; i<aprilTags.size(); i++) {

            auto detection = aprilTags[i];

            if (idToSize_.find(detection.id) == idToSize_.end()){
            continue; // ID is not found

            } else {

            detections[counter].id = detection.id;
            // auto dist = camera_->getDistortion();
            Eigen::Matrix4d transform = detection.getRelativeTransform(idToSize_[detection.id],
                        camera_->focalLengthU(), camera_->focalLengthU(),
                        camera_->imageCenterU(), camera_->imageCenterV());

            detections[counter].T_CT = kinematics::Transformation(transform);

            const std::pair<float ,float> *pair = detection.p;
            Eigen::Matrix<double, 2, 4> point_matx;

            for (int j = 0; j<3; j++){
                point_matx(0,j) = pair[j].first;
                point_matx(1,j) = pair[j].second;
            }

           detections[counter].points = point_matx;
           counter ++;
        }
      }
	
  return counter; // TODO: number of detections...
}

bool Frontend::setTarget(unsigned int id, double targetSizeMeters) {
  idToSize_[id] = targetSizeMeters;
  return true;
}

}  // namespace arp

