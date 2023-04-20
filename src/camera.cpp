#include "../include/camera.hpp"

CameraManager::CameraManager(ros::NodeHandle input){

    this->nh = input;

    this->rgbCameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    this->rgbDistortionCoeffs = cv::Mat::zeros(5, 1, CV_64F);


    // Subscribers
    this->subRgbImageRaw   = nh.subscribe("/rgb/image_raw", 1, &CameraManager::RgbImageRawCallback, this);
    this->subRgbImageComp  = nh.subscribe("/rgb/image_raw/compressed", 1, &CameraManager::RgbImageCompCallback, this);
    this->subRgbCameraInfo = nh.subscribe("/rgb/camera_info", 1, &CameraManager::RgbCameraInfoCallback, this);

}

void CameraManager::RosToCv(sensor_msgs::CompressedImage* input, cv::Mat* output){

    cv::Mat cvIm;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(*input, "bgr8");
    cv_ptr->image.copyTo(cvIm);

    output = &cvIm;

}

void CameraManager::RosToCv(sensor_msgs::Image* input, cv::Mat* output){

    cv::Mat cvIm;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(*input, "bgr8");
    cv_ptr->image.copyTo(cvIm);

    output = &cvIm;

}

void CameraManager::ExtractCameraCoefficients(){

    this->rgbCameraMatrix.at<double>(0,0) = this->rgbCameraInfo.K.at(0);
    this->rgbCameraMatrix.at<double>(0,2) = this->rgbCameraInfo.K.at(2);
    this->rgbCameraMatrix.at<double>(1,1) = this->rgbCameraInfo.K.at(4);
    this->rgbCameraMatrix.at<double>(1,2) = this->rgbCameraInfo.K.at(5);

    this->rgbDistortionCoeffs.at<double>(0,0) = this->rgbCameraInfo.D.at(0);
    this->rgbDistortionCoeffs.at<double>(0,1) = this->rgbCameraInfo.D.at(1);
    this->rgbDistortionCoeffs.at<double>(0,2) = this->rgbCameraInfo.D.at(2);
    this->rgbDistortionCoeffs.at<double>(0,3) = this->rgbCameraInfo.D.at(3);
    this->rgbDistortionCoeffs.at<double>(0,4) = this->rgbCameraInfo.D.at(4);
    
}