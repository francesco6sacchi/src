#pragma once

#ifndef __H_INCLUDE_CAMERA__
#define __H_INCLUDE_CAMERA__


#include "general.h"

class CameraManager{

private:
    // ROS NodeHandle
    ros::NodeHandle nh;

    // ROS data
    sensor_msgs::Image rgbImageRaw;
    sensor_msgs::CompressedImage rgbImageComp;
    sensor_msgs::CameraInfo rgbCameraInfo;

    // ROS subscribers
    ros::Subscriber subRgbImageRaw;
    ros::Subscriber subRgbImageComp;
    ros::Subscriber subRgbCameraInfo;

    // Subscriber callbacks
    void RgbImageRawCallback(const sensor_msgs::ImageConstPtr& msg)            { this->rgbImageRaw = *msg; };
    void RgbImageCompCallback(const sensor_msgs::CompressedImageConstPtr& msg) { this->rgbImageComp = *msg;
                                                                                 this->RosToCv(&this->rgbImageComp, this->cvRgbImage); };
                                                                                // this->RosToCv(&this->rgbImageComp, &this->cvRgbImage); };
    void RgbCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)     { this->rgbCameraInfo = *msg; 
                                                                                 this->ExtractCameraCoefficients(); }

    // Converted variables
    cv::Mat* cvRgbImage;
    // cv::Mat cvRgbImage;
    cv::Mat rgbCameraMatrix;
    cv::Mat rgbDistortionCoeffs;


    // Other Functions
    void ExtractCameraCoefficients();

public:
    // Constructor
    CameraManager(ros::NodeHandle input);

    // Getters
    sensor_msgs::Image* GetRgbImageRaw()            { return &(this->rgbImageRaw); };
    sensor_msgs::CompressedImage* GetRgbImageComp() { return &(this->rgbImageComp); };
    sensor_msgs::CameraInfo* GetRgbCameraInfo()     { return &(this->rgbCameraInfo); };
    cv::Mat* GetCvRgbImage()                        { return this->cvRgbImage; };
    // cv::Mat GetCvRgbImage()                         { return (this->cvRgbImage); };
    cv::Mat* GetRgbCameraMatrix()       { return &(this->rgbCameraMatrix); };
    cv::Mat* GetRgbDistortionCoeffs()   { return &(this->rgbDistortionCoeffs); };

    // Methods
    void RosToCv(sensor_msgs::CompressedImage* inputCompImage, cv::Mat* outputCvImage);
    void RosToCv(sensor_msgs::Image* inputRawImage, cv::Mat* outputCvImage);

};

#endif