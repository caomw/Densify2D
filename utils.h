//
// Created by Alex Hagiopol on 3/26/16.
//

#pragma once
#include "SFMPoint.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <string>


typedef std::vector<double> record_t;
typedef std::vector<record_t> data_t;
void getCamerasFromCSV(std::vector<gtsam::SimpleCamera>& myCameras, const std::string& csvFile);
void getImages(std::vector<cv::Mat>& images, int start, int end);
void getSFMPoints(std::vector<SFMPoint> &points, const std::string &csvFile);
void display(const cv::Mat& image, const cv::string& title, const int& waitTime);
void findXYZMinMax(const std::vector<SFMPoint>& points, double& xMin, double& xMax, double& yMin, double& yMax, double& zMin, double& zMax);
double convertScale(double in, double inMin, double inMax, double outMin, double outMax);





