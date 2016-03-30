//
// Created by alex on 3/29/16.
//
#include "utils.h"
void makeSparseImage(cv::Mat& colorImage, cv::Mat& pointImage, const std::vector<SFMPoint>& points);
cv::Point searchForNonZeroNeighbor(const cv::Mat& queryImage, const cv::Point& queryLocation, const int searchWindow);
std::tuple<cv::Point,cv::Point,cv::Point> searchForThreeNearestNeighbors(const cv::Mat& queryImage, const cv::Point& queryLocation, const int searchWindow);
cv::Vec3b computePixelColorViaProjection(const cv::Mat& pointImage, const cv::Point& queryLocation, const std::tuple<cv::Point,cv::Point,cv::Point>& neighbors, const std::vector<gtsam::SimpleCamera>& cameras, const std::vector<cv::Mat>& images);
