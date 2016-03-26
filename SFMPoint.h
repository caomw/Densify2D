//
// Created by Alex Hagiopol on 3/26/16.
//

#pragma once
#include <gtsam/geometry/Point3.h>
#include "opencv2/core/core.hpp"
struct SFMPoint{
    gtsam::Point3 location_;
    cv::Vec3b color_;
    SFMPoint(gtsam::Point3& location, cv::Vec3b& color):location_(location),color_(color){}
    gtsam::Point3 location() const {return location_;}
    cv::Vec3b color() const {return color_;}
};
