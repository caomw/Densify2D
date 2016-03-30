//
// Created by alex on 3/29/16.
//
#include "utils.h"
#include "SFMPoint.h"
#include "imageProcessing.h"

using namespace gtsam;
using namespace std;
using namespace cv;

void projectionTest(){
    const int rows = 300;
    const int cols = 400;
    const int searchWindow = 5;
    //vector<SimpleCamera> cameras;
    //vector<Mat> images;
    //getCamerasFromCSV(cameras,"/home/alex/Densify2D/cameras.csv");
    //getImages(images,644,667);
    Mat sparseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    //Mat denseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    Mat sparsePointImage(rows, cols, CV_32FC3, Scalar(0,0,0)); //use floats to store 3D points at each pixel
    vector<SFMPoint> points;
    getSFMPoints(points,"/home/alex/Densify2D/points.csv");
    makeSparseImage(sparseColorImage, sparsePointImage, points);
    display(sparseColorImage, "SPARSE IMAGE", 0);
}

int main(){
    projectionTest();
}