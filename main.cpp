//
// Created by alex on 2/24/16.
//
#include "utils.h"
#include "SFMPoint.h"

using namespace gtsam;
using namespace cv;
using namespace std;

void findXYZMinMax(const vector<SFMPoint>& points, double& xMin, double& xMax, double& yMin, double& yMax, double& zMin, double& zMax){
    xMin = 9999;
    yMin = 9999;
    zMin = 9999;
    xMax = -9999;
    yMax = -9999;
    zMax = -9999;
    for (int i = 0; i < points.size(); i++){
        double x = points[i].location().x();
        double y = points[i].location().y();
        double z = points[i].location().z();
        if (x > xMax) xMax = x;
        if (x < xMin) xMin = x;
        if (y > yMax) yMax = y;
        if (y < yMin) yMin = y;
        if (z > zMax) zMax = z;
        if (z < zMin) zMin = z;
    }
}

//convert a number "in" from the (inMin, inMax) scale to the (outMin, outMax) scale
double convertScale(double in, double inMin, double inMax, double outMin, double outMax){
    return (in + abs(inMin))*(outMax - outMin)/(inMax - inMin) - abs(outMin);
}

void display(const Mat& image, const string& title){
    namedWindow(title, WINDOW_AUTOSIZE );
    imshow(title,image );
    waitKey(0);
}

void makeSparseImage(Mat& image, const string& pointsFileName){
    int rows = image.rows;
    int cols = image.cols;
    vector<SFMPoint> points;
    getSFMPoints(points,pointsFileName);
    double xMin;
    double xMax;
    double yMin;
    double yMax;
    double zMin;
    double zMax;
    findXYZMinMax(points, xMin, xMax, yMin, yMax, zMin, zMax);
    //Mat sparseImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    int pointsSize = points.size();
    for (int pointIndex = 0; pointIndex < points.size(); pointIndex ++){
        int xPixelCoord = round(convertScale(points[pointIndex].location().x(),xMin,xMax,0,cols));
        int yPixelCoord = round(convertScale(points[pointIndex].location().y(),yMin,yMax,0,rows));
        Vec3b color = points[pointIndex].color();
        image.at<Vec3b>(Point(xPixelCoord,yPixelCoord)) = color;
    }
}

int main() {
    int rows = 600;
    int cols = 800;
    int rSearchRange = rows / 20;
    int cSearchRange = cols / 20;
    vector<SimpleCamera> cameras;
    vector<Mat> images;
    Mat sparseImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    makeSparseImage(sparseImage, "/Users/alexhagiopol/Densify2D/points.csv");
    display(sparseImage, "SPARSE IMAGE");
}
