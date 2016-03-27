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

void display(const Mat& image, const string& title, const int& waitTime){
    namedWindow(title, WINDOW_AUTOSIZE );
    imshow(title,image);
    waitKey(waitTime);
}

void makeSparseImage(Mat& colorImage, Mat& zImage, const string& pointsFileName){
    int rows = colorImage.rows;
    int cols = colorImage.cols;
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
        colorImage.at<Vec3b>(Point(xPixelCoord,yPixelCoord)) = color;
        Vec3b height(points[pointIndex].location().z()*-1,points[pointIndex].location().z()*-1,points[pointIndex].location().z()*-1);
        zImage.at<Vec3b>(Point(xPixelCoord,yPixelCoord)) = height;
    }
}

Point searchForNonZeroNeighbor(const Mat& queryImage, const Point& queryLocation, const int searchWindow){
    Point neighbor = Point(-1,-1); //return a negative location if we don't find a good candidate.
    int startXIndex = queryLocation.x - searchWindow;
    int startYIndex = queryLocation.y - searchWindow;
    int endXIndex = queryLocation.x + searchWindow;
    int endYIndex = queryLocation.y + searchWindow;
    if ((startXIndex < 0) || (endXIndex > queryImage.cols) || (startYIndex < 0) || (endYIndex > queryImage.rows)){
        return neighbor; //return a negative location if we don't find a good candidate.
    }
    Vec3b zero = Vec3b(0,0,0);
    std::vector<Point> candidateNeighbors;
    for (int c = startXIndex; c < endXIndex; c++){
        for (int r = startYIndex; r < endYIndex; r++){
            Vec3b channelValue = queryImage.at<Vec3b>(Point(c,r));
            if (channelValue != zero){
                candidateNeighbors.push_back(Point(c,r));
            }
        }
    }
    if (candidateNeighbors.size() == 0) return neighbor; //return a negative location if we don't find a good candidate.
    double minDistance = 9999;
    for (int n = 0; n < candidateNeighbors.size(); n++){
        Point candidate = candidateNeighbors[n];
        double distance = sqrt((candidate.x - queryLocation.x)^2 + (candidate.y - queryLocation.y)^2);
        if (distance <= minDistance){
            minDistance = distance;
            neighbor = candidate;
        }
    }
    cout << "query location = " << queryLocation << " neighbor location  = " << neighbor << endl;
    return neighbor;
}

int main() {
    const int rows = 300;
    const int cols = 400;
    const int searchWindow = 5;
    vector<SimpleCamera> cameras;
    vector<Mat> images;
    Mat sparseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    Mat denseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    Mat sparseZImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    makeSparseImage(sparseColorImage, sparseZImage, "/Users/alexhagiopol/Densify2D/points.csv");
    display(sparseColorImage, "SPARSE IMAGE", 1000);
    Vec3b zero = Vec3b(0,0,0);
    for (int r = 0; r < rows; r++){
        std::cout << "Computing row #" << r << std::endl;
        display(denseColorImage,"DENSE IMAGE",500);
        for (int c = 0; c < cols; c++){
            Vec3b channelValue = sparseZImage.at<Vec3b>(Point(c,r));
            if (channelValue == zero){
                Point bestNeighbor = searchForNonZeroNeighbor(sparseZImage,Point(c,r),searchWindow);
                if (bestNeighbor != Point(-1,-1)){
                    denseColorImage.at<Vec3b>(Point(c,r)) = sparseColorImage.at<Vec3b>(bestNeighbor);
                }
            } else{
                denseColorImage.at<Vec3b>(Point(c,r)) = sparseColorImage.at<Vec3b>(Point(c,r));
            }
        }
    }
    display(denseColorImage,"DENSE IMAGE",0);


}
