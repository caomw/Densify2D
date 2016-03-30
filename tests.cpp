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
    const int rows = 600;
    const int cols = 800;
    const int searchWindow = 5;
    Mat sparseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0)); //visualize 3D reconstruction in 2D
    Mat sparsePointImage(rows, cols, CV_32FC3, Scalar(0,0,0)); //use floats to store 3D points corresponding to each 2D point in visualization
    vector<SFMPoint> points;
    getSFMPoints(points,"/home/alex/Densify2D/points.csv");
    makeSparseImage(sparseColorImage, sparsePointImage, points);

    //Hand-selected camera and input image for test
    vector<SimpleCamera> cameras;
    vector<Mat> images;
    getCamerasFromCSV(cameras,"/home/alex/Densify2D/cameras.csv");
    getImages(images,659,659); //657
    Mat image = images[0];
    SimpleCamera camera = cameras[15]; //13 //camera corresponding to image chosen
    camera.print("CAMERA INFO: \n");

    //Point hand selected from dataset: Storing point [-3.02872, 38.0591, -80.0756] at (X:375,Y:379)
    Point rcImageLocation = Point(379,375); //using R,C convention here.
    Point xyImageLocation = Point(375,379); //using X,Y convention here. OpenCV requires different conventions in different places!
    circle(sparseColorImage,rcImageLocation,6,Scalar(0,0,255),2);
    //Draw circle on sparse image to show point
    display(sparseColorImage, "POINT IN SPARSE IMAGE", 0);

    //Verify values of world point in different formats
    Vec3d cvWorldPoint = sparsePointImage.at<Vec3d>(xyImageLocation);
    cout << "World Point: " << cvWorldPoint << endl;
    Point3 gtsamWorldPoint(cvWorldPoint[0],cvWorldPoint[1],cvWorldPoint[2]); //convert to gtsam
    gtsamWorldPoint.print("GTSAM World Point: ");

    //Perform projection
    Point2 gtsamProjectedImageLocation = camera.project(gtsamWorldPoint);
    gtsamProjectedImageLocation.print("Projected GTSAM Image Location: ");
    Point rcProjectedImageLocation(gtsamProjectedImageLocation.x(),gtsamProjectedImageLocation.y()); //correct conversion from gtsam to opencv
    circle(image,rcProjectedImageLocation,30,Scalar(0,0,255),6); //draw a circle where that point ought to be
    display(image, "POINT IN INPUT IMAGE", 0);
}


int main(){
    projectionTest();
}