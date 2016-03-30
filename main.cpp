//
// Created by alex on 2/24/16.
//
#include "utils.h"
#include "SFMPoint.h"
#include "imageProcessing.h"

using namespace gtsam;
using namespace std;
using namespace cv;

int main() {
    const int rows = 300;
    const int cols = 400;
    const int searchWindow = 5;
    vector<SimpleCamera> cameras;
    vector<Mat> images;
    getCamerasFromCSV(cameras,"/home/alex/Densify2D/cameras.csv");
    getImages(images,644,667);
    Mat sparseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    Mat denseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    Mat sparsePointImage(rows, cols, CV_32FC3, Scalar(0,0,0)); //use floats to store 3D points at each pixel
    vector<SFMPoint> points;
    getSFMPoints(points,"/home/alex/Densify2D/points.csv");
    makeSparseImage(sparseColorImage, sparsePointImage, points);
    display(sparseColorImage, "SPARSE IMAGE", 0);
    /*
    Vec3b zero = Vec3b(0,0,0);
    for (int r = 0; r < rows; r++){
        std::cout << "Computing row #" << r << std::endl;
        display(denseColorImage,"DENSE IMAGE",500);
        for (int c = 0; c < cols; c++){
            Vec3b channelValue = sparseColorImage.at<Vec3b>(Point(c,r));
            if (channelValue == zero){ //For ever blank pixel, compute a best color to fill in.
                //Point bestNeighbor = searchForNonZeroNeighbor(sparseZImage,Point(c,r),searchWindow);
                tuple<Point,Point,Point> bestNeighbors = searchForThreeNearestNeighbors(sparseColorImage,Point(c,r),searchWindow);

                Point bestNeighbor1 = get<0>(bestNeighbors);
                Point bestNeighbor2 = get<1>(bestNeighbors);
                Point bestNeighbor3 = get<2>(bestNeighbors);
                if (get<0>(bestNeighbors) != Point(-1,-1)){ //ensure that good neighbors are found; if not leave pixel unchanged
                    //denseColorImage.at<Vec3b>(Point(c,r)) = (sparseColorImage.at<Vec3b>(bestNeighbor1)/3 + sparseColorImage.at<Vec3b>(bestNeighbor2)/3 + sparseColorImage.at<Vec3b>(bestNeighbor3)/3); //use average of 3 best neighbors
                    Vec3b assignedColor = computePixelColorViaProjection(sparsePointImage,Point(c,r),bestNeighbors,cameras, images);
                    denseColorImage.at<Vec3b>(Point(c,r)) = assignedColor;
                }
            } else{ //if the pixel is not blank, copy over its color into dense image
                //cout << (double) channelValue.operator[](0) << " " << (double) channelValue.operator[](1) << " " <<  (double) channelValue.operator[](2) << endl;
                denseColorImage.at<Vec3b>(Point(c,r)) = sparseColorImage.at<Vec3b>(Point(c,r)); //copy the color from the sparse image if we have it.
            }
        }
    }
    display(denseColorImage,"DENSE IMAGE",0);
     */
}
