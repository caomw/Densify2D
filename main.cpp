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

//point image
void makeSparseImage(Mat& colorImage, Mat& pointImage, const string& pointsFileName){
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
        Vec3d point(points[pointIndex].location().x(),points[pointIndex].location().y(),points[pointIndex].location().z());
        pointImage.at<Vec3d>(Point(xPixelCoord,yPixelCoord)) = point;
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
    //cout << "query location = " << queryLocation << " neighbor location  = " << neighbor << endl;
    return neighbor;
}

tuple<Point,Point,Point> searchForThreeNearestNeighbors(const Mat& queryImage, const Point& queryLocation, const int searchWindow){
    tuple<Point,Point,Point> neighbors;
    Point firstNeighbor = Point(-1,-1);
    Point secondNeighbor = Point(-1,-1);
    Point thirdNeighbor = Point(-1,-1);
    int startXIndex = queryLocation.x - searchWindow;
    int startYIndex = queryLocation.y - searchWindow;
    int endXIndex = queryLocation.x + searchWindow;
    int endYIndex = queryLocation.y + searchWindow;
    get<0>(neighbors) = firstNeighbor;
    get<1>(neighbors) = secondNeighbor;
    get<2>(neighbors) = thirdNeighbor;
    if ((startXIndex < 0) || (endXIndex > queryImage.cols) || (startYIndex < 0) || (endYIndex > queryImage.rows)){
        return neighbors; //return a negative location if we don't find a good candidate.
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
    if (candidateNeighbors.size() < 3) return neighbors; //return a negative location if we don't find 3 good candidates.
    double minDistance1 = 9999;
    double minDistance2 = 9999;
    double minDistance3 = 9999;
    cout << "QUERY LOCATION = " << queryLocation << endl;
    for (int n = 0; n < candidateNeighbors.size(); n++){
        Point candidate = candidateNeighbors[n];

        double distance = sqrt(pow(candidate.x - queryLocation.x,2) + pow(candidate.y - queryLocation.y,2));
        cout << "candidate #" << n << " = " << candidate.x << ","<< candidate.y << " distance = " << distance << endl;
        if (distance < minDistance1){ //closest point
            minDistance3 = minDistance2;
            minDistance2 = minDistance1;
            minDistance1 = distance;
            get<2>(neighbors) = get<1>(neighbors);
            get<1>(neighbors) = get<0>(neighbors);
            get<0>(neighbors) = candidate;
        } else if ((distance >= minDistance1) && (distance < minDistance2)){ //second closest point
            minDistance3 = minDistance2;
            minDistance2 = distance;
            get<2>(neighbors) = get<1>(neighbors);
            get<1>(neighbors) = candidate;
        } else if ((distance >= minDistance1) && (distance >= minDistance2) && (distance < minDistance3)){ //third closest point
            minDistance3 = distance;
            get<2>(neighbors) = candidate;
        }
    }
    cout << " neighbor 1 = " << get<0>(neighbors) << " neighbor 2 = " << get<1>(neighbors) << " neighbor 3 = " << get<2>(neighbors) << endl;
    return neighbors;
}

Vec3b computePixelColorViaProjection(const Mat& pointImage, const Point& queryLocation, const tuple<Point,Point,Point>& neighbors, const std::vector<SimpleCamera>& cameras){
    //convert OpenCV 3D point representation to GTSAM 3D point representation
    Vec3d cvNeighbor1 = pointImage.at<Vec3d>(get<0>(neighbors));
    Vec3d cvNeighbor2 = pointImage.at<Vec3d>(get<1>(neighbors));
    Vec3d cvNeighbor3 = pointImage.at<Vec3d>(get<2>(neighbors));
    Point3 gtsamNeighbor1(cvNeighbor1[0],cvNeighbor1[1],cvNeighbor1[2]);
    Point3 gtsamNeighbor2(cvNeighbor2[0],cvNeighbor2[1],cvNeighbor2[2]);
    Point3 gtsamNeighbor3(cvNeighbor3[0],cvNeighbor3[1],cvNeighbor3[2]);
    //estimate world coordinates of query point
    Point3 query3DPoint = (gtsamNeighbor1 + gtsamNeighbor2 + gtsamNeighbor3)/3; //compute average location in world

    /*
    //cout << "Query Location = " << queryLocation << endl;
    cout << "Neighbor 1" << get<0>(neighbors) << endl;
    cout << "Neighbor 2" << get<1>(neighbors) << endl;
    cout << "Neighbor 3" << get<2>(neighbors) << endl;
    gtsamNeighbor1.print("GTSAM Neighbor 1 \n");
    gtsamNeighbor2.print("GTSAM Neighbor 2 \n");
    gtsamNeighbor3.print("GTSAM Neighbor 3 \n");
    query3DPoint.print("QUERY 3D POINT \n");
    */

    Vec3b color(0,0,0);
    return color;
}

int main() {
    const int rows = 300;
    const int cols = 400;
    const int searchWindow = 5;
    vector<SimpleCamera> cameras;
    vector<Mat> images;
    getCamerasFromCSV(cameras,"Users/alexhagiopol/Densify2D/cameras.csv");
    Mat sparseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    Mat denseColorImage(rows, cols, CV_8UC3, Scalar(0,0,0));
    Mat sparsePointImage(rows, cols, CV_32FC3, Scalar(0,0,0)); //use floats to store 3D points at each pixel
    makeSparseImage(sparseColorImage, sparsePointImage, "/Users/alexhagiopol/Densify2D/points.csv");
    display(sparseColorImage, "SPARSE IMAGE", 1000);
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
                    denseColorImage.at<Vec3b>(Point(c,r)) = (sparseColorImage.at<Vec3b>(bestNeighbor1)/3 + sparseColorImage.at<Vec3b>(bestNeighbor2)/3 + sparseColorImage.at<Vec3b>(bestNeighbor3)/3); //use average of 3 best neighbors
                    //Vec3b assignedColor = computePixelColorViaProjection(sparsePointImage,Point(c,r),bestNeighbors,cameras);
                }
            } else{ //if the pixel is not blank, copy over its color into dense image
                //cout << (double) channelValue.operator[](0) << " " << (double) channelValue.operator[](1) << " " <<  (double) channelValue.operator[](2) << endl;
                denseColorImage.at<Vec3b>(Point(c,r)) = sparseColorImage.at<Vec3b>(Point(c,r)); //copy the color from the sparse image if we have it.
            }
        }
    }
    display(denseColorImage,"DENSE IMAGE",0);
}
