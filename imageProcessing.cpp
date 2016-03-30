//
// Created by alex on 3/29/16.
//

#include "imageProcessing.h"

using namespace gtsam;
using namespace cv;
using namespace std;

//point image
void makeSparseImage(Mat& colorImage, Mat& pointImage, const std::vector<SFMPoint>& points){
    int rows = colorImage.rows;
    int cols = colorImage.cols;
    double xMin;
    double xMax;
    double yMin;
    double yMax;
    double zMin;
    double zMax;
    findXYZMinMax(points, xMin, xMax, yMin, yMax, zMin, zMax);
    for (int pointIndex = 0; pointIndex < points.size(); pointIndex ++){
        int xPixelCoord = round(convertScale(points[pointIndex].location().x(),xMin,xMax,0,cols-1));
        int yPixelCoord = round(convertScale(points[pointIndex].location().y(),yMin,yMax,0,rows-1));
        Vec3b color = points[pointIndex].color();
        colorImage.at<Vec3b>(Point(xPixelCoord,yPixelCoord)) = color;
        Vec3d point(points[pointIndex].location().x(),points[pointIndex].location().y(),points[pointIndex].location().z());
        //cout << "Storing point " << point << " at " << "(" << xPixelCoord << "," << yPixelCoord << ")" << endl;
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
    //cout << "QUERY LOCATION = " << queryLocation << endl;
    for (int n = 0; n < candidateNeighbors.size(); n++){
        Point candidate = candidateNeighbors[n];

        double distance = sqrt(pow(candidate.x - queryLocation.x,2) + pow(candidate.y - queryLocation.y,2));
        //cout << "candidate #" << n << " = " << candidate.x << ","<< candidate.y << " distance = " << distance << endl;
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
    //cout << " neighbor 1 = " << get<0>(neighbors) << " neighbor 2 = " << get<1>(neighbors) << " neighbor 3 = " << get<2>(neighbors) << endl;
    return neighbors;
}

Vec3b computePixelColorViaProjection(const Mat& pointImage, const Point& queryLocation, const tuple<Point,Point,Point>& neighbors, const std::vector<SimpleCamera>& cameras, const std::vector<Mat>& images){
    //convert OpenCV 3D point representation to GTSAM 3D point representation
    Vec3d cvNeighbor1 = pointImage.at<Vec3d>(get<0>(neighbors));
    Vec3d cvNeighbor2 = pointImage.at<Vec3d>(get<1>(neighbors));
    Vec3d cvNeighbor3 = pointImage.at<Vec3d>(get<2>(neighbors));
    Point3 gtsamNeighbor1(cvNeighbor1[0],cvNeighbor1[1],cvNeighbor1[2]);
    Point3 gtsamNeighbor2(cvNeighbor2[0],cvNeighbor2[1],cvNeighbor2[2]);
    Point3 gtsamNeighbor3(cvNeighbor3[0],cvNeighbor3[1],cvNeighbor3[2]);
    //estimate world coordinates of query point
    Point3 query3DPoint = (gtsamNeighbor1 + gtsamNeighbor2 + gtsamNeighbor3)/3; //compute average location in world
    assert(cameras.size() == images.size() && "Must have the same number of images and cameras.");
    std::vector<Vec3b> colors;
    for (int c = 0; c < cameras.size(); c++){
        Point2 projected = cameras[c].project(query3DPoint);
        if ((projected.x() >= 0) && (projected.y() >= 0) && (projected.x() < images[c].rows) && (projected.y() < images[c].cols)){ //check that camera can see point
            query3DPoint.print("*----------------------------------------------*\n3D POINT\n");
            cameras[c].print("CAMERA\n");
            projected.print("2D POINT\n");

            colors.push_back(images[c].at<Vec3b>(Point(projected.y(),projected.x())));
        } else {
            query3DPoint.print("*----------------------------------------------*\nBAD 3D POINT\n");
            cameras[c].print("BAD CAMERA\n");
            projected.print("BAD 2D POINT\n");
        }
    }
    Vec3b color(0,0,0);
    for (int c = 0; c < colors.size(); c++){
        color = color + colors[c] / (int) colors.size();
    }
    return color;
}