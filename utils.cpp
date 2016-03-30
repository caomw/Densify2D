//
// Created by Alex Hagiopol on 3/26/16.
//

#include "utils.h"
using namespace std;
using namespace gtsam;
using namespace cv;

istream &operator>>(istream& ins, record_t& record) {
    record.clear();
    string line;
    getline(ins, line);
    stringstream ss(line);
    string field;
    while (getline(ss, field, ',')) {
        stringstream fs(field);
        double f = 0.0;
        fs >> f;
        record.push_back(f);
    }
    return ins;
}

istream &operator>>(istream& ins, data_t& data) {
    data.clear();
    record_t record;
    while (ins >> record) {
        data.push_back(record);
    }
    return ins;
}

void getCamerasFromCSV(vector<SimpleCamera>& myCameras, const string& csvFile) {
    ifstream infile(csvFile);
    data_t data;
    infile >> data;
    infile.close();
    Cal3_S2::shared_ptr K(new Cal3_S2(1000, 1000, 0.1, 4000 / 2, 3000 / 2)); //made up
    for (int r = 0; r < data.size(); r++) { //start at row 1 because row 0 does not contain data
        Point3 location(data[r][0], data[r][1], data[r][2]);
        Rot3 orientation(data[r][6], data[r][7], data[r][8],
                         data[r][9], data[r][10], data[r][11],
                         data[r][12], data[r][13], data[r][14]);
        Pose3 pose(orientation, location);
        SimpleCamera camera(pose, *K);
        myCameras.push_back(camera);
    }
}

void getImages(vector<Mat>& images, int start, int end) {
    for (int i = start; i <= end; i++) {
        string strFileName = "/home/alex/Densify2D/images/dji_%04i.jpg";
        char chrFileName[80]; //Name of csv file we write
        sprintf(chrFileName, strFileName.c_str(), i); //format filename
        string finalStrFileName = chrFileName;
        cout << "reading " << finalStrFileName << endl;
        Mat image = imread(finalStrFileName, CV_LOAD_IMAGE_COLOR);
        images.push_back(image);
    }
}

void getSFMPoints(vector<SFMPoint> &points, const string &csvFile) {
    ifstream infile(csvFile);
    data_t data;
    infile >> data;
    infile.close();
    for (int r = 0; r < data.size(); r++) { //start at row 1 because row 0 does not contain data
        Point3 location(data[r][1], data[r][2], data[r][3]);
        Vec3b color(data[r][6], data[r][5], data[r][4]);
        SFMPoint point(location,color);
        points.push_back(point);
    }
}

void display(const Mat& image, const string& title, const int& waitTime){
    namedWindow(title, WINDOW_AUTOSIZE );
    imshow(title,image);
    waitKey(waitTime);
}

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
