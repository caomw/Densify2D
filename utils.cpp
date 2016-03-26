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
    Cal3_S2::shared_ptr K(new Cal3_S2(500, 500, 0.1, 4000 / 2, 3000 / 2)); //made up
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
        string strFileName = "/Users/alexhagiopol/Densify2D/images/dji_%04i.jpg";

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
