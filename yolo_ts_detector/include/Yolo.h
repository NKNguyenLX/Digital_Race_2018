#ifndef __YOLO__H__
#define __YOLO__H__

#include <string.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <darknet.h>
#include <image_opencv.h>

using namespace std;
using namespace cv;

struct YoloObject{
    Rect2d bbox;
    int objClass;
    float prob;
    string name;
};

struct YoloParam{

};

class Yolo{

public:
    Yolo();
    ~Yolo();

    void loadNetwork();
    vector<YoloObject> predict(Mat X);

protected:

private:
    char **names;
    image **alphabet;
    network *net;
    layer output_layer;
    YoloParam yoloParam;
};

#endif