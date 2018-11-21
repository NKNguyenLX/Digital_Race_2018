#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

#include "detectlane.h"

using namespace std;
using namespace cv;

typedef struct {
    int sign;
    float prob;
    Rect2d bbox;
    string name;
}SIGN_SIGNAL;

class CarControl 
{
public:
    CarControl();
    ~CarControl();
    void driverCar(const vector<Point> &left, const vector<Point> &right, float velocity);
    SIGN_SIGNAL sign_signal;

private:
    float errorAngle(const Point &dst);
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    ros::NodeHandle node_trafficSign;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;
    ros::Subscriber traf_sub;

    Point carPos;
    
    float laneWidth = 40;

    float minVelocity = 10;
    float maxVelocity = 50;

    float preError;

    float kP;
    float kI;
    float kD;

    int t_kP;
    int t_kI;
    int t_kD;
};

#endif
