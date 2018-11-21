#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <dr_msg/YoloPredict.h>

#include "detectlane.h"
#include "carcontrol.h"

#define SUB_TOPIC_TRAF "yoloPredict"


bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
int skipFrame = 1;


void trafCallback(const dr_msg::YoloPredict& yoloPredict)
{
    try
    {
        car->sign_signal.sign = yoloPredict.objClass;
        car->sign_signal.prob = yoloPredict.prob;
        car->sign_signal.name = yoloPredict.name;
        car->sign_signal.bbox.x = yoloPredict.bbox.x;
        car->sign_signal.bbox.y = yoloPredict.bbox.y;
        car->sign_signal.bbox.width = yoloPredict.bbox.width;
        car->sign_signal.bbox.height = yoloPredict.bbox.height;
        ROS_INFO("%s: %d: %f", car->sign_signal.name.c_str(), car->sign_signal.sign, car->sign_signal.prob);
    }
    catch (int e)
    {
        ROS_ERROR("Failed to read traffic sign: %d", e);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("View", cv_ptr->image);
	waitKey(1);
        detect->update(cv_ptr->image);
        car->driverCar(detect->getLeftLane(), detect->getRightLane(), 50);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void videoProcess()
{
    Mat src;
    while (true)
    {
        capture >> src;
        if (src.empty()) break;
        
        imshow("View", src);
        detect->update(src);
        waitKey(30);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    cv::namedWindow("Threshold");
    cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");

    detect = new DetectLane();
    car = new CarControl();

    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        ros::NodeHandle trafficSign;
        image_transport::ImageTransport it(nh);
        ros::Subscriber traf_sub = trafficSign.subscribe(SUB_TOPIC_TRAF, 1, trafCallback);
        image_transport::Subscriber sub = it.subscribe("Team1_image", 1, imageCallback);
        

        ros::spin();
    } else {
        videoProcess();
    }
    cv::destroyAllWindows();
}
