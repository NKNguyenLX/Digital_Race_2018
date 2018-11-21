#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <Yolo.h>
#include <dr_msg/YoloPredict.h>

#define OPENCV_WINDOW	"TS_Detectors"
//#define SUB_TOPIC		"/camera/image_raw"
#define SUB_TOPIC		"Team1_image"
#define PUB_TOPIC 		"yoloPredict"

using namespace cv;

class TS_Detector
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher yoloPredict_publisher;
	Yolo* yolo;

public:
	TS_Detector()
    : it_(nh_){
		image_sub_ = it_.subscribe(SUB_TOPIC, 1,&TS_Detector::imReceiveCb, this);
		yoloPredict_publisher = nh_.advertise<dr_msg::YoloPredict>(PUB_TOPIC, 5);
		yolo = new Yolo();
		yolo->loadNetwork();
		namedWindow(OPENCV_WINDOW);
	}

	~TS_Detector(){
		destroyWindow(OPENCV_WINDOW);
	}

	void imReceiveCb(const sensor_msgs::ImageConstPtr& msg){

		/* Convert to opencv image */
    	cv_bridge::CvImagePtr cv_ptr;
    	try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		Mat input_img = cv_ptr->image;

		/* Make predtiction with yolo */
		vector<YoloObject>  yoloObjects;

		yoloObjects = yolo->predict(input_img);
		
		/* Publish result */
		dr_msg::YoloPredict yoloPredict;
		if(yoloObjects.size() > 0){
			yoloPredict.bbox.x = yoloObjects[0].bbox.x;
			yoloPredict.bbox.y = yoloObjects[0].bbox.y;
			yoloPredict.bbox.width = yoloObjects[0].bbox.width;
			yoloPredict.bbox.height = yoloObjects[0].bbox.height;
			yoloPredict.objClass = yoloObjects[0].objClass;
			yoloPredict.prob = yoloObjects[0].prob;
			yoloPredict.name = yoloObjects[0].name;
			yoloPredict_publisher.publish(yoloPredict);
			rectangle(input_img, yoloObjects[0].bbox, cv::Scalar(100, 255, 0));
		}
		else{
			yoloPredict.objClass = -1;
			yoloPredict_publisher.publish(yoloPredict);
		}	

		/* Show the result */
		imshow(OPENCV_WINDOW, input_img);
		waitKey(1);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_ts_detector");
  TS_Detector* ts_dt = new TS_Detector();
  ros::spin();
  return 0;
}