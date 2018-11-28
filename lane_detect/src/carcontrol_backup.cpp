#include "carcontrol.h"

CarControl::CarControl()
{
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("Team1_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("Team1_speed",10);
}

CarControl::~CarControl() {}

float CarControl::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

Point CarControl::getOffsetPoint(const vector<Point> &lane, const int pointNum, const int length, const int laneName)
{
    vector<Point> subLane(lane.begin() + pointNum -2, lane.begin() + pointNum +2);
    Vec4f line;
    fitLine(subLane, line, 2, 0, 0.01, 0.01);
    Point dst;
    // Left lane
    if(laneName == 0)
    {
        dst.x = line[1]*length/(2*sqrt(line[0]*line[0] + line[1]*line[1])) + lane[pointNum].x;
    }
    else if (laneName == 1)
    {
        dst.x = -line[1]*length/(2*sqrt(line[0]*line[0] + line[1]*line[1])) + lane[pointNum].x;
    }
    dst.y = int(-(dst.x - lane[pointNum].x)*line[0]/line[1] + lane[pointNum].y);
    dst.x = int(dst.x);
    ROS_INFO("offset Point : %d %d", dst.x,dst.y);
    return dst;
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    speed.data = velocity;

    int i = left.size() - 11;
    float error = preError;

    // int null_countLeft =0;
    // int null_countRight =0;

    // for(int j=0; j < left.size();j++)
    // {
    //     if(left[i] == DetectLane::null)
    //         null_countLeft ++;
    //     if(right[i] == DetectLane::null)
    //         null_countRight ++;
    // }
    if(sign_signal.sign == 0)
    {
        laneStatus = 0;
    }
    if(sign_signal.sign == 1)
    {
        laneStatus = 1;
    }
    // check vail signlaneStatus
    if(sign_signal.bbox.x > (imageWidth - offsetX) && sign_signal.bbox.y < (imageHeight - offsetY))
    {
        statusDelay = -1;
        delayCount = 50;
    }

    if(statusDelay == -1)
    {
        if(delayCount < 0)
        {
            laneStatus = -1;
            speed.data = 60;
            statusDelay = 0;
        }
        else 
            delayCount --;

    }

    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        if (i < 0) return;
    }
    if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)
    {
        if(laneStatus == 0)
        {
            ROS_INFO("sign signal: %d",sign_signal.sign);
            // Point offSetPoint = getOffsetPoint(left, i, laneWidth/2, 0);
            // error = errorAngle(offSetPoint);
            error = errorAngle(left[i] + Point(laneWidth / 2, 0));
            speed.data = 30;
        }
        else if(laneStatus == 1)
        {
            ROS_INFO("sign signal: %d",sign_signal.sign);
            // Point offSetPoint = getOffsetPoint(right, i, laneWidth/2, 1);
            // error = errorAngle(offSetPoint);
            error = errorAngle(right[i] - Point(laneWidth / 2, 0));
            speed.data = 30; 
        }
        else
        {
            // Point offSetPoint = getOffsetPoint(right, i, laneWidth/2, 1);
            // error = errorAngle(offSetPoint);
            ROS_INFO("Tracking center");
            // error = errorAngle((left[i] + right[i])/2);
            error = errorAngle(right[i] - Point(laneWidth / 2, 0));
        }         
    } 
    else if (left[i] != DetectLane::null)
    {
        // Point offSetPoint = getOffsetPoint(left, i, laneWidth/2, 0);
        // error = errorAngle(offSetPoint);
        error = errorAngle(left[i] + Point(laneWidth / 2, 0));
    }
    else
    {
        // Point offSetPoint = getOffsetPoint(right, i, laneWidth/2, 1);
        // error = errorAngle(offSetPoint);
        error = errorAngle(right[i] - Point(laneWidth / 2, 0));
    }

    // if(sign_signal.bbox.x > (imageWidth - offsetX) && sign_signal.bbox.y < (imageHeight - offsetY)  && sign_signal.sign == 0)
    // {
    //     turn = sign_signal.sign;
    //     turn_count = turn_times;
    //     ROS_INFO("sign position: %f  %f", sign_signal.bbox.x, sign_signal.bbox.y );
    // }
    // else if(sign_signal.bbox.x > (imageWidth - offsetX) && sign_signal.bbox.y < (imageHeight - offsetY)  && sign_signal.sign == 1)
    // {speed
    //     turn = sign_signal.sign;
    //     turn_count = turn_times;
    //     ROS_INFO("sign position: %f  %f", sign_signal.bbox.x, sign_signal.bbox.y );
    // }       

    // if(turn_count > 0)
    //     if(turn == 0)
    //     {
    //         error = -5;
    //         turn_count --;
    //     }
    //     else if (turn == 1)
    //     {
    //         error += turnRight_offset;
    //         turn_count --;
    //     }
    // else turn = -1;

    // if(sign_signal.bbox.x > (imageWidth - offsetX) && sign_signal.bbox.y < (imageHeight - offsetY) && sign_signal.sign == 0)
    // {
    //     speedStatus = -1; 
    // }

    ROS_INFO("angle error: %f  ", error);
    angle.data = error;

    speed.data -= error*error*1500/90/90;
    if(speed.data <= 30) speed.data = 20;
    ROS_INFO("speed: %f  ", speed.data);
.

        if(laneStatus == 0)
        {
            speed.data = 40;
        }
        else if(laneStatus == 1)
        {
            speed.data = 40; 
        }       
    

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 

