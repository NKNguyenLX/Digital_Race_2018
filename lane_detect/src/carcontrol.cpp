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

float CarControl::checksTraightLine(const vector<Point> &lane)
{
    Vec4f line;
    float error = 0;
    fitLine(lane, line, 2, 0,0.01,0.01);
    for(int i=0;i < lane.size();i++)
    {
        if(lane[i] != DetectLane::null)
        {
            error += sqrt((line[1]*(lane[i].x - line[2]) - line[0]*(lane[i].y - line[3]))*
                (line[1]*(lane[i].x - line[2]) - line[0]*(lane[i].y - line[3])));
        }
    }
    return error;
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;
    vector<Point> bestLane;

    // Set initial speed
    // speed.data = velocity;
    speed.data = INIT_SPEED;

    int i = left.size() - 11;
    float error = preError;
    // Check longer lane
    int null_countLeft =0;
    int null_countRight =0;

    for(int j=0; j < left.size();j++)
    {
        if(left[i] == DetectLane::null)
            null_countLeft ++;
        if(right[i] == DetectLane::null)
            null_countRight ++;
    }
    if(null_countRight < null_countLeft)
    {
        bestLane = right;
    }
    else
    {
        bestLane = left;
    } 

    // Lane state
    if(sign_signal.sign == LEFT_SIGN)
    {
        laneStatus = LEFT_SIGN;
    }
    if(sign_signal.sign == RIGHT_SIGN)
    {
        laneStatus = RIGHT_SIGN;
    }
    // check vaild signlaneStatus
    if(sign_signal.bbox.x > (imageWidth - offsetX) && sign_signal.bbox.y < (imageHeight - offsetY))
    {
        statusDelay = DELAY;
        delayCount = DELAY_COUNT;
    }

    if(statusDelay == DELAY)
    {
        if(delayCount < 0)
        {
            laneStatus = NO_SIGN;
            speed.data = INIT_SPEED;
            statusDelay = NO_DELAY;
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
        if(laneStatus == LEFT_SIGN)
        {
            ROS_INFO("sign signal: %d",sign_signal.sign);
            error = errorAngle(left[i] + Point(laneWidth / 2, 0));
            speed.data = TURN_LEFT_SPEED;
        }
        else if(laneStatus == RIGHT_SIGN)
        {
            ROS_INFO("sign signal: %d",sign_signal.sign);
            error = errorAngle(right[i] - Point(laneWidth / 2, 0));
            speed.data = TURN_RIGHT_SPEED; 
        }
        else
        {
            ROS_INFO("Tracking center");
            error = errorAngle(right[i] - Point(laneWidth / 2, 0));
        }         
    } 
    else if (left[i] != DetectLane::null)
    {
        error = errorAngle(left[i] + Point(laneWidth / 2, 0));
    }
    else
    {
        error = errorAngle(right[i] - Point(laneWidth / 2, 0));
    }

    // Calculate error from straight line
    float straightLineError = checksTraightLine(bestLane);
    ROS_INFO("Line error: %f",straightLineError );

    // Get offset form straight line
    straightLineError -= ERROR_OFFSET;
    if(straightLineError <= MIN_ERROR)
        straightLineError = MIN_ERROR;
    if(straightLineError > MAX_ERROR)
        straightLineError = MAX_ERROR;
    if(speed.data > ERROR_MIN_SPEED)
        speed.data -= straightLineError/(MAX_ERROR-MIN_ERROR)*SPEED_OFFSET;
    ROS_INFO("Speed:  %f",speed.data);
    
    // Publish speed and angle
    angle.data = error;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 
