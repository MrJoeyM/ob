/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */


#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <sensor_msgs/Image.h>
#include <tld_msgs/Target.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include<signal.h>

#include"ImageProc.h"
#include"PIDController.h"

#include "std_msgs/Float64.h"

using namespace DJI::onboardSDK;
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

#define PI 3.1415926  //圆周率常量

//! Function Prototypes for Mobile command callbacks - Core Functions
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);

//! For LAS logging
void StartMapLASLoggingMobileCallback(DJIDrone *drone);
void StopMapLASLoggingMobileCallback(DJIDrone *drone);
void StartCollisionAvoidanceCallback(DJIDrone *drone);
void StopCollisionAvoidanceCallback(DJIDrone *drone);

//自定义手机回调函数
void StartMission1Callback(DJIDrone *drone);
void StartMission2Callback(DJIDrone *drone);
void StartMission3Callback(DJIDrone *drone);
void StartMission4Callback(DJIDrone *drone);

void StopMissionCallback(DJIDrone *drone);


DJIDrone* drone;
//飞机姿态参数
float drone_pitch, drone_roll, drone_yaw;//俯仰角 横滚角 偏航角
float localPos_x, localPos_y, localPos_z ;//相对起飞位置

//图像处理
VideoCapture capture(0);
double imgWidth , imgHeight;//图像宽高
ImageProc imageProc;
IplImage *imgSrc;
IplImage *imgHSV;

//FOR TEST
int mission ;//任务代码
double k_forwardb, k_leftr;//height
double k_yaw, radius;//rotation
double pdi;//physical size of per pixels

//图像 ------>>>>飞行控制
double centerx, centery;//中心像素坐标
double targetx, targety;//target像素位置x,y;
double lastheadx, lastheady;//上一个端点坐标
double errorx, errory;//errorx、errory获得控制量
double lasterrorx, lasterrory;
double forwardb, leftr, height, yaw;//0X5B 前后v(-10 ~10m/s)、左右v、上下position、偏航v控制量

//mission2棋盘/TLD/kcf追踪
bool isFindChessboard;
bool isTargetTracking;
int counter = 0;//hesder
//TLD OR KCF tracking
bool ifTld;
bool ifKcf;
int blueDetectedFrame;//terminal

//PID
PIDController pidVX, pidVY, pidVYAW;//前后、上下、偏航PID控制

//回调函数
//drone
void attitude_quaternion_subscriber_callback(const dji_sdk::AttitudeQuaternion attitude_quaternion);//姿态信息回调函数
void global_position_subscriber_callback(const dji_sdk::GlobalPosition global_position);//全局位置回调
void local_position_subscriber_callback(const dji_sdk::LocalPosition local_position);//局部位置回调
//tld
void tracked_objectCallBack(const tld_msgs::BoundingBoxConstPtr & msg);//tld目标消息回调


//发布/订阅者
//openTLD
ros::Publisher imagePub;//publis tldimage
ros::Publisher msgPub;//publish "enter" key
ros::Subscriber bbSub;//subscriber tracker boundingbox
//drone data
ros::Subscriber attitude_quaternion_subscriber;//姿态消息回调
ros::Subscriber global_position_subscriber;//全局位置回调
ros::Subscriber local_position_subscriber;//本地定位回调
//PID node
ros::Publisher state_pub;//状态发布者
ros::Publisher setpoint_pub;//设定值发布者
std_msgs::Float64 setpoint;//设定值
std_msgs::Float64 state;//状态值

void imageReceived(Mat &srcImage);
// when ctrl + C execute
void Stop(int signo)
{
    capture.release();
//    cvReleaseImage( &imgSrc );
//    cvReleaseImage( &imgHSV );
    drone->landing();
    cout<<"landing";
    exit(0);
}

//main
int main(int argc, char *argv[])
{
    //dji code
    int main_operate_code = 0;
    int temp32;
    int circleRadius;
    int circleHeight;
    float Phi, circleRadiusIncrements;
    int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    //DJIDrone* drone = new DJIDrone(nh);
	drone = new DJIDrone(nh);

	//virtual RC test data
    uint32_t virtual_rc_data[6];

	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
    uint8_t userData = 0;
	

    //mission code
    ros::param::get("~mission",mission);//1循迹2追踪3闯关4室内任务
    ros::param::get("~ifTld",ifTld);//true
    ros::param::get("~ifKcf",ifKcf);//false
    ros::param::get("~imgWidth",imgWidth);//图像宽
    ros::param::get("~imgHeight",imgHeight);//图像高
    imgSrc = cvCreateImage( cvSize(imgWidth,imgHeight), 8, 3 );
    imgHSV = cvCreateImage( cvSize(imgWidth,imgHeight), 8, 3 );


    //控制量
    forwardb=0 , leftr=0, yaw=0;

	//飞机参数
     drone_pitch=0;//俯仰角
     drone_roll=0;//横滚角
     drone_yaw=0;//偏航角
     localPos_x=0;
     localPos_y=0;
     localPos_z=0;

	 //图像中心和目标像素距离
    errorx=0;
    errory=0;
    lasterrorx=0;
    lasterrory=0;
	

    //发布opentld追踪指令
    msgPub=nh.advertise<tld_msgs::Target>("bounding_box", 1, true);
    imagePub=nh.advertise<sensor_msgs::Image>("image", 1, true);
    bbSub=nh.subscribe("tracked_object",1,&tracked_objectCallBack);

    //drone data
    attitude_quaternion_subscriber=nh.subscribe("dji_sdk/attitude_quaternion",10,attitude_quaternion_subscriber_callback);
    global_position_subscriber = nh.subscribe("dji_sdk/global_position", 10,global_position_subscriber_callback);
    local_position_subscriber = nh.subscribe("dji_sdk/local_position", 10,local_position_subscriber_callback);

    //PID node
    state_pub = nh.advertise<std_msgs::Float64>("/state", 1000);
    setpoint_pub = nh.advertise<std_msgs::Float64>("/setpoint", 1000);

	ros::spinOnce();
	{   
		//! Setting functions to be called for Mobile App Commands mode 
		drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
		drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
		drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
		drone->setLandingMobileCallback(LandingMobileCallback, &userData);
		drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
		drone->setArmMobileCallback(ArmMobileCallback, &userData);
		drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
		drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
		drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
		drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
		drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
		drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
		drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
		drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
		drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
		drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
		drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
		drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
		drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

		drone->setStartMapLASLoggingMobileCallback(StartMapLASLoggingMobileCallback, &userData);
		drone->setStopMapLASLoggingMobileCallback(StopMapLASLoggingMobileCallback, &userData);
		drone->setStartCollisionAvoidanceCallback(StartCollisionAvoidanceCallback, &userData);
		drone->setStopCollisionAvoidanceCallback(StopCollisionAvoidanceCallback, &userData);
		
        drone->setStartMission1Callback(StartMission1Callback, &userData);//mission1
        drone->setStartMission2Callback(StartMission2Callback, &userData);//mission2
//        drone->setStartMission3Callback(StartMission3Callback, &userData);//mission3
//        drone->setStartMission4Callback(StartMission4Callback, &userData);//mission4
        drone->setStopMissionCallback(StopMissionCallback, &userData);//stop
	}

    //开启后立即获取权限 8s后手机或者onboardsdk发送指令

    signal(SIGINT,Stop);
 ros::AsyncSpinner spinner(4); // Use 4 threads
 spinner.start();

 ros::waitForShutdown();
//    ros::spin();
    return 0;
}

//自定义手机回调函数
void StartMission1Callback(DJIDrone *drone)
{
    drone->request_sdk_permission_control();
    sleep(1);// 8s
    cout<<"obtain control"<<endl;


    //obtain control & start mission
        //camera openfailed
            uint8_t a = (uint8_t)('a');
            uint8_t b= (uint8_t)('b');
           std::vector<uint8_t> data_;
            data_.push_back(a);
            data_.push_back(b);
            size_t len = data_.size();
          if(drone->sendDataToMobile(data_,len))
                 cout<<"helloworld";


//        drone->takeoff();
//        sleep(6);
    //mission=1;
    height=1.0;
    pidVX.setParam(0.8, 0, 0.2, 2);
    pidVY.setParam(0.6, 0, 0.2, 2);
    pidVYAW.setParam(5, 0, 0, 2);

    //FOR TEST
    k_yaw = 6.0;
    k_forwardb = 0.08;
    k_leftr = 0.20;//height
    //rotation
    radius=10.0;
    pdi=1;//physical size of per pixels


    //循迹端点初始化
    lastheadx=640;
    lastheady=240;

        //ros::spinOnce();//update
//        for(int i=0;i<100 ;i++)
//        {
//                if(i < 80)
//                    drone->attitude_control(0x5B, 0, 0, height, 0);
//                    usleep(10000);
//        }

          int lostFrame = 0;
          int framenum = 0;//to save picture
          if(!capture.isOpened())
          {
              cout<<"camera open failed"<<endl;
              drone->landing();
              //return to mobile
              //reopen
          }else{
              while(1)
              {
                  //ros::spinOnce();
                  lostFrame++;
                  if(lostFrame>100){
                      cout<<"frameLost"<<endl;
                      //return to mobile
                  }else{
                      ros::Time tic = ros::Time::now();
                      lostFrame=0;
                      Mat image;
                      capture>>image;

                      //to save picture
                      char path[255]={0};
                      memset(path,'\0',sizeof(char)*255);
                     char prefix[]="/home/exbot/图片/Webcam/";
                     char postfix[]=".jpg";
                      framenum++;
                      sprintf(path,"%sframe_%04d%s",prefix,framenum,postfix);
                      imwrite(path,image);
                      //to save picture

                      imageReceived(image);
                      ros::Duration toc = (ros::Time::now() - tic);
                      //cout<<"duratime"<<toc<<endl;
                  }
              }//while
          }
        cout<<"takeoff"<<endl;
}

void StartMission2Callback(DJIDrone *drone)
{
//        drone->takeoff();
//        sleep(6);
    //mission=2;
    height=3.0;
    pidVX.setParam(0.45, 0, 0, 2);
    pidVYAW.setParam(150, 30, 0, 2);
    isFindChessboard = false;
    isTargetTracking = false;

        //ros::spinOnce();//update
        for(int i=0;i<200 ;i++)
        {
                if(i < 180)
                    drone->attitude_control(0x5B, 0, 0, height, 0);
                    usleep(10000);
        }

          int lostFrame = 0;
          if(!capture.isOpened())
          {
              cout<<"camera open failed"<<endl;
              drone->landing();
              //return to mobile
              //reopen
          }else{
              while(1)
              {
                  ros::spinOnce();
                  lostFrame++;
                  if(lostFrame>100){
                      cout<<"frameLost"<<endl;
                      //return to mobile
                  }else{
                      lostFrame=0;
                      Mat image;
                      capture>>image;

                      ros::Time tic = ros::Time::now();
                      imageReceived(image);
                      ros::Duration toc = (ros::Time::now() - tic);
                     // cout<<"duratime"<<toc<<endl;
                  }
              }//while
          }
        cout<<"takeoff"<<endl;
}


//图像接收chuli
void imageReceived(Mat &srcImage)
{
    imgWidth=srcImage.cols;//图像宽
    imgHeight=srcImage.rows;//图像高
  #define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}
    centery= imgHeight/2;// - tan(drone_pitch * PI/180) * 800;//飞机飞行时摄像头会有个倾角，倾角校正0.5104*thread.navdata.altd;（像素坐标）
    centerx= imgWidth/2;// + tan(drone_roll * PI/180) * 800;//0.5104*thread.navdata.altd;
    CLIP3(10.0, centerx, 630.0);
    CLIP3(10.0, centery, 470.0);

    targetx=centerx;
    targety=centery;
    errorx=0;errory=0;
    forwardb = 0, leftr=0, yaw=0;

   if(mission ==1)//循迹飞行
    {

       /*camera is hengzhede
        *  (pixerrory targetvx)forwardb
        * ^
        * |
        * |
        * -------->(pixerrorx targetvy)leftr      >>>>>fly direction
       */
       double targetvx=0 , targetvy=0;//FOR PID
        int NUM_RED = 0;////红色轨迹点的个数
        bool isBlueDetected = false;//蓝色点检测标志

        //图像处理
        IplImage image = IplImage (srcImage);//cvImagePtr->image Mat类型
        imgSrc = &image;
        cvSmooth(imgSrc, imgSrc, CV_GAUSSIAN,3,3,0,0);//added 4.11
        cvCvtColor(imgSrc, imgHSV, CV_BGR2HSV);//转换为HSV空间

        NUM_RED=0;//红色轨迹
        isBlueDetected = false;//蓝色点
        //结果点
        CvPoint   *redPoints , bluePoint;
        redPoints  = new CvPoint[20];
         bluePoint.x=0;bluePoint.y=0;
         redPoints = imageProc.findRedContours(imgHSV,NUM_RED);
         bluePoint = imageProc.findBlueContour(imgHSV,isBlueDetected);

         //redPoint
         for(int i=0;i<NUM_RED;i++)
         {
             Point redCenter(redPoints[i].x,redPoints[i].y);
             circle(srcImage, redCenter,50, CV_RGB(255, 0,0),3, 8, 0);
         }
         Point redCenter(centerx,centery);
         circle(srcImage, redCenter,4, CV_RGB(255, 0,0),3, 8, 0);

         //bluePoint
         if(isBlueDetected)
         {
             Point blueCenter(bluePoint.x, bluePoint.y);
            circle(srcImage, blueCenter,50, CV_RGB(0, 0,255),3, 8, 0);
         }


        //飞行策略
         double slope = 0;//forecast
        //检测到红色轨迹
        if(NUM_RED > 0)
        {
            cout<<"circle num:"<<NUM_RED<<endl;
            //按端点飞行 no yaw
            if(false)
            {
                int i,j;
                double dist, mindist[100], mindist2[200];//mindist最近距离，mindist2次近距离
                double headdist, minheaddist=1e8;
                double newheadx = 0, newheady=0;
                double x1,y1,x2,y2;//计算端点中间参数
                for (i=0; i<NUM_RED; ++i)
                {
                    mindist[i]=1e8;
                    mindist2[i]=1e8;
                    x1=redPoints[i].x;
                    y1=redPoints[i].y;
                    for (j=0; j<NUM_RED; ++j)
                    {
                        if (i != j)
                        {
                            //摄像头右侧朝前，设定处理范围减少计算。
                            if(redPoints[j].x > 150)
                            {
                               x2=redPoints[j].x;
                               y2=redPoints[j].y;
                               dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
                               if (dist < mindist[i]) {
                                   mindist2[i]=mindist[i];
                                   mindist[i]=dist;
                               } else if (dist < mindist2[i]){
                                   mindist2[i]=dist;
                               }
                            }
                        }//if
                    }//for j

                    //如果次近距离是最近距离两倍，则为端点
                    if (mindist2[i] > (mindist[i] * 1.7) )
                    {
                        //计算和上次端点距离,防止找到左边端点
                        headdist=sqrt((x1-lastheadx)*(x1-lastheadx) + (y1-lastheady)*(y1-lastheady));
                        if (headdist < 200)
                        if (headdist < minheaddist)
                        {
                            minheaddist = headdist;
                            //本次端点位置
                            newheadx= x1;
                            newheady= y1;
                            //上次端点位置
                            lastheadx = newheadx;
                            lastheady = newheady;
                        }
                    }

                }//for i
                targetx=newheadx;
                targety=newheady;
            }

            //we choose this mode
            //yaw
            if(true){
                //method 2
                double x_1, y_1, x_2, y_2;//计算飞行路径
                //turn = (180 - BlobsRed.GetBlob(0)->y) / 256;
                x_1=imgWidth/2; y_1=imgHeight/2; x_2=imgWidth/2; y_2=imgHeight/2;
                if (NUM_RED==1){
                    x_1 = redPoints[0].x;
                    y_1 = redPoints[0].y;
                    targetx=x_1;
                    targety=y_1;
                }
                else if (NUM_RED==2) {
                    x_1 = (redPoints[0].x + redPoints[1].x)/2 ;
                    y_1 = (redPoints[0].y + redPoints[1].y)/2;
                    targetx=x_1;
                    targety=y_1;
                }
                else if (NUM_RED==3){
                    x_1 = redPoints[0].x;
                    y_1 = redPoints[0].y;
                    x_2 = redPoints[2].x ;
                    y_2 = redPoints[2].y;
                    targetx=(redPoints[0].x +redPoints[1].x)/2;
                    targety=(redPoints[0].y+ 2*redPoints[1].y + redPoints[2].y)/4;
                }else {
                    x_1 = (redPoints[0].x +redPoints[1].x)/2 ;
                    y_1 = (redPoints[0].y + redPoints[1].y)/2;
                    x_2 = (redPoints[2].x +redPoints[3].x)/2;
                    y_2 = (redPoints[2].y + redPoints[3].y)/2;
                    targetx=(redPoints[0].x +redPoints[1].x)/2;
                    targety=(redPoints[0].y+ 2*redPoints[1].y + redPoints[2].y)/4;
                }
                if (x_1==x_2)
                    slope =0;
                else
                    slope =  (y_1-y_2)/(x_1-x_2);
                CLIP3(-10, slope, 10);
                //slope=atan2(y_1-y_2,x_1-x_2);

                //左旋转theta dji -180 度 ~ 180 度
                yaw =(targety - centery) /260;//k * detaY
                if (NUM_RED>=3)
                    yaw += slope/2;//前瞻性预测
                yaw=k_yaw * yaw;
            }

            //拟合直线飞行
            if(false)
            {
                double a,b, t1=0, t2=0, t3=0, t4=0; ;//拟合参数
                double x_1, y_1, x_2, y_2;
                if (NUM_RED==1){
                    x_1 = redPoints[0].x;
                    y_1 = redPoints[0].y;
                }
                else if (NUM_RED==2) {
                    x_1 = (redPoints[0].x + redPoints[1].x)/2 ;
                    y_1 = (redPoints[0].y + redPoints[1].y)/2;
                }
                else if (NUM_RED==3){
                    x_1 = redPoints[0].x;
                    y_1 = redPoints[0].y;
                    x_2 = redPoints[2].x ;
                    y_2 = redPoints[2].y;
                }else {
                    for (int i=0; i<NUM_RED; ++i)
                    {
                        t1 += redPoints[i].x * redPoints[i].x;
                        t2 += redPoints[i].x;
                        t3 += redPoints[i].x * redPoints[i].y;
                        t4 += redPoints[i].y;
                    }
                }
                a = (t3 * NUM_RED - t2*t4) / (t1 * NUM_RED - t2*t2);  // 求得β1
                b = (t1*t4 - t2*t3) / (t1 * NUM_RED - t2*t2);        // 求得β2
                targetx=imgWidth*3/4;
                targety=a * targetx+b;

                Point pt1(targetx,targety);
                Point pt2(imgWidth/2,imgHeight/2);
                line(srcImage, pt1, pt2, Scalar( 255, 0, 0 ) ,2,8);
            }


            //增量式PID控制 output
            double addx,addy;
            //目标像素坐标和平面中心误差
            lasterrorx = errorx;
            lasterrory = errory;
            errorx = double(targetx - centerx)/(imgWidth/2);//归一化
            errory = double(centery - targety)/(imgHeight/2);//归一化

            //distance 2 velocity
            if( abs(centery - targety) >25 )
            {
                // targetvx is for pid
                targetvx = k_forwardb * errory + k_forwardb * (errory-lasterrory); // method 1:
                //
               // addx = pidVX.getOutputByAdd( targetvx - drone->velocity.vx);
                forwardb = targetvx;//drone->velocity.vx + addx ;
                CLIP3(-0.1, forwardb, 0.1); //-10 m/s ~ 10 m/s
            }else{
                forwardb=0;
            }


            if(abs(targetx - centerx) > 10)
            {
                if( yaw < 0.2)
                {
                    targetvy = k_leftr * errorx + k_leftr * (errorx-lasterrorx); // method 1:
                }else
                {
                      targetvy = 0.10 + radius*abs(yaw);
                }
                // targetvy is for pid              
               //addy = pidVY.getOutputByAdd( targetvy - drone->velocity.vy);
                 leftr  = targetvy;//drone->velocity.vy + addy ;
                 CLIP3(-0.35, leftr, 0.35);
            }else{
                leftr=0;
            }
        }

        //检测到终点
        if (isBlueDetected)
        {
                blueDetectedFrame++;
                targetx=bluePoint.x;
                targety=bluePoint.y;
                errorx = targetx - centerx;//归一化
                errory = centery - targety;//归一化

                if (abs(errorx) <80 && abs(errory) <80&& blueDetectedFrame>5)
                {
                    forwardb = 0;
                    leftr=0;
                    yaw=0;
                }else{
                    yaw=0;
                    forwardb = 0.1*errory/abs(errory);//-0.1 / +0.1
                    leftr= 0.1*errorx/abs(errorx);//-0.1 / +0.1
                }
        }

        Point targetPoint(targetx, targety);
        circle(srcImage, targetPoint,5, CV_RGB(0, 255,0),3, 8, 0);

        Point minPoint_blue(240,160); Point maxPoint_blue(400,320);
        rectangle(srcImage, minPoint_blue, maxPoint_blue,  (100, 100, 100),1,8,0);
        imshow("result",srcImage);
        waitKey(10);

      //  int Fy =287;//Fy值待确定，(v-imgHeight/2)/Fy,Fy=ydirection
        //单位像素长度/焦距。得到与摄像机光轴夹角

         cout<< "errory:"<<targety - centery<< " errorx:"<<targetx - centerx<<endl<<
//                    "targetvx:"<<targetvx<< " targetvy:"<<targetvy<<endl<<
//                      "dronevx:"<<drone->velocity.vx<< "  dronevy:"<<drone->velocity.vy<<endl<<
//                    "addx:"<<addx<< " addy:"<<addy<<endl<<
                   "forwardb:"<<forwardb<<" leftr: "<<leftr<<endl<<
               " yaw:"<<yaw<<" slope:"<<slope<<" droneyaw:"<<drone_yaw*180/PI<<endl;
         cout<<"--------------"<<endl;

         //ros PID
//        state.data = drone->velocity.vx;
//        setpoint.data = forwardb;
//        state_pub.publish(state);
//        setpoint_pub.publish(setpoint);

        drone->attitude_control(0x5B, forwardb , leftr, height,yaw);// height=1.0
//        usleep(20000);

    }//mission1

   if(mission == 2)//人物追踪
    {
         forwardb=0, leftr=0,yaw=0;
        Rect targetRect ;
        if( ! isFindChessboard  && ! isTargetTracking ) //棋盘检测追踪
                 targetRect = imageProc.cheesePlaneDetector(srcImage , isFindChessboard);
        if(isFindChessboard   && ! isTargetTracking )
        {
            //TLD OR KCF maybe failed launch
            isFindChessboard=false;
            isTargetTracking=false;
            if( ifTld)
          {
                //发布openTLD追踪
               tld_msgs::Target msg;
               msg.bb.x =targetRect.x;
               msg.bb.y =targetRect.y;
               msg.bb.width = targetRect.width;
               msg.bb.height = targetRect.height;
               msg.bb.confidence = 1.0;
               cout<<"width"<<targetRect.width<<" height"<<targetRect.height<<endl;

              cv_bridge::CvImage cvImage;
              std_msgs::Header header; // empty header
              header.seq = counter; // user defined counter
              header.stamp = ros::Time::now(); // time
              cvImage = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, srcImage);
              cvImage.toImageMsg(msg.img);
               msgPub.publish(msg);
           }else if(ifKcf)
            {

            }
       }else if(! isFindChessboard){
       //no cheeseboard
        //drone ->rotate
       }
  }//mission2

   if(mission == 3)//降落停机坪
    {
        bool isFindSquares = false;
       //矩形结果
        vector<Mat> resultImages;
        vector<Point> resultCenters;
        isFindSquares= imageProc.findSquares(srcImage,resultImages, resultCenters);
        if(isFindSquares)
            for(int i=0; i < resultImages.size(); i++)
            {
                //imshow("result111",resultImages[i]);
                int number=imageProc.getNumber(resultImages[i]);
                cout<<number<<"\t";
            }
        cout<<endl;
        //cout<<"-------------------\n";

      // cout<< "num of squares" <<;
//        if(isFindSquares)
//        {
//            imshow("结果图", rectResult);
//              waitKey(10);
//        }else
//        {
//            imshow("结果图", srcImage);
//              waitKey(10);
//        }
    }

   if(mission == 4)//室内任务
   {
   }
}

void StopMissionCallback(DJIDrone *drone)
{
    //实现自己的代码
}
    
//目标追踪到回调函数
void tracked_objectCallBack(const tld_msgs::BoundingBoxConstPtr & msg)
{
    if(msg->x > 1 && msg->y> 1 )
      {
        isFindChessboard=true;
        isTargetTracking=true;
        targetx=msg->x+msg->width/2;
        targety=msg->y+msg->height/2;

        #define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}

        centery= imgHeight/2 - tan(drone_pitch * PI/180) * 800;//飞机飞行时摄像头会有个倾角，倾角校正0.5104*thread.navdata.altd;（像素坐标）
        centerx= imgWidth/2 + tan(drone_roll * PI/180) * 800;//0.5104*thread.navdata.altd;
        CLIP3(10.0, centerx, 630.0);
        CLIP3(10.0, centery, 470.0);

        //目标像素坐标和成像平面中心误差
        lasterrorx = errorx;
        lasterrory = errory;
        //            errorx = double(target.x - centerx)/(imgWidth/2);//归一化
        //            errory = double(centery - target.y)/(imgHeight/2);//归一化
        errorx = targetx - centerx;//errorx  in [-w/2,w/2]
        errory = centery - targety ;//errory  in [-h/2,h/2]

        int Fy =287;//Fy值待确定，(v-imgHeight/2)/Fy,Fy=ydirection 单位像素长度/焦距。得到与摄像机光轴夹角
        double err_theta_forwardb=atan2(errory,Fy);

         //增量式PID控制
        double camera_incline_angle = 45;
           double pid_forwardb = pidVX.getOutputByAdd(err_theta_forwardb);
           forwardb=localPos_z/tan(pid_forwardb + camera_incline_angle)
                           - localPos_z/tan(pid_forwardb + camera_incline_angle + pid_forwardb);
          yaw = yaw + pidVYAW.getOutputByAdd(errorx);


          CLIP3(-30, yaw, 30);//-180 度 ~ 180 度
           drone->attitude_control(0x4B, 0, 0, 0, yaw);
           usleep(20000);

          //forwarddb
          state.data = drone->velocity.vx;
          setpoint.data = forwardb;
          state_pub.publish(state);
          setpoint_pub.publish(setpoint);

        //scale
        //            forwardb = 0.05*errory+0.05*(errory-lasterrory);
        //            yaw = 0.1*errorx;// + 0.1*(errorx-lasterrorx);




      //          CLIP3(-1.8, forwardb, 1.8); //-10 m/s ~ 10 m/s
      //          CLIP3(-0.5, leftr, 0.5);
      //          CLIP3(-0.2, yaw, 0.3);//-4 m/s ~ 4 m/s
      drone->attitude_control(0x4B, forwardb, 0, height, 0);
      usleep(20000);
      cout<< "errorx:"<<errorx<<" errory"<<errory<<endl<<
          "forwardb:"<<forwardb<<"leftr: "<<leftr<<" yaw:"<<yaw<<endl;
      cout<<"--------------"<<endl;

      }else{
        isFindChessboard=false;
        isTargetTracking=false;
    }
}


/*---------任务相关函数---------*/
//姿态信息回调函数
void attitude_quaternion_subscriber_callback(const dji_sdk::AttitudeQuaternion attitude_quaternion)
{
              float ax,ay,az,aw;
              ax=attitude_quaternion.q0;
              ay=attitude_quaternion.q1;
              az=attitude_quaternion.q2;
              aw=attitude_quaternion.q3;
             double q2sqr = attitude_quaternion.q2 * attitude_quaternion.q2;
             double t0 = -2.0 * (q2sqr + attitude_quaternion.q3 * attitude_quaternion.q3) + 1.0;
             double t1 = +2.0 * (attitude_quaternion.q1 * attitude_quaternion.q2 + attitude_quaternion.q0 * attitude_quaternion.q3);
             double t2 = -2.0 * (attitude_quaternion.q1 * attitude_quaternion.q3 - attitude_quaternion.q0 * attitude_quaternion.q2);
             double t3 = +2.0 * (attitude_quaternion.q2 * attitude_quaternion.q3 + attitude_quaternion.q0 * attitude_quaternion.q1);
             double t4 = -2.0 * (attitude_quaternion.q1 * attitude_quaternion.q1 + q2sqr) + 1.0;

             t2 = t2 > 1.0 ? 1.0 : t2;
             t2 = t2 < -1.0 ? -1.0 : t2;
            drone_pitch = asin(t2);
            drone_roll = atan2(t3, t4);
            drone_yaw = atan2(t1, t0);
}

//全局位置回调
void global_position_subscriber_callback(const dji_sdk::GlobalPosition global_position)
{
	  float latitude,longitude,altitude,height;
	  int health;
	  latitude=global_position.latitude;
	  longitude=global_position.longitude;
	  altitude=global_position.altitude;
	  height=global_position.height;
	  health=global_position.health;
}

//局部位置回调
void local_position_subscriber_callback(const dji_sdk::LocalPosition local_position)
{       
	localPos_x=local_position.x;
	localPos_y=local_position.y;
	localPos_z=local_position.z;
}


//demo菜单
static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [1]  SDK Version Query        | [20] Set Sync Flag Test          |\n");
	printf("| [2]  Request Control          | [21] Set Msg Frequency Test      |\n");	
	printf("| [3]  Release Control          | [22] Waypoint Mission Upload     |\n");	
	printf("| [4]  Takeoff                  | [23] Hotpoint Mission Upload     |\n");	
	printf("| [5]  Landing                  | [24] Followme Mission Upload     |\n");	
	printf("| [6]  Go Home                  | [25] Mission Start               |\n");	
	printf("| [7]  Gimbal Control Sample    | [26] Mission Pause               |\n");	
	printf("| [8]  Attitude Control Sample  | [27] Mission Resume              |\n");	
	printf("| [9]  Draw Circle Sample       | [28] Mission Cancel              |\n");	
	printf("| [10] Draw Square Sample       | [29] Mission Waypoint Download   |\n");	
	printf("| [11] Take a Picture           | [30] Mission Waypoint Set Speed  |\n");	
	printf("| [12] Start Record Video       | [31] Mission Waypoint Get Speed  |\n");	 
	printf("| [13] Stop Record Video        | [32] Mission Hotpoint Set Speed  |\n");	
	printf("| [14] Local Navigation Test    | [33] Mission Hotpoint Set Radius |\n");	
	printf("| [15] Global Navigation Test   | [34] Mission Hotpoint Reset Yaw  |\n");	
	printf("| [16] Waypoint Navigation Test | [35] Mission Followme Set Target |\n");	
	printf("| [17] Arm the Drone            | [36] Mission Hotpoint Download   |\n");	
	printf("| [18] Disarm the Drone         | [37] Enter Mobile commands mode  |\n");
    printf("| [19] Virtual RC Test           \n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input 1/2/3 etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}





/*-----手机发送指令控制OnboardSdk执行任务------nboardsdk执行任务------*/
//Callback functions for Mobile Commands
//OnboardSdk获得权限,注MobileSDK没必要获取权限
void ObtainControlMobileCallback(DJIDrone *drone)
{
  drone->request_sdk_permission_control();
}
//OnboardSdk释放权限
void ReleaseControlMobileCallback(DJIDrone *drone)
{
  drone->release_sdk_permission_control();
}
//起飞
void TakeOffMobileCallback(DJIDrone *drone)
{
  drone->takeoff();
}
//降落
void LandingMobileCallback(DJIDrone *drone)
{
  drone->landing();
}
//获得SDK版本
void GetSDKVersionMobileCallback(DJIDrone *drone)
{
  drone->check_version();
}
//Arm
void ArmMobileCallback(DJIDrone *drone)
{
  drone->drone_arm();
}
//DisArm
void DisarmMobileCallback(DJIDrone *drone)
{
  drone->drone_disarm();
}
//一键返回
void GoHomeMobileCallback(DJIDrone *drone)
{
  drone->gohome();
}
//拍照
void TakePhotoMobileCallback(DJIDrone *drone)
{
  drone->take_picture();
}
//开始摄像
void StartVideoMobileCallback(DJIDrone *drone)
{
  drone->start_video();
}
//停止摄像
void StopVideoMobileCallback(DJIDrone *drone)
{
  drone->stop_video();
}
//飞机飞圆形
void DrawCircleDemoMobileCallback(DJIDrone *drone)
{
    static float R = 2;
    static float V = 2;
    static float x;
    static float y;
    int circleRadius;
    int circleHeight;
    float Phi =0, circleRadiusIncrements;
    int x_center, y_center, yaw_local;

    circleHeight = 7;
    circleRadius = 7;

    x_center = drone->local_position.x;
    y_center = drone->local_position.y;
    circleRadiusIncrements = 0.01;

    for(int j = 0; j < 1000; j ++)
    {
        if (circleRadiusIncrements < circleRadius)
        {
            x =  x_center + circleRadiusIncrements;
            y =  y_center;
            circleRadiusIncrements = circleRadiusIncrements + 0.01;
            drone->local_position_control(x ,y ,circleHeight, 0);
            usleep(20000);
        }
            else
        {
            break;
        }
    }


    /* start to draw circle */
    for(int i = 0; i < 1890; i ++)
    {
        x =  x_center + circleRadius*cos((Phi/300));
        y =  y_center + circleRadius*sin((Phi/300));
        Phi = Phi+1;
        drone->local_position_control(x ,y ,circleHeight, 0);
        usleep(20000);
    }

}
//飞机飞正方形
void DrawSquareDemoMobileCallback(DJIDrone *drone)
{
/*draw square sample*/
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
        Flight::VerticalLogic::VERTICAL_VELOCITY |
        Flight::YawLogic::YAW_ANGLE |
        Flight::HorizontalCoordinate::HORIZONTAL_BODY |
        Flight::SmoothMode::SMOOTH_ENABLE,
        3, 3, 0, 0 );
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
        Flight::VerticalLogic::VERTICAL_VELOCITY |
        Flight::YawLogic::YAW_ANGLE |
        Flight::HorizontalCoordinate::HORIZONTAL_BODY |
        Flight::SmoothMode::SMOOTH_ENABLE,
        -3, 3, 0, 0);
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
        Flight::VerticalLogic::VERTICAL_VELOCITY |
        Flight::YawLogic::YAW_ANGLE |
        Flight::HorizontalCoordinate::HORIZONTAL_BODY |
        Flight::SmoothMode::SMOOTH_ENABLE,
        -3, -3, 0, 0);
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
        Flight::VerticalLogic::VERTICAL_VELOCITY |
        Flight::YawLogic::YAW_ANGLE |
        Flight::HorizontalCoordinate::HORIZONTAL_BODY |
        Flight::SmoothMode::SMOOTH_ENABLE,
        3, -3, 0, 0);
        usleep(20000);
    }
}
//万向节控制
void GimbalControlDemoMobileCallback(DJIDrone *drone)
{
    drone->gimbal_angle_control(0, 0, 1800, 20);
    sleep(2);
    drone->gimbal_angle_control(0, 0, -1800, 20);
    sleep(2);
    drone->gimbal_angle_control(300, 0, 0, 20);
    sleep(2);
    drone->gimbal_angle_control(-300, 0, 0, 20);
    sleep(2);
    drone->gimbal_angle_control(0, 300, 0, 20);
    sleep(2);
    drone->gimbal_angle_control(0, -300, 0, 20);
    sleep(2);
    drone->gimbal_speed_control(100, 0, 0);
    sleep(2);
    drone->gimbal_speed_control(-100, 0, 0);
    sleep(2);
    drone->gimbal_speed_control(0, 0, 200);
    sleep(2);
    drone->gimbal_speed_control(0, 0, -200);
    sleep(2);
    drone->gimbal_speed_control(0, 200, 0);
    sleep(2);
    drone->gimbal_speed_control(0, -200, 0);
    sleep(2);
    drone->gimbal_angle_control(0, 0, 0, 20);
}
//姿态控制
void AttitudeControlDemoMobileCallback(DJIDrone *drone)
{
    /* attitude control sample*/
    drone->takeoff();
    sleep(8);


    for(int i = 0; i < 100; i ++)
    {
        if(i < 90)
            drone->attitude_control(0x40, 0, 2, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 2, 0, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, -2, 0, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 0, 2, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 0, -2, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 0, 0, 0.5, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 0, 0, -0.5, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0xA, 0, 0, 0, 90);
        else
            drone->attitude_control(0xA, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0xA, 0, 0, 0, -90);
        else
            drone->attitude_control(0xA, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    drone->landing();

}
//导航（任务4）
void LocalNavigationTestMobileCallback(DJIDrone *drone)
{

}
void GlobalNavigationTestMobileCallback(DJIDrone *drone)
{

}
void WaypointNavigationTestMobileCallback(DJIDrone *drone)
{

}
void VirtuaRCTestMobileCallback(DJIDrone *drone)
{
    //virtual RC test data
    uint32_t virtual_rc_data[16];
    //virtual rc test 1: arm & disarm
    drone->virtual_rc_enable();
    usleep(20000);

    virtual_rc_data[0] = 1024;  //0-> roll      [1024-660,1024+660]
    virtual_rc_data[1] = 1024;  //1-> pitch     [1024-660,1024+660]
    virtual_rc_data[2] = 1024+660;  //2-> throttle  [1024-660,1024+660]
    virtual_rc_data[3] = 1024;  //3-> yaw       [1024-660,1024+660]
    virtual_rc_data[4] = 1684;      //4-> gear      {1684(UP), 1324(DOWN)}
    virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

    for (int i = 0; i < 100; i++){
        drone->virtual_rc_control(virtual_rc_data);
        usleep(20000);
    }

    //virtual rc test 2: yaw
    drone->virtual_rc_enable();
    virtual_rc_data[0] = 1024;      //0-> roll      [1024-660,1024+660]
    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
    virtual_rc_data[2] = 1024-200;  //2-> throttle  [1024-660,1024+660]
    virtual_rc_data[3] = 1024;      //3-> yaw       [1024-660,1024+660]
    virtual_rc_data[4] = 1324;      //4-> gear      {1684(UP), 1324(DOWN)}
    virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

    for(int i = 0; i < 100; i++) {
        drone->virtual_rc_control(virtual_rc_data);
        usleep(20000);
    }
    drone->virtual_rc_disable();
}

void StartMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("roslaunch point_cloud_las start_velodyne_and_loam.launch &");
  system("rosrun point_cloud_las write _topic:=/laser_cloud_surround _folder_path:=. &");
}

void StopMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("rosnode kill /write_LAS /scanRegistration /laserMapping /transformMaintenance /laserOdometry  &");
}
//开始逼障
void StartCollisionAvoidanceCallback(DJIDrone *drone)
{
  uint8_t freq[16];
  freq[0] = 1;    // 0 - Timestamp
  freq[1] = 4;    // 1 - Attitude Quaterniouns
  freq[2] = 1;    // 2 - Acceleration
  freq[3] = 4;    // 3 - Velocity (Ground Frame)
  freq[4] = 4;    // 4 - Angular Velocity (Body Frame)
  freq[5] = 3;    // 5 - Position
  freq[6] = 0;    // 6 - Magnetometer
  freq[7] = 3;    // 7 - M100:RC Channels Data, A3:RTK Detailed Information
  freq[8] = 0;    // 8 - M100:Gimbal Data, A3: Magnetometer
  freq[9] = 3;    // 9 - M100:Flight Status, A3: RC Channels
  freq[10] = 0;   // 10 - M100:Battery Level, A3: Gimble Data
  freq[11] = 2;   // 11 - M100:Control Information, A3: Flight Status

  drone->set_message_frequency(freq);
  usleep(1e4);
  system("roslaunch dji_collision_avoidance from_DJI_ros_demo.launch &");
}
//停止逼障
void StopCollisionAvoidanceCallback(DJIDrone *drone)
{
  drone->release_sdk_permission_control();
  system("rosnode kill /drone_tf_builder /dji_occupancy_grid_node /dji_collision_detection_node /collision_velodyne_nodelet_manager /manual_fly");
  usleep(1e4);
  drone->request_sdk_permission_control();
}
