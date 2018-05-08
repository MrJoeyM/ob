#ifndef _Image_Proc_H_
#define _Image_Proc_H_

#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include <string.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/legacy/legacy.hpp"


#include <vector>
#include <stdlib.h>
#include "svm.h"


#define xBOXm(box)   (unsigned int)(box.center.x - box.size.width/2.0)
#define yBOXm(box)   (unsigned int)(box.center.y - box.size.height/2.0)
#define max(a,b) ( ((a)>(b)) ? (a):(b) )

//HSV
#define red 2,140,140,0
#define blue 103,80,142,0
#define green 57,90,143,0
#define ControlDelays 30

#define pi 3.1415926  //圆周率常量

using namespace std;
using namespace cv;

class ImageProc
{
public:
    ImageProc();
   ~ImageProc();
    int size_width,size_height;//帧尺寸
int static framenum;

	//控制高度
    volatile int altitude;// mm
	int alt_ind;
    volatile double set_alt;
	
    /*-------轨迹检测--------*/
    //目标HSV阀值
    CvScalar green_lower,green_upper;
    CvScalar red_lower,red_upper;
    CvScalar blue_lower,blue_upper;
	
	IplImage* imgThresh_green;
    IplImage* imgThresh_red;
    IplImage* imgThresh_blue;

	//储存结果点
	CvPoint greenPoint;
	CvPoint redPoints[20];
	CvPoint bluePoint;
	
    CvFont font;
    //cvInitFont;
	
    int w;
    double last_yaw,ec_yaw;
    
	int frame_count;
	
	int IsSizeEqual( CvSize x, CvSize y );
	//void MakeControlZero(control_data_t *l_control);
	int my_round(float t);
	//void limit_control(control_data_t *l_control);
	float absfl( float num );
	//void printControlData(control_data_t src);
	void debugDumpMatrix (CvMat *A,const char *Title);
	float calcDistHSI( CvScalar pix,CvScalar sclr);
        //Calculate the Distance between two Scalars
	
	//反向投影
	int BackProjectHSI( IplImage *src,IplImage *dst, CvScalar sclr ); 
    // Get a Image corresponding
    void itoa_ht(float data, char s[], int pos);	
	//void AddText(IplImage *img,control_data_t tmp);
	
	//计算x平均值
	int CalAvgX(CvPoint * points ,int count);
	//计算y平均值
	int CalAvgY(CvPoint * points ,int count);
	//给点集排序，选择排序
	void SortPoints(CvPoint * points, int count );
	int IsAligned(CvPoint * points, int count );
	//速度控制
	void SpeedControl(double set_speed,double real_speed);
	
	//轨迹检测函数
        void clip(int n1,int& n,int n2);
	bool isCircle(Mat monoImage , CvPoint contoursCenter);
	CvPoint findGreenContour(IplImage* image, bool &isGreenDetected);//搜索绿色起点
	CvPoint* findRedContours(IplImage* image, int &NUM_RED);//搜索红色轨迹
	CvPoint findBlueContour(IplImage* image , bool &isBlueDetected);//搜索蓝色终点
	double lastheadx, lastheady;
	
	
	
	//矩形检测
        double scale;
	int thresh ;//Canny算子边缘检测的参数
       //计算向量 pt0->pt1， pt0->pt2夹角
       Point getPointAffinedPos(const Point &src, const Point center, double angle);
        bool isYellowSquare(Mat& image_H,vector<Point>&square);
	double getAngle(Point pt1, Point pt2, Point pt0);
        void cutImg( Mat& image, vector<vector<Point> >& squares,vector<Mat>& rectResult);
	//void BackProject(Mat& image_H, Mat& templateMat_H , Mat& backprojResult);
	// returns sequence of squares detected on the image
	bool getSquarePts(const Mat& image, vector<vector<Point> >& squares);
	// the function draws all the squares in the image
	void drawSquares(Mat& image, const vector<vector<Point> >& squares);
	bool findSquares(Mat &image, vector<Mat>& resultImage,vector<Point>& center);
	
	//数字检测
        FILE *input1, *input2;
        Mat mA, v;
        int FeaturesSize,PcaSize;
	char *tem_line;//定义txt文件的每一行
	int max_line_len;
	struct svm_node *x;//定义预测时的特征向量
	int max_nr_attr;
	struct svm_model* model;//定义训练模型
	int predict_probability;
	
	
	int getNumber(Mat &image);//数字识别主函数
	//提取LBP特征
	void ComputeLBPFeatureVector_256(const Mat &srcImage, Size cellSize, Mat &featureVector);
	void ComputeLBPImage_256(const Mat &srcImage, Mat &LBPImage);// 计算256维LBP特征图
	//利用事先得到的均值矩阵和投影矩阵对LBP特征进行降维处理
	void fastPCA(Mat tem_feattureVector, Mat &featureVector);
	void OpenPCAData(FILE *input, Mat &result, int width, int height);
	//读取txt的每一行
	char* readline(FILE *input);
	//进行预测
	double svmpredict(Mat &featureVector);
	//void my_imresize(Mat matSrc, Mat &matDst1, int height, int width);
        void my_imresize(Mat matSrc, Mat &matDst1, int height, int width);
	
	
	/*------棋盘检测------*/
	Rect cheesePlaneDetector(Mat &imgRGB , bool &isFindChessboard);
};

#endif



