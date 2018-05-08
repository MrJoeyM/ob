#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <assert.h>
#include <iostream>  
using namespace std;

class PIDController
{
public:
    //coefficients（系数） for P, I, D
    double _kP, _kI, _kD;
    ///coefficient for filtering the derivative values
    double _kN;
    double _P, _I, _D;
    bool _bFirstFrame;
    double prev_err, last_err, cur_err;//上上次误差(用于增量式PID)、上次误差、本次误差
    double errs[20];
    /* error integration*/
    double cur_I;
    /* error derivative*/
    double old_D;
	double lastPIDOut;//上一次PID控制结果

    PIDController():_kP(0.0),_kI(0.0),_kD(0.0){
		reset();
		cout<<"hello";
	}
    void setParam(double kP, double kI, double kD, double kN);

    void reset();
    double getOutputByPos(double curerr);//位置式PID控制
	
	//参数：
	//realVal  系统真实值
	//targetVal目标值
	//curerr   当前误差（初始为0）
	//lasterr  上次误差（初始为0）
	//lastPIDOut上次控制量
	
	double getOutputByAdd(double curerr);//增量式PID控制，计算简单，过渡效果更好
};

#endif



