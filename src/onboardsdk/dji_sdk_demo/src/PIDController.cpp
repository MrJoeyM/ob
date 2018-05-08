#include"PIDController.h"
using namespace std;

//位置式PID
//single contral
void PIDController::setParam(double kP, double kI, double kD, double kN){
    _kP = kP;
    _kI = kI;
    _kD = kD;
    _kN = kN;
	
}
void PIDController::reset(){
    prev_err = 0;//上上次误差
    last_err = 0;//上次误差
    cur_err  = 0;//当前误差
	lastPIDOut=0;//上一次输出结果
    old_D    = 0;
    cur_I    = 0;
	_bFirstFrame = false;
}
//dt积分步长
double PIDController::getOutputByPos(double curerr)
{
    last_err = cur_err;
    cur_err = curerr;
    ///////////////////////////////
    for (int i =0; i<10; ++i) errs[i+1]=errs[i];
    errs[0]=curerr;
    ///////////////////////////

    double s = 0;
    if( !_bFirstFrame){
        //assert(dt > 0);//dt积分步长，判断dt是否大于0
        cur_I += cur_err;
        _P = cur_err;
        _I = cur_I;
        double Derr = cur_err - last_err;
        _D = (_kN*Derr + old_D)/(_kN + 1);
        old_D = _D;
    }else{
        _P = cur_err;
        _I = 0;
        _D = 0;
        _bFirstFrame = false;
    }
    //////////////////////////////
    double aveerr = (errs[0]+errs[1])/2;
    double pasterr =0;
    for (int i=1; i<=3; ++i) pasterr+=errs[i];
    pasterr/=3;
    _D = aveerr - pasterr;
    cout << "        _P" << _P << " _D" << _D; //<< " _I" << _I
    return _kP*_P + _kI*_I + _kD*_D;
}

//增量式PID控制
double PIDController::getOutputByAdd(double curerr)
{
    prev_err = last_err;
	last_err = cur_err;
	cur_err  = curerr;
	_P = _kP *(cur_err-last_err);
	_I = _kI * cur_err;
	_D = _kD *(cur_err- 2*last_err + prev_err);
    double pwm_value = _P + _I + _D;//增量结果
	
	if(pwm_value > 20)
	   pwm_value = 10;
	if(pwm_value < -20)
       pwm_value = -10;
	
	double PIDOut = lastPIDOut + pwm_value;
	lastPIDOut = PIDOut;
	if(PIDOut > 20)
	   PIDOut = 20;
	if(PIDOut < -20)
       PIDOut = -20;
	return PIDOut;     
}

