#include "cpid.hpp"

CPid::CPid(){
    prevError = 0;
    prevPrevError = 0;
    dt = 0.02;
}

void CPid::set_constant(float _kp, float _ki, float _kd){
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

float CPid::calc_D_value(float _error){
    float tmp = ((_error - prevError)/dt)*kd;
    prevError = _error;
    return tmp;
}

float CPid::calc_I_value(float _error){
    float errorSum = _error + prevError + prevPrevError;
    prevError = _error;
    prevPrevError =prevError;
    float tmp = errorSum*dt*ki;
    return tmp;
}

float CPid::calc_P_value(float _error){
    float tmp = _error * kp;
    return tmp;
}

float CPid::calc_error(float _target_val, float _current_val){
    float tmp = _target_val - _current_val;
    return tmp;
}

float CPid::get_PID_value(float _target_val, float _current_val){
    float _error = calc_error(_target_val, _current_val);
    float _p = calc_P_value(_error);
    float _i = calc_I_value(_error);
    float _d = calc_D_value(_error);

    float tmp = _p + _i + _d;
    return tmp;
}


