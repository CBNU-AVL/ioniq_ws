#ifndef CPID_H
#define CPID_H


class CPid
{
private:
    float kp;
    float ki;
    float kd;
    float dt;
    float prevError, prevPrevError;
public:
    CPid();
    void set_constant(float _kp, float _ki, float _kd);
    float calc_P_value(float _error);
    float calc_I_value(float _error);
    float calc_D_value(float _error);
    float calc_error(float _target_val, float _current_val);
    float get_PID_value(float _target_val, float _current_val);
};

#endif // CPID_H
