/// @file	AC_PNew.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PNew.h"

const AP_Param::GroupInfo AC_PNew ::var_info[] = {
    // @Param: P
    // @DisplayName: PI Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_PNew, _kp, default_kp),
   
    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 1, AC_PNew, _kd, default_kd),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I", 2, AC_PNew, _ki, default_ki),

    // 3 was for uint16 IMAX

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: IMAX Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 4, AC_PNew, _kimax, default_kimax),

    AP_GROUPEND
};
AC_PNew::AC_PNew(float initial_p,float initial_d, float initial_i, float initial_imax) :
    default_kp(initial_p),
    default_kd(initial_d),
    default_ki(initial_i),
    default_kimax(initial_imax)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    memset(&_pid_info, 0, sizeof(_pid_info));

}
float AC_PNew::update_all(float target, float measurement, float dt, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }
    float error_last = _error;
    //float error = _error;
    _error = target - measurement;
    //float target_last = _target;
    //float error = _target - measurement;
    float derivative = (_error - error_last) / dt;


    float P_out = (_error * _kp);
    float D_out = (derivative * _kd);

    _pid_info.target = target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + D_out+ _integrator;
}
float AC_PNew::update_error(float error, float dt, bool limit)
{
      // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }
    _target = 0.0;
    const float output = update_all(0.0, -error, dt, limit);

    // Make sure logged target and actual are still 0 to maintain behaviour
    _pid_info.target = 0.0;
    _pid_info.actual = 0.0;

    return output;
}
//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
/*float AC_PNew::get_p(float error) const
{
    return (float)error * _kp;
}*/
void AC_PNew::update_i(float dt, bool limit)
{
    if (!is_zero(_ki) && is_positive(dt)) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
    _pid_info.I = _integrator;
    _pid_info.limit = limit;

    // Set I set flag for logging and clear
    _pid_info.I_term_set = _flags._I_set;
    _flags._I_set = false;
}

float AC_PNew::get_p() const
{
    return _pid_info.P;
}

float AC_PNew::get_d() const
{
    return _pid_info.D;
}

float AC_PNew::get_i() const
{
    return _integrator;
}
void AC_PNew::reset_I()
{
    _flags._I_set = true;
    _integrator = 0.0;
}

void AC_PNew::load_gains()
{
    _kp.load();
    _kd.load();
    _ki.load();
    _kimax.load();
}

void AC_PNew::save_gains()
{
    _kp.save();
    _kd.save();
    _ki.save();
    _kimax.save();
}
void AC_PNew::operator()(float p_val, float d_val, float i_val, float imax_val)
{
    _kp.set(p_val);
    _kd.set(d_val);
    _ki.set(i_val);
    _kimax.set(imax_val);
}
void AC_PNew::set_integrator(float integrator)
{
    _flags._I_set = true;
    _integrator = constrain_float(integrator, -_kimax, _kimax);
}

void AC_PNew::relax_integrator(float integrator, float dt, float time_constant)
{
    integrator = constrain_float(integrator, -_kimax, _kimax);
    if (is_positive(dt)) {
        _flags._I_set = true;
        _integrator = _integrator + (integrator - _integrator) * (dt / (dt + time_constant));
    }
}