/// @file    AC_PNew.cpp
/// @brief   Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PNew.h"

const AP_Param::GroupInfo AC_PNew::var_info[] = {
    // @Param: P
    // @DisplayName: PI Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P", 0, AC_PNew, _kp, default_kp),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 1, AC_PNew, _kd, default_kd),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I", 2, AC_PNew, _ki, default_ki),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: IMAX Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 4, AC_PNew, _kimax, default_kimax),

    AP_GROUPEND
};

AC_PNew::AC_PNew(float initial_p, float initial_d, float initial_i, float initial_imax) :
    default_kp(initial_p),
    default_kd(initial_d),
    default_ki(initial_i),
    default_kimax(initial_imax),
    _integrator(0.0f),
    _last_error(0.0f)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp.set(initial_p);
    _kd.set(initial_d);
    _ki.set(initial_i);
    _kimax.set(initial_imax);
}

float AC_PNew::update_all(float target, float measurement, float dt, bool limit) {
    float error = target - measurement;
    float P_out = compute_p(error);
    float D_out = compute_d(error, dt);
    float I_out = compute_i(error, dt);
    return P_out + I_out + D_out;
}

float AC_PNew::compute_p(float error) {
    return _kp.get() * error;
}

float AC_PNew::compute_d(float error, float dt) {
    float derivative = (error - _last_error) / dt;
    _last_error = error;
    return _kd.get() * derivative;
}

float AC_PNew::compute_i(float error, float dt) {
    _integrator += error * dt;
    _integrator = constrain_float(_integrator, -_kimax.get(), _kimax.get());
    return _ki.get() * _integrator;
}

void AC_PNew::reset_I() {
    _integrator = 0.0f;
    _last_error = 0.0f;
}

void AC_PNew::load_gains() {
    _kp.load();
    _kd.load();
    _ki.load();
    _kimax.load();
}

void AC_PNew::save_gains() {
    _kp.save();
    _kd.save();
    _ki.save();
    _kimax.save();
}

void AC_PNew::operator()(float p_val, float d_val, float i_val, float imax_val) {
    _kp.set(p_val);
    _kd.set(d_val);
    _ki.set(i_val);
    _kimax.set(imax_val);
}

void AC_PNew::set_integrator(float i) {
    _flags._I_set = true;
    _integrator = constrain_float(i, -_kimax.get(), _kimax.get());
}

void AC_PNew::relax_integrator(float integrator, float dt, float time_constant) {
    integrator = constrain_float(integrator, -_kimax.get(), _kimax.get());
    if (is_positive(dt)) {
        _flags._I_set = true;
        _integrator = _integrator + (integrator - _integrator) * (dt / (dt + time_constant));
    }
}
