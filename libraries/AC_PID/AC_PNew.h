#pragma once

/// @file    AC_PNew.h
/// @brief   Generic PID controller with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include "AP_PIDInfo.h"

/// @class  AC_PNew
/// @brief  Object managing one PID controller

#define AC_PID_RESET_TC          0.16f   // Time constant for integrator reset decay to zero

class AC_PNew {
public:
    struct Defaults {
        float p;
        float d;
        float i;
        float imax;
    };

    /// Constructor for PID that saves its settings to EEPROM
    ///
    /// @note PIDs must be named to avoid either multiple parameters with the
    ///       same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_d       Initial value for the D term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_imax    Initial value for the IMAX term.
    ///
    AC_PNew(float initial_p, float initial_d, float initial_i, float initial_imax);
    AC_PNew(const AC_PNew::Defaults &defaults) :
        AC_PNew(
            defaults.p,
            defaults.d,
            defaults.i,
            defaults.imax
        )
    {}

    CLASS_NO_COPY(AC_PNew);

    float update_all(float target, float measurement, float dt, bool limit = false);

    float compute_p(float error);
    float compute_d(float error, float dt);
    float compute_i(float error, float dt);
    
    /// Iterate the PID controller, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error    The measured error value
    /// @param dt       The time delta in seconds
    ///
    /// @returns        The updated control output.
    ///
    float update_error(float error, float dt, bool limit = false);

    /// Reset the integrator
    void reset_I();

    /// Reset the input filter
    void reset_filter() {
        _flags._reset_filter = true;
    }

    /// Load gain properties
    void load_gains();

    /// Save gain properties
    void save_gains();

    /// Operator function call for easy initialization
    void operator()(float p_val, float d_val, float i_val, float imax_val);

    /// Accessors
    AP_Float& kP() { return _kp; }
    const AP_Float& kP() const { return _kp; }
    AP_Float& kD() { return _kd; }
    const AP_Float& kD() const { return _kd; }
    AP_Float& kI() { return _ki; }
    const AP_Float& kI() const { return _ki; }
    AP_Float& kIMAX() { return _kimax; }
    const AP_Float& kIMAX() const { return _kimax; }

    float imax() const { return _kimax.get(); }

    void kP(const float v) { _kp.set(v); }
    void kD(const float v) { _kd.set(v); }
    void kI(const float v) { _ki.set(v); }
    void imax(const float v) { _kimax.set(fabsf(v)); }

    // Set the desired and actual rates (for logging purposes)
    void set_target_angle(float target) { _pid_info.target = target; }
    void set_actual_angle(float actual) { _pid_info.actual = actual; }

    // Integrator setting functions
    void set_integrator(float i);
    void relax_integrator(float integrator, float dt, float time_constant);

    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    void update_i(float dt, bool limit);

    AP_Float _kp;
    AP_Float _kd;
    AP_Float _ki;
    AP_Float _kimax;

    // Flags
    struct ac_pnew_flags {
        bool _reset_filter :1; // True when input filter should be reset during next call to set_input
        bool _I_set :1; // True if the I term has been set externally including zeroing
    } _flags;

    float _integrator;  // Integrator value
    float _target;      // Target value to enable filtering
    float _error;       // Error value to enable filtering
    float _derivative;  // Derivative value to enable filtering

    AP_PIDInfo _pid_info;

private:
    const float default_kp;
    const float default_kd;
    const float default_ki;
    const float default_kimax;
    float _last_error;
};