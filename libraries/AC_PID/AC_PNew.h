#pragma once

/// @file	AC_PD.h
/// @brief	Generic P controller with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include "AP_PIDInfo.h"

/// @class	AC_PNew
/// @brief	Object managing one P controller

#define AC_PID_RESET_TC          0.16f   // Time constant for integrator reset decay to zero

class AC_PNew {
public:
    struct Defaults {
        float p;
        float d;
        float i;
        float imax;
    };
    /// Constructor for P that saves its settings to EEPROM
    ///
    /// @note	PIs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_d       Initial value for the d term.
    /// @param  initial_i       Initial value for the i term.
    /// @param  initial_imax       Initial value for the imax term.
    ///
    AC_PNew(float initial_p, float initial_d, float initial_i, float initial_imax) ;
    AC_PNew(const AC_PNew::Defaults &defaults) :
        AC_PNew(
            defaults.p,
            defaults.d,
            defaults.i,
            defaults.imax
    )
    {}
    /*{
        AP_Param::setup_object_defaults(this, var_info);
    }*/
    CLASS_NO_COPY(AC_PNew);
        
    float update_all(float target, float measurement, float dt, bool limit = false);

    float get_p() const;
    float get_d() const;
    float get_i() const;
    
    /// Iterate the P controller, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    ///
    /// @returns		The updated control output.
    ///
    float update_error(float error, float dt,bool limit = false);
    
    /*float get_p(float error, float dt) const;
    float get_d(float error, float dt) const;
    float get_i(float error, float dt) const;
    float get_imax(float error, float dt);*/
    //float       get_p(float error) const;

  // reset_I - reset the integrator
    void reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void reset_filter() {
        _flags._reset_filter = true;
    }


    /// Load gain properties
    ///
    void        load_gains();

    /// Save gain properties
    ///
    void        save_gains();

    /// operator function call for easy initialisation
    void operator()(float p_val, float d_val, float i_val, float imax_val);


   /* /// @name	parameter accessors*/
  /*  //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator() (const float p) { _kp.set(p); }*/

    // accessors
    AP_Float    &kP() { return _kp; }
    const AP_Float &kP() const { return _kp; }
    AP_Float &kD() { return _kd; }
    AP_Float &kI() { return _ki; }
    AP_Float &kIMAX() { return _kimax; }

    float imax() const { return _kimax.get(); }

    void kP(const float v) { _kp.set(v); }
    void kD(const float v) { _kd.set(v); }
    void kI(const float v) { _ki.set(v); }
    void imax(const float v) { _kimax.set(fabsf(v)); }

    // set the desired and actual rates (for logging purposes)
    void set_target_angle(float target) { _pid_info.target = target; }
    void set_actual_angle(float actual) { _pid_info.actual = actual; }

    // integrator setting functions
    void set_integrator(float i);
    void relax_integrator(float integrator, float dt, float time_constant);

    const AP_PIDInfo& get_pid_info(void) const {return _pid_info;}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    void update_i(float dt, bool limit);

    AP_Float _kp;
    AP_Float _kd;
    AP_Float _ki;
    AP_Float _kimax;

    // flags
    struct ac_pnew_flags {
        bool _reset_filter :1; // true when input filter should be reset during next call to set_input
        bool _I_set :1; // true if if the I terms has been set externally including zeroing
    } _flags;
    
    float _integrator;        // integrator value
    float _target;            // target value to enable filtering
    float _error;             // error value to enable filtering
    float _derivative;        // derivative value to enable filtering

    AP_PIDInfo _pid_info;
private:

    const float default_kp;
    const float default_kd;
    const float default_ki;
    const float default_kimax;
    float _last_error;

};