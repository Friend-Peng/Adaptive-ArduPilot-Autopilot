#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <cmath>

class AP_YawController {
public:	
    AP_YawController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms)
        : aparm(parms)
        , _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _pid_info.target = 0;
        _pid_info.FF = 0;
        _pid_info.P = 0;
    }

    /* Do not allow copies */
    AP_YawController(const AP_YawController &other) = delete;
    AP_YawController &operator=(const AP_YawController&) = delete;

	int32_t get_servo_out(float scaler, bool disable_integrator);

	void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I() {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }

	float saturation(float x);
	float sign(float x);
	float _update_yaw_adaptive_robust_rule(float pid_sum, float error, float error_dot, float delta_time);
	
	const AP_Logger::PID_Info& get_pid_info(void) const {return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_Vehicle::FixedWing &aparm;
	AP_Float _K_A;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _K_FF;
    AP_Int16 _imax;
	uint32_t _last_t;
	float _last_out;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _K_D_last;

	float _asmc_alfa= 0.001;
	float _sat_eps=0.001;
	float _eta = 1;
	
	// yaw ASMC controller integraor parameter, the upper saturation limit can be tuned by _satYaw, the lower limit is 0
	float _intK0Yaw = 0.00001;
	float _intK1Yaw = 0.00001;
	float _intK2Yaw = 0.00001;

	float _integrator;

	AP_Logger::PID_Info _pid_info;

	AP_AHRS &_ahrs;
};
