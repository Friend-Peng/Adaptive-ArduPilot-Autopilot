/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Initial Code by Jon Challinger
//  Modified by Paul Riseborough

#include <AP_HAL/AP_HAL.h>
#include "AP_PitchController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] = {

	// @Param: TCONST
	// @DisplayName: Pitch Time Constant
	// @Description: Time constant in seconds from demanded to achieved pitch angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
	// @Range: 0.4 1.0
	// @Units: s
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_PitchController, gains.tau,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from pitch angle demands to elevator. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0.1 3.0
	// @Increment: 0.1
	// @User: Standard
	AP_GROUPINFO("P",        1, AP_PitchController, gains.P,          1.0f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from pitch acceleration to elevator. Higher values reduce pitching in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.2
	// @Increment: 0.01
	// @User: Standard
    AP_GROUPINFO("D",        2, AP_PitchController, gains.D,        0.04f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term pitch angle offsets to elevator. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.5
	// @Increment: 0.05
	// @User: Standard
	AP_GROUPINFO("I",        3, AP_PitchController, gains.I,        0.3f),

	// @Param: RMAX_UP
	// @DisplayName: Pitch up max rate
	// @Description: Maximum pitch up rate that the pitch controller demands (degrees/sec) in ACRO mode.
	// @Range: 0 100
	// @Units: deg/s
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX_UP",     4, AP_PitchController, gains.rmax,   0.0f),

	// @Param: RMAX_DN
	// @DisplayName: Pitch down max rate
	// @Description: This sets the maximum nose down pitch rate that the controller will demand (degrees/sec). Setting it to zero disables the limit.
	// @Range: 0 100
	// @Units: deg/s
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX_DN",     5, AP_PitchController, _max_rate_neg,   0.0f),

	// @Param: RLL
	// @DisplayName: Roll compensation
	// @Description: Gain added to pitch to keep aircraft from descending or ascending in turns. Increase in increments of 0.05 to reduce altitude loss. Decrease for altitude gain.
	// @Range: 0.7 1.5
	// @Increment: 0.05
	// @User: Standard
	AP_GROUPINFO("RLL",      6, AP_PitchController, _roll_ff,        1.0f),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of pitch integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 3000 allows trim of up to 2/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      7, AP_PitchController, gains.imax,     3000),

	// @Param: FF
	// @DisplayName: Feed forward Gain
	// @Description: Gain from demanded rate to elevator output.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: Standard
	AP_GROUPINFO("FF",        8, AP_PitchController, gains.FF,       0.0f),

    // @Param: SRMAX
    // @DisplayName: Servo slew rate limit
    // @Description: Sets an upper limit on the servo slew rate produced by the D-gain (pitch rate feedback). If the amplitude of the control action produced by the pitch rate feedback exceeds this value, then the D-gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive D-gain. The limit should be set to no more than 25% of the servo's specified slew rate to allow for inertia and aerodynamic load effects. Note: The D-gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Units: deg/s
    // @Range: 0 500
    // @Increment: 10.0
    // @User: Advanced
    AP_GROUPINFO("SRMAX", 9, AP_PitchController, _slew_rate_max, 150.0f),

    // @Param: SRTAU
    // @DisplayName: Servo slew rate decay time constant
    // @Description: This sets the time constant used to recover the D gain after it has been reduced due to excessive servo slew rate.
    // @Units: s
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SRTAU", 10, AP_PitchController, _slew_rate_tau, 1.0f),

    AP_GROUPEND
};



/* calculate sign function
 */
float AP_PitchController::sign(float x)
{
	if (x > 0.0000001f) {
		return 1.0f;
	} else if (x < -0.0000001f) {
		return -1.0f;
	} else {
		return 0.0f;
	}
	return 0.0f;
};

/* calculate saturation value to avoid chattering
 */
float AP_PitchController::saturation(float x)
{
	if (x > 1.0f) {
		return 1.0f;
	} else if (x < -1.0f) {
		return -1.0f;
	} else {
		return x;
	}
	return x;
};


/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded pitch rate in degrees/second
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
 5) maximum FBW airspeed (metres/sec)
*/
int32_t AP_PitchController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	float delta_time    = (float)dt * 0.001f;
	
	// Get body rate vector (radians/sec)
	float omega_y = _ahrs.get_gyro().y;
	
	// Calculate the pitch rate error (deg/sec) and scale
    float achieved_rate = ToDeg(omega_y);
    _pid_info.error = desired_rate - achieved_rate;
    float rate_error = _pid_info.error * scaler;
    _pid_info.target = desired_rate;
    _pid_info.actual = achieved_rate;

	uint32_t _switch = 2;  //  0  original,     1  adaptive    2  finite-time
	
	// Multiply pitch rate error by _ki_rate and integrate
	// Scaler is applied before integrator so that integrator state relates directly to elevator deflection
	// This means elevator trim offset doesn't change as the value of scaler changes with airspeed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (!disable_integrator && gains.I > 0) {
        float k_I = gains.I;
        if (is_zero(gains.FF)) {
            /*
              if the user hasn't set a direct FF then assume they are
              not doing sophisticated tuning. Set a minimum I value of
              0.15 to ensure that the time constant for trimming in
              pitch is not too long. We have had a lot of user issues
              with very small I value leading to very slow pitch
              trimming, which causes a lot of problems for TECS. A
              value of 0.15 is still quite small, but a lot better
              than what many users are running.
             */
            k_I = MAX(k_I, 0.15f);
        }
        float ki_rate = k_I * gains.tau;
		//only integrate if gain and time step are positive and airspeed above min value.
		if (dt > 0 && aspeed > 0.5f*float(aparm.airspeed_min)) {
		    float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			if (_last_out < -45) {
				// prevent the integrator from increasing if surface defln demand is above the upper limit
				integrator_delta = MAX(integrator_delta , 0);
			} else if (_last_out > 45) {
				// prevent the integrator from decreasing if surface defln demand  is below the lower limit
				integrator_delta = MIN(integrator_delta , 0);
			}
			_pid_info.I += integrator_delta;
		}
	} else {
		_pid_info.I = 0;
	}

    // Scale the integration limit
    float intLimScaled = gains.imax * 0.01f;

    // Constrain the integrator state
    _pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);

	// Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
    float eas2tas = _ahrs.get_EAS2TAS();
	float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D , 0) / eas2tas;
    float k_ff = gains.FF / eas2tas;

	// Calculate the demanded control surface deflection
	// Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	// path, but want a 1/speed^2 scaler applied to the rate error path. 
	// This is because acceleration scales with speed^2, but rate scales with speed.
    _pid_info.P = desired_rate * kp_ff * scaler;
    _pid_info.FF = desired_rate * k_ff * scaler;
    _pid_info.D = rate_error * gains.D * scaler;

    if (dt > 0 && _slew_rate_max > 0) {
        // Calculate the slew rate amplitude produced by the unmodified D term

        // calculate a low pass filtered slew rate
        float Dterm_slew_rate = _slew_rate_filter.apply((fabsf(_pid_info.D - _last_pid_info_D)/ delta_time), delta_time);

        // rectify and apply a decaying envelope filter
        float alpha = 1.0f - constrain_float(delta_time/_slew_rate_tau, 0.0f , 1.0f);
        _slew_rate_amplitude = fmaxf(fabsf(Dterm_slew_rate), alpha * _slew_rate_amplitude);
        _slew_rate_amplitude = fminf(_slew_rate_amplitude, 10.0f*_slew_rate_max);

        // Calculate and apply the D gain adjustment
        _pid_info.Dmod = _D_gain_modifier = _slew_rate_max / fmaxf(_slew_rate_amplitude, _slew_rate_max);
        _pid_info.D *= _D_gain_modifier;
    }

    _last_pid_info_D = _pid_info.D;

    _last_out = _pid_info.D + _pid_info.FF + _pid_info.P;


    if (autotune.running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values 
        // Note that we don't pass the integrator component so we get
        // a better idea of how much the base PD controller
        // contributed
        autotune.update(desired_rate, achieved_rate, _last_out);
        
        // set down rate to rate up when auto-tuning
        _max_rate_neg.set_and_save_ifchanged(gains.rmax);
    }

	_last_out += _pid_info.I;
	
    // Add adaptive-robust rule term, and airspeed above min value
	if (_switch == 1 && aspeed > 1.8 * float(aparm.airspeed_min)) 
	{
		float adaptive_robust_value = _update_pitch_adaptive_robust_rule(_last_out,  _pid_info.P, _pid_info.D, _pid_info.I, delta_time);
		if (_last_out < -45)
		{
			adaptive_robust_value = MAX(adaptive_robust_value , 0);
		}
		else if (_last_out > 45)
		{
			adaptive_robust_value = MIN(adaptive_robust_value, 0);
		}
	    _last_out += adaptive_robust_value;
	}
    
    if (_switch == 2  && aspeed > 1.8 * aparm.airspeed_min)
	{
		float adpative_finite_time_value = _update_pitch_finite_time_adaptive_rule(_last_out, _pid_info.P, _pid_info.D, _pid_info.I, delta_time);
		if (_last_out < -45)
		{
			adpative_finite_time_value = MAX(adpative_finite_time_value , 0);
		}
		else if (_last_out > 45)
		{
			adpative_finite_time_value = MIN(adpative_finite_time_value, 0);
		}
		_last_out += adpative_finite_time_value;
	}
	

    /*
      when we are past the users defined roll limit for the
      aircraft our priority should be to bring the aircraft back
      within the roll limit. Using elevator for pitch control at
      large roll angles is ineffective, and can be counter
      productive as it induces earth-frame yaw which can reduce
      the ability to roll. We linearly reduce elevator input when
      beyond the configured roll limit, reducing to zero at 90
      degrees
    */
    float roll_wrapped = labs(_ahrs.roll_sensor);
    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }
    if (roll_wrapped > aparm.roll_limit_cd + 500 && aparm.roll_limit_cd < 8500 &&
        labs(_ahrs.pitch_sensor) < 7000) {
        float roll_prop = (roll_wrapped - (aparm.roll_limit_cd+500)) / (float)(9000 - aparm.roll_limit_cd);
        _last_out *= (1 - roll_prop);
    }
    
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}

/*
  Get pitch finite-time adaptive rule term
 */
float AP_PitchController::_update_pitch_finite_time_adaptive_rule(float pid_sum, float error, float error_dot, float error_int, float delta_time)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i^v, i = 0,1,2;
	// xi = [error, error_dot, error_int];

	float error_delta = 0;
	float alpha1 = 0;
    float alpha2 = 0;
	if (fabs(error)>_varepsPitch)
	{
		error_delta = pow(fabs(error), _gammaPitch)*sign(error);
	}
	else
	{
		alpha1 = (2-_gammaPitch)* pow(_varepsPitch, _gammaPitch-1);
	    alpha2 = (_gammaPitch-1)* pow(_varepsPitch, _gammaPitch-2);
		error_delta = alpha1*error + alpha2*sign(error)*pow(error, 2);
	}

	
	float s       = pid_sum + _lambda3Pitch*error_delta; // s = error +  error_dot +  error_int + _lambda3Pitch*error_delta;
	float norm_xi = sqrt((fabs(error)+pow(fabs(error), _gammaPitch))*(fabs(error)+pow(fabs(error), _gammaPitch)) + error_dot*error_dot + error_int*error_int);
	float norm_s  = fabs(s);
	
	// Calculate sign(s), but avoid chattering
	float sat_s = saturation(s / _upsilonPitch);
 	float rho = _intK0Pitch + _intK1Pitch * norm_xi + _intK2Pitch * norm_xi*norm_xi;

	if (delta_time > 0) {
		float intK0_delta = (norm_s - _betaPitch * pow(_intK0Pitch,_vPitch)) * delta_time;
		float intK1_delta = (norm_s * norm_xi - _betaPitch * pow(_intK1Pitch,_vPitch)) * delta_time;
		float intK2_delta = (norm_s * norm_xi*norm_xi - _betaPitch * pow(_intK2Pitch,_vPitch)) * delta_time;
		// prevent the integrator from increasing if surface defln demand is above the upper limit
		if (_last_out < -45 || _last_out > 45) {
		float intK0_delta_temp = - _betaPitch * pow(_intK0Pitch,_vPitch) * delta_time;  //keep nagative     ruguo  lianggezhi xiangjia zhihou, meiyou fashengbaohe,  name jiubuyong decrease
		float intK1_delta_temp = - _betaPitch * pow(_intK1Pitch,_vPitch) * delta_time;
		float intK2_delta_temp = - _betaPitch * pow(_intK2Pitch,_vPitch) * delta_time;
		float _intK0Pitch_temp = _intK0Pitch; _intK0Pitch_temp += intK0_delta_temp;
		float _intK1Pitch_temp = _intK1Pitch; _intK1Pitch_temp += intK1_delta_temp;
		float _intK2Pitch_temp = _intK2Pitch; _intK2Pitch_temp += intK2_delta_temp;
		float rho_temp = _intK0Pitch_temp + _intK1Pitch_temp * norm_xi + _intK2Pitch_temp * norm_xi*norm_xi;
		float _last_out_temp = _lambda3Pitch*error_delta +  rho_temp * sat_s + _sigmaPitch*pow(fabs(s),_vPitch)*sign(s);;   // printf("pitch 111111111111\n");
		if (_last_out_temp < -45 || _last_out_temp > 45)
		{
			intK0_delta  = intK0_delta_temp;
			intK1_delta  = intK1_delta_temp;
			intK2_delta  = intK2_delta_temp;
		}
	}  
	
		_intK0Pitch += intK0_delta;
		_intK1Pitch += intK1_delta;
		_intK2Pitch += intK2_delta;
		if (_intK0Pitch < 0) {
		_intK0Pitch = 0;
		}
		if (_intK1Pitch < 0) {
		_intK1Pitch = 0;
		}
		if (_intK2Pitch < 0) {
		_intK2Pitch = 0;
		}
	}

	return _lambda3Pitch*error_delta + rho*sat_s + _sigmaPitch*pow(fabs(s),_vPitch)*sign(s);
}

/*
  Get pitch adaptive robust rule term
 */
float AP_PitchController::_update_pitch_adaptive_robust_rule(float pid_sum, float error, float error_dot, float error_int, float delta_time)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i, i = 0,1,2;
	// xi = [error, error_dot, error_int];
	
	float s       = pid_sum;
	float norm_xi = sqrt(error*error + error_dot*error_dot +  error_int*error_int);
	float norm_s  = fabs(s);
	
	// Calculate sign(s), but avoid chattering
	float sign_s = saturation(s / _sat_eps);
	float rho = _intK0Pitch + _intK1Pitch * norm_xi + _intK2Pitch * norm_xi*norm_xi;

    // Calculate rho
	float intK0_delta = (norm_s - _asmc_alfa * _intK0Pitch) * delta_time;
	float intK1_delta = (norm_s * norm_xi - _asmc_alfa * _intK1Pitch) * delta_time;
	float intK2_delta = (norm_s * norm_xi*norm_xi - _asmc_alfa * _intK2Pitch) * delta_time;
	if (_last_out < -45 || _last_out > 45) {
		float intK0_delta_temp = ( - _asmc_alfa * _intK0Pitch) * delta_time;  //keep nagative     ruguo  lianggezhi xiangjia zhihou, meiyou fashengbaohe,  name jiubuyong decrease
		float intK1_delta_temp = ( - _asmc_alfa * _intK1Pitch) * delta_time;
		float intK2_delta_temp = (  - _asmc_alfa * _intK2Pitch) * delta_time;
		float _intK0Pitch_temp = _intK0Pitch; _intK0Pitch_temp += intK0_delta_temp;
		float _intK1Pitch_temp = _intK1Pitch; _intK1Pitch_temp += intK1_delta_temp;
		float _intK2Pitch_temp = _intK2Pitch; _intK2Pitch_temp += intK2_delta_temp;
		float rho_temp = _intK0Pitch_temp + _intK1Pitch_temp * norm_xi + _intK2Pitch_temp * norm_xi*norm_xi;
		float _last_out_temp = rho_temp * sign_s;   // printf("pitch 111111111111\n");
		if (_last_out_temp < -45 || _last_out_temp > 45)
		{
			intK0_delta  = intK0_delta_temp;
			intK1_delta  = intK1_delta_temp;
			intK2_delta  = intK2_delta_temp;
		}
	}  
	
	_intK0Pitch += intK0_delta;
	_intK1Pitch += intK1_delta;
	_intK2Pitch += intK2_delta;
	if (_intK0Pitch < 0) {
	_intK0Pitch = 0;
	}
	if (_intK1Pitch < 0) {
	_intK1Pitch = 0;
	}
	if (_intK2Pitch < 0) {
	_intK2Pitch = 0;
	}
	
	return rho*sign_s;
}


/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded pitch rate in degrees/second
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
 5) maximum FBW airspeed (metres/sec)
*/
int32_t AP_PitchController::get_rate_out(float desired_rate, float scaler)
{
    float aspeed;
	if (!_ahrs.airspeed_estimate(aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
	}
    return _get_rate_out(desired_rate, scaler, false, aspeed);
}

/*
  get the rate offset in degrees/second needed for pitch in body frame
  to maintain height in a coordinated turn.
  
  Also returns the inverted flag and the estimated airspeed in m/s for
  use by the rest of the pitch controller
 */
float AP_PitchController::_get_coordination_rate_offset(float &aspeed, bool &inverted) const
{
	float rate_offset;
	float bank_angle = _ahrs.roll;

	// limit bank angle between +- 80 deg if right way up
	if (fabsf(bank_angle) < radians(90))	{
	    bank_angle = constrain_float(bank_angle,-radians(80),radians(80));
        inverted = false;
	} else {
		inverted = true;
		if (bank_angle > 0.0f) {
			bank_angle = constrain_float(bank_angle,radians(100),radians(180));
		} else {
			bank_angle = constrain_float(bank_angle,-radians(180),-radians(100));
		}
	}
	if (!_ahrs.airspeed_estimate(aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
	}
    if (abs(_ahrs.pitch_sensor) > 7000) {
        // don't do turn coordination handling when at very high pitch angles
        rate_offset = 0;
    } else {
        rate_offset = cosf(_ahrs.pitch)*fabsf(ToDeg((GRAVITY_MSS / MAX((aspeed * _ahrs.get_EAS2TAS()) , MAX(aparm.airspeed_min, 1))) * tanf(bank_angle) * sinf(bank_angle))) * _roll_ff;
    }
	if (inverted) {
		rate_offset = -rate_offset;
	}
    return rate_offset;
}

// Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
// A positive demand is up
// Inputs are: 
// 1) demanded pitch angle in centi-degrees
// 2) control gain scaler = scaling_speed / aspeed
// 3) boolean which is true when stabilise mode is active
// 4) minimum FBW airspeed (metres/sec)
// 5) maximum FBW airspeed (metres/sec)
//
int32_t AP_PitchController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
	// Calculate offset to pitch rate demand required to maintain pitch angle whilst banking
	// Calculate ideal turn rate from bank angle and airspeed assuming a level coordinated turn
	// Pitch rate offset is the component of turn rate about the pitch axis
	float aspeed;
	float rate_offset;
	bool inverted;

    if (gains.tau < 0.1f) {
        gains.tau.set(0.1f);
    }

    rate_offset = _get_coordination_rate_offset(aspeed, inverted);
	
	// Calculate the desired pitch rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / gains.tau;
	
	// limit the maximum pitch rate demand. Don't apply when inverted
	// as the rates will be tuned when upright, and it is common that
	// much higher rates are needed inverted	
	if (!inverted) {
		if (_max_rate_neg && desired_rate < -_max_rate_neg) {
			desired_rate = -_max_rate_neg;
		} else if (gains.rmax && desired_rate > gains.rmax) {
			desired_rate = gains.rmax;
		}
	}
	
	if (inverted) {
		desired_rate = -desired_rate;
	}

	// Apply the turn correction offset
	desired_rate = desired_rate + rate_offset;

    return _get_rate_out(desired_rate, scaler, disable_integrator, aspeed);
}

void AP_PitchController::reset_I()
{
	_pid_info.I = 0;
}
