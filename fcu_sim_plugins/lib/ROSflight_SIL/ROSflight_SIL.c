/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University, Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink.h"

uint8_t mavlink_system;
void init_mavlink(void)
{
  mavlink_system = 250;
}


#include "param.h"
#include "mixer.h"

params_t _params;

// local function definitions
static void init_param_int(param_id_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
  _params.values[id] = value;
  _params.types[id] = PARAM_TYPE_INT32;
}

static void init_param_float(param_id_t id, char name[PARAMS_NAME_LENGTH], float value)
{
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
  _params.values[id] = *((int32_t *) &value);
  _params.types[id] = PARAM_TYPE_FLOAT;
}

void init_params(void)
{
  initEEPROM();
  if (!read_params())
  {
    set_param_defaults();
    write_params();
  }

  for (uint16_t id = 0; id < PARAMS_COUNT; id++)
    param_change_callback((param_id_t) id);
}

void set_param_defaults(void)
{
  init_param_int(PARAM_BOARD_REVISION, "BOARD_REV", 5);

  init_param_int(PARAM_BAUD_RATE, "BAUD_RATE", 921600);

  init_param_int(PARAM_SYSTEM_ID, "SYS_ID", 1);
  init_param_int(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1);

  init_param_int(PARAM_STREAM_ATTITUDE_RATE, "STRM_ATTITUDE", 100);
  init_param_int(PARAM_STREAM_IMU_RATE, "STRM_IMU", 500);
  init_param_int(PARAM_STREAM_MAG_RATE, "STRM_MAG", 0);
  init_param_int(PARAM_STREAM_BARO_RATE, "STRM_BARO", 0);
  init_param_int(PARAM_STREAM_AIRSPEED_RATE, "STRM_AIRSPEED", 0);
  init_param_int(PARAM_STREAM_GPS_RATE, "STRM_GPS", 0);
  init_param_int(PARAM_STREAM_SONAR_RATE, "STRM_SONAR", 0);

  init_param_int(PARAM_STREAM_SERVO_OUTPUT_RAW_RATE, "STRM_SERVO", 50);
  init_param_int(PARAM_STREAM_RC_RAW_RATE, "STRM_RC", 50);

  init_param_int(PARAM_DIFF_PRESS_UPDATE, "DIFF_PRESS_UP", 0); // us
  init_param_int(PARAM_BARO_UPDATE, "BARO_UPDATE", 0);
  init_param_int(PARAM_SONAR_UPDATE, "SONAR_UPDATE", 0);
  init_param_int(PARAM_MAG_UPDATE, "MAG_UPDATE", 20000);

  init_param_int(PARAM_INIT_TIME, "FILTER_INIT_T", 3000); // ms
  init_param_float(PARAM_FILTER_KP, "FILTER_KP", 1.0f);
  init_param_float(PARAM_FILTER_KI, "FILTER_KI", 0.1f);
  init_param_float(PARAM_GYRO_ALPHA, "GYRO_LPF_ALPHA", 0.6f);
  init_param_float(PARAM_ACC_ALPHA, "ACC_LPF_ALPHA", 0.6f);
  init_param_int(PARAM_STREAM_ADJUSTED_GYRO, "STRM_ADJUST_GYRO", 0);
  init_param_float(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0.0f);
  init_param_float(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0.0f);
  init_param_float(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0.0f);
  init_param_float(PARAM_ACC_X_BIAS,  "ACC_X_BIAS", 0.0f);
  init_param_float(PARAM_ACC_Y_BIAS,  "ACC_Y_BIAS", 0.0f);
  init_param_float(PARAM_ACC_Z_BIAS,  "ACC_Z_BIAS", 0.0f);
  init_param_float(PARAM_ACC_X_TEMP_COMP,  "ACC_X_TEMP_COMP", 0.0f);
  init_param_float(PARAM_ACC_Y_TEMP_COMP,  "ACC_Y_TEMP_COMP", 0.0f);
  init_param_float(PARAM_ACC_Z_TEMP_COMP,  "ACC_Z_TEMP_COMP", 0.0f);

  init_param_int(PARAM_MOTOR_PWM_SEND_RATE, "MOTOR_PWM_UPDATE", 400);
  init_param_int(PARAM_MOTOR_IDLE_PWM, "MOTOR_IDLE_PWM", 1150);
  init_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED, "ARM_SPIN_MOTORS", true);
  init_param_int(PARAM_RC_TYPE, "RC_TYPE", 1);
  init_param_int(PARAM_RC_X_CHANNEL, "RC_X_CHN", 0);
  init_param_int(PARAM_RC_Y_CHANNEL, "RC_Y_CHN", 1);
  init_param_int(PARAM_RC_Z_CHANNEL, "RC_Z_CHN", 3);
  init_param_int(PARAM_RC_F_CHANNEL, "RC_F_CHN", 2);
  init_param_int(PARAM_RC_NUM_CHANNELS, "RC_NUM_CHN", 6);

  init_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, "RC_ATT_OVRD_CHN", 4);
  init_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, "RC_THR_OVRD_CHN", 4);
  init_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,  "RC_ATT_CTRL_CHN", 5);
  init_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL,    "RC_F_CTRL_CHN", 7);

  init_param_int(PARAM_RC_X_CENTER, "RC_X_CENTER", 1500);
  init_param_int(PARAM_RC_Y_CENTER, "RC_Y_CENTER", 1500);
  init_param_int(PARAM_RC_Z_CENTER, "RC_Z_CENTER", 1500);
  init_param_int(PARAM_RC_F_BOTTOM, "RC_F_BOTTOM", 1000);
  init_param_int(PARAM_RC_X_RANGE,  "RC_X_RANGE", 1000);
  init_param_int(PARAM_RC_Y_RANGE,  "RC_Y_RANGE", 1000);
  init_param_int(PARAM_RC_Z_RANGE,  "RC_Z_RANGE", 1000);
  init_param_int(PARAM_RC_F_RANGE,  "RC_F_RANGE", 1000);
  init_param_int(PARAM_RC_SWITCH_5_DIRECTION, "SWITCH_5_DIR", 1);
  init_param_int(PARAM_RC_SWITCH_6_DIRECTION, "SWITCH_6_DIR", 1);
  init_param_int(PARAM_RC_SWITCH_7_DIRECTION, "SWITCH_7_DIR", 1);
  init_param_int(PARAM_RC_SWITCH_8_DIRECTION, "SWITCH_8_DIR", 1);

  init_param_int(PARAM_RC_OVERRIDE_DEVIATION, "RC_OVRD_DEV", 100);
  init_param_int(PARAM_OVERRIDE_LAG_TIME, "OVRD_LAG_TIME", 1000);
  init_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, "MIN_THROTTLE", false);

  init_param_float(PARAM_RC_MAX_ROLL, "RC_MAX_ROLL", 0.786f);
  init_param_float(PARAM_RC_MAX_PITCH, "RC_MAX_PITCH", 0.786f);
  init_param_float(PARAM_RC_MAX_ROLLRATE, "RC_MAX_ROLLRATE", 3.14159f);
  init_param_float(PARAM_RC_MAX_PITCHRATE, "RC_MAX_PITCHRATE", 3.14159f);
  init_param_float(PARAM_RC_MAX_YAWRATE, "RC_MAX_YAWRATE", 3.14159f);
  init_param_float(PARAM_FAILSAFE_THROTTLE, "FAILSAFE_THR", 0.5);

  init_param_float(PARAM_ROLL_TRIM, "ROLL_TRIM", 0.0f);
  init_param_float(PARAM_PITCH_TRIM, "PITCH_TRIM", 0.0f);
  init_param_float(PARAM_ROLLRATE_TRIM, "ROLLRATE_TRIM", 0.0f);
  init_param_float(PARAM_PITCHRATE_TRIM, "PITCHRATE_TRIM", 0.0f);
  init_param_float(PARAM_YAWRATE_TRIM, "YAWRATE_TRIM", 0.0f);

  init_param_int(PARAM_ARM_STICKS, "ARM_STICKS", true);
  init_param_int(PARAM_ARM_CHANNEL, "ARM_CHANNEL", 5);
  init_param_int(PARAM_ARM_THRESHOLD, "ARM_THRESHOLD", 150);

  init_param_float(PARAM_PID_ALT_P, "PID_ALT_P", 10.0f);
  init_param_float(PARAM_PID_ALT_I, "PID_ALT_I", 0.0f);
  init_param_float(PARAM_PID_ALT_D, "PID_ALT_D", 0.0f);

  init_param_float(PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", 0.15f);
  init_param_float(PARAM_PID_ROLL_ANGLE_I, "PID_ROLL_ANG_I", 0.0f);
  init_param_float(PARAM_PID_ROLL_ANGLE_D, "PID_ROLL_ANG_D", 0.07f);
  init_param_float(PARAM_MAX_ROLL_ANGLE, "MAX_ROLL_ANG", 0.786f);

  init_param_float(PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", 0.15f);
  init_param_float(PARAM_PID_PITCH_ANGLE_I, "PID_PITCH_ANG_I", 0.0f);
  init_param_float(PARAM_PID_PITCH_ANGLE_D, "PID_PITCH_ANG_D", 0.07f);
  init_param_float(PARAM_MAX_PITCH_ANGLE, "MAX_PITCH_ANG", 0.786);

  init_param_float(PARAM_PID_ROLL_RATE_P, "PID_ROLL_RATE_P", 0.070f);
  init_param_float(PARAM_PID_ROLL_RATE_I, "PID_ROLL_RATE_I", 0.00001f);
  init_param_float(PARAM_MAX_ROLL_RATE, "MAX_ROLL_RATE", 3.14159f);

  init_param_float(PARAM_PID_PITCH_RATE_P, "PID_PITCH_RATE_P", 0.070f);
  init_param_float(PARAM_PID_PITCH_RATE_I, "PID_PITCH_RATE_I", 0.00001f);
  init_param_float(PARAM_MAX_PITCH_RATE, "MAX_PITCH_RATE", 3.14159f);

  init_param_float(PARAM_PID_YAW_RATE_P, "PID_YAW_RATE_P", 0.025f);
  init_param_float(PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", 0.0f);
  init_param_float(PARAM_MAX_YAW_RATE, "MAX_YAW_RATE", 6.283f);

  init_param_float(PARAM_PID_TAU, "PID_TAU", 0.05f);

  init_param_int(PARAM_MAX_COMMAND, "PARAM_MAX_CMD", 1000);

  init_param_int(PARAM_MIXER, "MIXER", QUADCOPTER_PLUS);
  init_param_int(PARAM_ELEVATOR_REVERSE, "ELEVATOR_REV", 0);
  init_param_int(PARAM_AILERON_REVERSE, "AIL_REV", 0);
  init_param_int(PARAM_RUDDER_REVERSE, "RUDDER_REV", 0);
  init_param_int(PARAM_FIXED_WING, "FIXED_WING", false);
}


#ifdef __cplusplus
}
#endif
