/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#pragma once

#include <stddef.h>

#include <uORB/uORB.h>

static constexpr size_t ORB_TOPICS_COUNT{168};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	actuator_armed = 0,
	actuator_controls = 1,
	actuator_controls_0 = 2,
	actuator_controls_1 = 3,
	actuator_controls_2 = 4,
	actuator_controls_3 = 5,
	actuator_controls_virtual_fw = 6,
	actuator_controls_virtual_mc = 7,
	actuator_outputs = 8,
	adc_report = 9,
	airspeed = 10,
	airspeed_validated = 11,
	battery_status = 12,
	camera_capture = 13,
	camera_trigger = 14,
	camera_trigger_secondary = 15,
	cellular_status = 16,
	collision_constraints = 17,
	collision_report = 18,
	commander_state = 19,
	cpuload = 20,
	debug_array = 21,
	debug_key_value = 22,
	debug_value = 23,
	debug_vect = 24,
	differential_pressure = 25,
	distance_sensor = 26,
	ekf2_timestamps = 27,
	ekf_gps_drift = 28,
	esc_report = 29,
	esc_status = 30,
	estimator_attitude = 31,
	estimator_global_position = 32,
	estimator_innovation_test_ratios = 33,
	estimator_innovation_variances = 34,
	estimator_innovations = 35,
	estimator_local_position = 36,
	estimator_odometry = 37,
	estimator_optical_flow_vel = 38,
	estimator_selector_status = 39,
	estimator_sensor_bias = 40,
	estimator_states = 41,
	estimator_status = 42,
	estimator_visual_odometry_aligned = 43,
	follow_target = 44,
	fw_virtual_attitude_setpoint = 45,
	generator_status = 46,
	geofence_result = 47,
	gps_dump = 48,
	gps_inject_data = 49,
	home_position = 50,
	hover_thrust_estimate = 51,
	input_rc = 52,
	iridiumsbd_status = 53,
	irlock_report = 54,
	landing_gear = 55,
	landing_target_innovations = 56,
	landing_target_pose = 57,
	led_control = 58,
	log_message = 59,
	logger_status = 60,
	manual_control_setpoint = 61,
	mavlink_log = 62,
	mc_virtual_attitude_setpoint = 63,
	mission = 64,
	mission_result = 65,
	mount_orientation = 66,
	multirotor_motor_limits = 67,
	navigator_mission_item = 68,
	obstacle_distance = 69,
	obstacle_distance_fused = 70,
	offboard_control_mode = 71,
	onboard_computer_status = 72,
	optical_flow = 73,
	orb_multitest = 74,
	orb_test = 75,
	orb_test_large = 76,
	orb_test_medium = 77,
	orb_test_medium_multi = 78,
	orb_test_medium_queue = 79,
	orb_test_medium_queue_poll = 80,
	orb_test_medium_wrap_around = 81,
	orbit_status = 82,
	parameter_update = 83,
	ping = 84,
	position_controller_landing_status = 85,
	position_controller_status = 86,
	position_setpoint = 87,
	position_setpoint_triplet = 88,
	power_button_state = 89,
	power_monitor = 90,
	pwm_input = 91,
	px4io_status = 92,
	qshell_req = 93,
	qshell_retval = 94,
	radio_status = 95,
	rate_ctrl_status = 96,
	rc_channels = 97,
	rc_parameter_map = 98,
	rpm = 99,
	safety = 100,
	satellite_info = 101,
	sensor_accel = 102,
	sensor_accel_fifo = 103,
	sensor_baro = 104,
	sensor_combined = 105,
	sensor_correction = 106,
	sensor_gps = 107,
	sensor_gyro = 108,
	sensor_gyro_fft = 109,
	sensor_gyro_fifo = 110,
	sensor_mag = 111,
	sensor_preflight_mag = 112,
	sensor_selection = 113,
	sensors_status_imu = 114,
	system_power = 115,
	task_stack_info = 116,
	tecs_status = 117,
	telemetry_status = 118,
	test_motor = 119,
	timesync = 120,
	timesync_status = 121,
	trajectory_bezier = 122,
	trajectory_setpoint = 123,
	trajectory_waypoint = 124,
	transponder_report = 125,
	tune_control = 126,
	uavcan_parameter_request = 127,
	uavcan_parameter_value = 128,
	ulog_stream = 129,
	ulog_stream_ack = 130,
	vehicle_acceleration = 131,
	vehicle_air_data = 132,
	vehicle_angular_acceleration = 133,
	vehicle_angular_velocity = 134,
	vehicle_angular_velocity_groundtruth = 135,
	vehicle_attitude = 136,
	vehicle_attitude_groundtruth = 137,
	vehicle_attitude_setpoint = 138,
	vehicle_command = 139,
	vehicle_command_ack = 140,
	vehicle_constraints = 141,
	vehicle_control_mode = 142,
	vehicle_global_position = 143,
	vehicle_global_position_groundtruth = 144,
	vehicle_gps_position = 145,
	vehicle_imu = 146,
	vehicle_imu_status = 147,
	vehicle_land_detected = 148,
	vehicle_local_position = 149,
	vehicle_local_position_groundtruth = 150,
	vehicle_local_position_setpoint = 151,
	vehicle_magnetometer = 152,
	vehicle_mocap_odometry = 153,
	vehicle_odometry = 154,
	vehicle_rates_setpoint = 155,
	vehicle_roi = 156,
	vehicle_status = 157,
	vehicle_status_flags = 158,
	vehicle_trajectory_bezier = 159,
	vehicle_trajectory_waypoint = 160,
	vehicle_trajectory_waypoint_desired = 161,
	vehicle_vision_attitude = 162,
	vehicle_visual_odometry = 163,
	vtol_vehicle_status = 164,
	wheel_encoders = 165,
	wind_estimate = 166,
	yaw_estimator_status = 167,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
