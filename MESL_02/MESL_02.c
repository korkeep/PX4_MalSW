/**
 * @file MESL_02.c
 * Cannod withstand environmental hazrads
 * @author korkeep <korkeep@khu.ac.kr>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/wind_estimate.h>

__EXPORT int MESL_02_main(int argc, char *argv[]);

int MESL_02_main(int argc, char *argv[])
{
	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(wind_estimate));
	/* limit the update rate to 5 Hz */
	//orb_set_interval(sensor_sub_fd, 200);

	/* one could wait for multiple topics with this technique, just using one here */
	//px4_pollfd_struct_t fds[] = {
	//	{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	//};

	//int error_counter = 0;

	for (int i = 0; i < 20; i++) {
		/* wait for sensor update of 1 file descriptor for 250 ms */
		//int poll_ret = px4_poll(fds, 1, 250);

		/* handle the poll result */
		//if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			//PX4_ERR("Got no data within a second");

		//} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			//if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				//PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			//}

			//error_counter++;

		//} else {
			//if (fds[0].revents & POLLIN) {
				// obtained data for the first file descriptor
				struct wind_estimate_s wind_value;
				// copy sensors raw data into local buffer
				orb_copy(ORB_ID(wind_estimate), sensor_sub_fd, &wind_value);
				
				PX4_INFO("Original Wind Estimator:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
						(double)wind_value.windspeed_north,
						(double)wind_value.windspeed_east,
						(double)wind_value.variance_north,
						(double)wind_value.variance_east,
						(double)wind_value.tas_innov,
						(double)wind_value.tas_innov_var,
						(double)wind_value.tas_scale,
						(double)wind_value.beta_innov,
						(double)wind_value.beta_innov_var);

				wind_value.windspeed_north = 0,
				wind_value.windspeed_east = 0,
				wind_value.variance_north = 0, //
				wind_value.variance_east = 0,
				wind_value.tas_innov = 0, //
				wind_value.tas_innov_var = 0,
				wind_value.tas_scale = 0, //
				wind_value.beta_innov = 0,
				wind_value.beta_innov_var = 0;

				/*
				wind_value.variance_north = 0;
				wind_value.variance_east = 0;

				orb_advert_t wind_pub = orb_advertise(ORB_ID(wind_estimate), &wind_value);
				orb_publish(ORB_ID(wind_estimate), wind_pub, &wind_value);

				PX4_INFO("Updated Wind Estimator:\t%8.4f\t%8.4f",
						(double)wind_value.variance_north,
						(double)wind_value.variance_east);*/
				/*
				if(argc == 4){
					PX4_INFO("Publishing accelerometer data as user input.");

					// set and publish malicious data as input value
					struct wind_estimate_s wind_value;
					accel_user_input.accelerometer_m_s2[0] = atof(argv[1]);
					accel_user_input.accelerometer_m_s2[1] = atof(argv[2]);
					accel_user_input.accelerometer_m_s2[2] = atof(argv[3]);
					
					orb_advert_t accel_pub = orb_advertise(ORB_ID(sensor_combined), &accel_user_input);
					orb_publish(ORB_ID(sensor_combined), accel_pub, &accel_user_input);
					
					PX4_INFO("Original Accel:\t%8.4f\t%8.4f\t%8.4f",
						(double)accel.accelerometer_m_s2[0],
						(double)accel.accelerometer_m_s2[1],
						(double)accel.accelerometer_m_s2[2]);

					PX4_INFO("Updated Accel:\t%8.4f\t%8.4f\t%8.4f",
						(double)accel_user_input.accelerometer_m_s2[0],
						(double)accel_user_input.accelerometer_m_s2[1],
						(double)accel_user_input.accelerometer_m_s2[2]);
				}
				else {
					PX4_ERR("Please input 3 float values (X, Y, Z) Ex) MESL_02 11.1 22.2 33.0");
					break;
				}*/
			//}
		//}
	}
	return 0;
}
