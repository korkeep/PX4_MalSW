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
#include <time.h>

__EXPORT int MESL_03_main(int argc, char *argv[]);

int MESL_03_main(int argc, char *argv[])
{
	// Maintaining sleep state for input value
	int cnt = atoi(argv[1]);
	PX4_INFO("Drone goes sleep state for %d seconds", cnt);
	sleep(cnt);

	// Clear buffer input (first command input)
	while(getchar() != '\n');

	// Goes to endless sleep state
	//while(getchar() != EOF);

	return 0;
}
