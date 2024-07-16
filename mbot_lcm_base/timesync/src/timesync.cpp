#include <iostream>
#include <unistd.h>

#include <lcm/lcm-cpp.hpp>

#include <timesync/lcm_config.h>
#include <timesync/timestamp.h>

#include <mbot_lcm_msgs/timestamp_t.hpp>

/**
	A program that gets the current system time and publishes an lcm message of the current time
**/
int main(){

	//sleep duration between time samplings
	const int sleep_usec = 1000000;

	lcm::LCM lcmConnection(MULTICAST_URL);
	if(!lcmConnection.good()) return 1;

	mbot_lcm_msgs::timestamp_t now;

	while(true){

		now.utime = utime_now();

		lcmConnection.publish(MBOT_TIMESYNC_CHANNEL, &now);

		usleep(sleep_usec);
	}

	return 0;
}
