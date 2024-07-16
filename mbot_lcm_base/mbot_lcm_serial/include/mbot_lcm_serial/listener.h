#include <stdio.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <search.h>

#include "comms_common.h"
#include "protocol.h"
#include "topic_data.h"

#ifndef LISTENER_H
#define LISTENER_H
extern bool listener_running;
void* comms_listener_loop(void* arg);

#endif
