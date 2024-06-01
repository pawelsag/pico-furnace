#pragma once

#include <stdint.h>
#if CONFIG_SPIN_COATER == CONFIG_SPIN_COATER_DSHOT
#include "dshot_encoder.pio.h"
#elif CONFIG_SPIN_COATER == CONFIG_SPIN_COATER_BIDIR_DSHOT
#include "dshot_bdir_encoder.pio.h"
#endif
#define SPIN_COATER_MIN_THROTTLE_COMMAND dshot_encoder_MIN_THROTTLE_VALUE
#define SPIN_COATER_MAX_THROTTLE_COMMAND dshot_encoder_MAX_THROTTLE_VALUE
/* Normaly IDLE state for the engine is SPIN_COATER_MIN_THROTTLE_COMMAND dshot value
   but since the engine is under load it starts spining from 100 dshot value
   instead of 49 dshot value. This variable was added to speed up process
   of spining up the engine in automatic mode with timer to skip unsued values */
#define DSHOT_HEAVY_LOADED_IDLE_DUTY 100

bool dshot_init(uint16_t dshot_gpio);

void dshot_send_command(uint16_t c);

#if CONFIG_SPIN_COATER == CONFIG_SPIN_COATER_BIDIR_DSHOT
uint32_t
dshot_recv_telemetry();
#endif