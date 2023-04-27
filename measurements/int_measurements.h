#ifndef INT_MEASUREMENTS_H_
#define INT_MEASUREMENTS_H_

#include <stdbool.h>
#include <stdint.h>

#include "defines.h"

void measurements_open();
void measurement_critical_enter();
void measurement_critical_exit();
bool is_measurement_critical();
void measurements_close();
bool measurement_get(Measurement_t* measurement);

#endif /* INT_MEASUREMENTS_H_ */
