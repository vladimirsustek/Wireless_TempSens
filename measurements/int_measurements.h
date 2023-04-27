#ifndef INT_MEASUREMENTS_H_
#define INT_MEASUREMENTS_H_

#include <measurement_defines.h>
#include <stdbool.h>
#include <stdint.h>


void IntMeas_Open();
void measurement_critical_enter();
void measurement_critical_exit();
bool is_measurement_critical();
void IntMeas_Close();
bool IntMeas_Get(Measurement_t* measurement);

#endif /* INT_MEASUREMENTS_H_ */
