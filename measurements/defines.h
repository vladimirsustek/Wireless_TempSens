#ifndef DEFINES_H_
#define DEFINES_H_

#include <stdint.h>

typedef struct Measurement
{
	int32_t ntc2;
	int32_t ntc1;
	int32_t oagp;
	int32_t curr;
	int32_t tmpi;
	int32_t vdda;
	int32_t tmpe;
}Measurement_t;

#endif /* DEFINES_H_ */
