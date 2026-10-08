#ifndef MCS1806_H
#define MCS1806_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCS1806_SENSITIVITY   0.264f
#define MCS1806_DEFAULT_ZERO_V  1.650f
#define MCS1806_ADC_REFERENCE_V 3.300f
#define MCS1806_ADC_MAX_COUNT   4095.0f  /* ADC2 is configured for 12 bit */
#define MCS1806_SAMPLE_COUNT    16U

typedef struct
{
    float voltage;
    float current;
} MCS1806_Data_t;

void MCS1806_Init(void);

uint16_t MCS1806_ReadRaw(void);
float MCS1806_ReadVoltage(void);
float MCS1806_ReadCurrent(void);

void MCS1806_SetZero(float zero_voltage);

#ifdef __cplusplus
}
#endif

#endif /* MCS1806_H */
