#ifndef ADC_CTRL_H
#define ADC_CTRL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_CTRL_DMA_SAMPLE_COUNT  64U

/*
 * 30V_ADC divider: R33=100 kOhm, R35=5.1 kOhm, buffered by TLV9002.
 * With VDDA=3.3 V, ADC full scale represents about 68.006 V at VCC_30V.
 */
#ifndef ADC_CTRL_DC_VOLTAGE_FULL_SCALE_V
#define ADC_CTRL_DC_VOLTAGE_FULL_SCALE_V  68.0059f
#endif

void ADC_CTRL_Init(void);
bool ADC_CTRL_Start(void);
void ADC_CTRL_Stop(void);
uint16_t ADC_CTRL_GetADC1Latest(void);
uint16_t ADC_CTRL_GetADC1Average(void);
bool ADC_CTRL_ReadDCVoltageRaw(uint16_t *raw, uint32_t timeout_ms);
bool ADC_CTRL_ReadDCVoltage(float *voltage, uint32_t timeout_ms);
bool ADC_CTRL_ReadADC2Raw(uint16_t *raw, uint32_t timeout_ms);
bool ADC_CTRL_ReadNTC(uint16_t *raw, float *voltage, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CTRL_H */
