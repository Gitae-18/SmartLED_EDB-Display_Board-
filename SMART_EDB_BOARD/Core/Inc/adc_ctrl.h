#ifndef ADC_CTRL_H
#define ADC_CTRL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_CTRL_DMA_SAMPLE_COUNT  64U

void ADC_CTRL_Init(void);
bool ADC_CTRL_Start(void);
void ADC_CTRL_Stop(void);
uint16_t ADC_CTRL_GetADC1Latest(void);
uint16_t ADC_CTRL_GetADC1Average(void);
bool ADC_CTRL_ReadADC2Raw(uint16_t *raw, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CTRL_H */
