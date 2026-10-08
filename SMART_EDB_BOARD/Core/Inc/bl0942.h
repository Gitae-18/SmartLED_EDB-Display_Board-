#ifndef BL0942_H
#define BL0942_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Board-dependent calibration. Adjust these after measuring a known load. */
#ifndef BL0942_VOLTAGE_COEFFICIENT
#define BL0942_VOLTAGE_COEFFICIENT  73989.0f
#endif
#ifndef BL0942_CURRENT_COEFFICIENT
#define BL0942_CURRENT_COEFFICIENT  305978.0f
#endif
#ifndef BL0942_POWER_COEFFICIENT
#define BL0942_POWER_COEFFICIENT    3537.0f
#endif
#ifndef BL0942_ENERGY_COEFFICIENT
/* CF counts per Wh, derived from the datasheet CF period equation. */
#define BL0942_ENERGY_COEFFICIENT   ((BL0942_POWER_COEFFICIENT * 3600.0f) / 419430.4f)
#endif

typedef struct
{
    float voltage_rms;
    float current_rms;
    float active_power;
    float energy;
    float frequency;
} BL0942_Data_t;

bool BL0942_Init(void);
bool BL0942_ReadRegister(uint8_t reg, uint32_t *value);
bool BL0942_WriteRegister(uint8_t reg, uint32_t value);

bool BL0942_ReadVoltage(float *voltage);
bool BL0942_ReadCurrent(float *current);
bool BL0942_ReadPower(float *power);
bool BL0942_ReadEnergy(float *energy);
bool BL0942_ReadFrequency(float *frequency);

bool BL0942_ReadAll(BL0942_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* BL0942_H */
