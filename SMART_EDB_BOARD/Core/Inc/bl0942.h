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
#define BL0942_VOLTAGE_COEFFICIENT  0.000062956f
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

typedef enum
{
    BL0942_RESULT_NONE = 0,
    BL0942_RESULT_OK,
    BL0942_RESULT_TX_TIMEOUT,
    BL0942_RESULT_TX_ERROR,
    BL0942_RESULT_RX_TIMEOUT,
    BL0942_RESULT_RX_ERROR,
    BL0942_RESULT_CHECKSUM,
    BL0942_RESULT_INVALID_ARGUMENT,
    BL0942_RESULT_INVALID_FREQUENCY
} BL0942_Result_t;

typedef struct
{
    BL0942_Result_t last_result;
    HAL_StatusTypeDef hal_status;
    uint32_t uart_error;
    uint32_t tx_failures;
    uint32_t rx_timeouts;
    uint32_t rx_errors;
    uint32_t checksum_errors;
    uint8_t reg;
    uint8_t received_bytes;
    uint8_t response[4];
    uint8_t checksum_expected;
    uint8_t checksum_received;
    uint32_t i_rms_raw;
    uint32_t v_rms_raw;
    uint32_t watt_raw;
    uint32_t energy_raw;
    uint32_t frequency_raw;
} BL0942_Diagnostics_t;

const BL0942_Diagnostics_t *BL0942_GetDiagnostics(void);
const char *BL0942_ResultName(BL0942_Result_t result);

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
