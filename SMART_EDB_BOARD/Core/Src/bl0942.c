#include "bl0942.h"

#define BL0942_READ_COMMAND       0x58U
#define BL0942_WRITE_COMMAND      0xA8U
#define BL0942_REG_I_RMS          0x03U
#define BL0942_REG_V_RMS          0x04U
#define BL0942_REG_WATT           0x06U
#define BL0942_REG_CF_CNT         0x07U
#define BL0942_REG_FREQ           0x08U
#define BL0942_REG_STATUS         0x09U
#define BL0942_UART_TIMEOUT_MS     30U
#define BL0942_DEVICE_ADDRESS      0U

extern UART_HandleTypeDef huart1;

static uint8_t BL0942_Checksum(const uint8_t *bytes, uint32_t length)
{
    uint8_t sum = 0U;
    uint32_t i;

    for (i = 0U; i < length; ++i)
    {
        sum = (uint8_t)(sum + bytes[i]);
    }
    return (uint8_t)(~sum);
}

static int32_t BL0942_SignExtend24(uint32_t value)
{
    value &= 0x00FFFFFFUL;
    if ((value & 0x00800000UL) != 0U)
    {
        value |= 0xFF000000UL;
    }
    return (int32_t)value;
}

bool BL0942_Init(void)
{
    uint32_t status;

    /* PA4 is labelled RX_ENABLE in the IOC and enables the meter RX path. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(20U);
    return BL0942_ReadRegister(BL0942_REG_STATUS, &status);
}

bool BL0942_ReadRegister(uint8_t reg, uint32_t *value)
{
    uint8_t request[2];
    uint8_t response[4];
    uint8_t checksum_data[5];

    if (value == NULL)
    {
        return false;
    }

    request[0] = (uint8_t)(BL0942_READ_COMMAND | BL0942_DEVICE_ADDRESS);
    request[1] = reg;
    if (HAL_UART_Transmit(&huart1, request, sizeof(request),
                          BL0942_UART_TIMEOUT_MS) != HAL_OK)
    {
        return false;
    }
    if (HAL_UART_Receive(&huart1, response, sizeof(response),
                         BL0942_UART_TIMEOUT_MS) != HAL_OK)
    {
        return false;
    }

    checksum_data[0] = request[0];
    checksum_data[1] = reg;
    checksum_data[2] = response[0];
    checksum_data[3] = response[1];
    checksum_data[4] = response[2];
    if (BL0942_Checksum(checksum_data, sizeof(checksum_data)) != response[3])
    {
        return false;
    }

    *value = (uint32_t)response[0] |
             ((uint32_t)response[1] << 8U) |
             ((uint32_t)response[2] << 16U);
    return true;
}

bool BL0942_WriteRegister(uint8_t reg, uint32_t value)
{
    uint8_t frame[6];

    frame[0] = (uint8_t)(BL0942_WRITE_COMMAND | BL0942_DEVICE_ADDRESS);
    frame[1] = reg;
    frame[2] = (uint8_t)value;
    frame[3] = (uint8_t)(value >> 8U);
    frame[4] = (uint8_t)(value >> 16U);
    frame[5] = BL0942_Checksum(frame, 5U);

    return HAL_UART_Transmit(&huart1, frame, sizeof(frame),
                             BL0942_UART_TIMEOUT_MS) == HAL_OK;
}

bool BL0942_ReadVoltage(float *voltage)
{
    uint32_t raw;
    if ((voltage == NULL) || !BL0942_ReadRegister(BL0942_REG_V_RMS, &raw))
    {
        return false;
    }
    *voltage = (float)raw / BL0942_VOLTAGE_COEFFICIENT;
    return true;
}

bool BL0942_ReadCurrent(float *current)
{
    uint32_t raw;
    if ((current == NULL) || !BL0942_ReadRegister(BL0942_REG_I_RMS, &raw))
    {
        return false;
    }
    *current = (float)raw / BL0942_CURRENT_COEFFICIENT;
    return true;
}

bool BL0942_ReadPower(float *power)
{
    uint32_t raw;
    if ((power == NULL) || !BL0942_ReadRegister(BL0942_REG_WATT, &raw))
    {
        return false;
    }
    *power = (float)BL0942_SignExtend24(raw) / BL0942_POWER_COEFFICIENT;
    return true;
}

bool BL0942_ReadEnergy(float *energy)
{
    uint32_t raw;
    if ((energy == NULL) || !BL0942_ReadRegister(BL0942_REG_CF_CNT, &raw))
    {
        return false;
    }
    *energy = (float)raw / BL0942_ENERGY_COEFFICIENT;
    return true;
}

bool BL0942_ReadFrequency(float *frequency)
{
    uint32_t raw;
    if ((frequency == NULL) || !BL0942_ReadRegister(BL0942_REG_FREQ, &raw) ||
        ((raw & 0xFFFFU) == 0U))
    {
        return false;
    }
    *frequency = 1000000.0f / (float)(raw & 0xFFFFU);
    return true;
}

bool BL0942_ReadAll(BL0942_Data_t *data)
{
    BL0942_Data_t sample;

    if ((data == NULL) || !BL0942_ReadVoltage(&sample.voltage_rms) ||
        !BL0942_ReadCurrent(&sample.current_rms) ||
        !BL0942_ReadPower(&sample.active_power) ||
        !BL0942_ReadEnergy(&sample.energy) ||
        !BL0942_ReadFrequency(&sample.frequency))
    {
        return false;
    }

    *data = sample;
    return true;
}
