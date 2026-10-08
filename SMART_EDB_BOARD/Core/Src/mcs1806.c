#include "mcs1806.h"
#include "adc_ctrl.h"

static float mcs1806_zero_voltage = MCS1806_DEFAULT_ZERO_V;

void MCS1806_Init(void)
{
    mcs1806_zero_voltage = MCS1806_DEFAULT_ZERO_V;
}

uint16_t MCS1806_ReadRaw(void)
{
    uint32_t sum = 0U;
    uint16_t raw = 0U;
    uint32_t valid_samples = 0U;
    uint32_t i;

    for (i = 0U; i < MCS1806_SAMPLE_COUNT; ++i)
    {
        if (ADC_CTRL_ReadADC2Raw(&raw, 10U))
        {
            sum += raw;
            ++valid_samples;
        }
    }

    if (valid_samples == 0U)
    {
        return 0U;
    }

    return (uint16_t)(sum / valid_samples);
}

float MCS1806_ReadVoltage(void)
{
    return ((float)MCS1806_ReadRaw() * MCS1806_ADC_REFERENCE_V) /
           MCS1806_ADC_MAX_COUNT;
}

float MCS1806_ReadCurrent(void)
{
    return (MCS1806_ReadVoltage() - mcs1806_zero_voltage) /
           MCS1806_SENSITIVITY;
}

void MCS1806_SetZero(float zero_voltage)
{
    if ((zero_voltage >= 0.0f) && (zero_voltage <= MCS1806_ADC_REFERENCE_V))
    {
        mcs1806_zero_voltage = zero_voltage;
    }
}
