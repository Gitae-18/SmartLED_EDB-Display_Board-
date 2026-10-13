#include "mcs1806.h"
#include "adc_ctrl.h"

static float mcs1806_zero_voltage = MCS1806_DEFAULT_ZERO_V;

void MCS1806_Init(void)
{
    mcs1806_zero_voltage = MCS1806_DEFAULT_ZERO_V;
}

uint16_t MCS1806_ReadRaw(void)
{
    /*
     * CURRENT_ADC is connected to PA6/ADC1_INP3. ADC1 samples this input
     * continuously into the DMA buffer; PC4/ADC2_INP4 is reserved for NTC_TEMP.
     */
    return ADC_CTRL_GetADC1Average();
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
