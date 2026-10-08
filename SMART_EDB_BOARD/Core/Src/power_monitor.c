#include "power_monitor.h"

#include "bl0942.h"
#include "main.h"
#include "mcs1806.h"

#include <string.h>

static PowerMonitor_Data_t power_monitor_data;
static uint32_t power_monitor_interval_ms = POWER_MONITOR_UPDATE_INTERVAL_MS;
static uint32_t power_monitor_last_attempt_ms;

void PowerMonitor_Init(void)
{
    memset(&power_monitor_data, 0, sizeof(power_monitor_data));
    power_monitor_interval_ms = POWER_MONITOR_UPDATE_INTERVAL_MS;

    /* Make the first call to PowerMonitor_Update() take a sample immediately. */
    power_monitor_last_attempt_ms = HAL_GetTick() - power_monitor_interval_ms;
}

bool PowerMonitor_ForceUpdate(void)
{
    BL0942_Data_t meter_data;
    uint32_t now = HAL_GetTick();

    power_monitor_last_attempt_ms = now;

    /* The auxiliary current sensor is independent of the BL0942. */
    power_monitor_data.auxiliary_current = MCS1806_ReadCurrent();

    if (!BL0942_ReadAll(&meter_data))
    {
        power_monitor_data.meter_valid = false;
        ++power_monitor_data.failed_updates;
        return false;
    }

    power_monitor_data.voltage_rms = meter_data.voltage_rms;
    power_monitor_data.current_rms = meter_data.current_rms;
    power_monitor_data.active_power = meter_data.active_power;
    power_monitor_data.energy = meter_data.energy;
    power_monitor_data.frequency = meter_data.frequency;
    power_monitor_data.updated_at_ms = now;
    power_monitor_data.meter_valid = true;
    ++power_monitor_data.successful_updates;
    return true;
}

void PowerMonitor_Update(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - power_monitor_last_attempt_ms) >= power_monitor_interval_ms)
    {
        (void)PowerMonitor_ForceUpdate();
    }
}

const PowerMonitor_Data_t *PowerMonitor_GetData(void)
{
    return &power_monitor_data;
}

bool PowerMonitor_GetSnapshot(PowerMonitor_Data_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    *data = power_monitor_data;
    return power_monitor_data.meter_valid;
}

bool PowerMonitor_IsValid(void)
{
    return power_monitor_data.meter_valid;
}

void PowerMonitor_SetUpdateInterval(uint32_t interval_ms)
{
    if (interval_ms > 0U)
    {
        power_monitor_interval_ms = interval_ms;
    }
}

uint32_t PowerMonitor_GetUpdateInterval(void)
{
    return power_monitor_interval_ms;
}
