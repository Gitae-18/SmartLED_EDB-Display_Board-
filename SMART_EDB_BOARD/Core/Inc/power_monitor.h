#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Override at compile time when a different sampling period is required. */
#ifndef POWER_MONITOR_UPDATE_INTERVAL_MS
#define POWER_MONITOR_UPDATE_INTERVAL_MS  500U
#endif

typedef struct
{
    float voltage_rms;
    float current_rms;
    float active_power;
    float energy;
    float frequency;
    float auxiliary_current;
    uint32_t updated_at_ms;
    uint32_t successful_updates;
    uint32_t failed_updates;
    bool meter_valid;
} PowerMonitor_Data_t;

void PowerMonitor_Init(void);
void PowerMonitor_Update(void);
bool PowerMonitor_ForceUpdate(void);

const PowerMonitor_Data_t *PowerMonitor_GetData(void);
bool PowerMonitor_GetSnapshot(PowerMonitor_Data_t *data);
bool PowerMonitor_IsValid(void);

void PowerMonitor_SetUpdateInterval(uint32_t interval_ms);
uint32_t PowerMonitor_GetUpdateInterval(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MONITOR_H */
