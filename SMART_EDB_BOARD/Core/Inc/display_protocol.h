#ifndef DISPLAY_PROTOCOL_H
#define DISPLAY_PROTOCOL_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Application commands carried in RS485_Packet_t.command. */
typedef enum
{
    DISPLAY_CMD_GET_MEASUREMENTS = 0x01U,
    DISPLAY_CMD_GET_STATUS       = 0x02U,
    DISPLAY_CMD_SET_AC_OUTPUT    = 0x10U,
    DISPLAY_CMD_PING             = 0x7EU,
    DISPLAY_CMD_RESPONSE_FLAG    = 0x80U,
    DISPLAY_CMD_ERROR            = 0xFFU
} DisplayProtocol_Command_t;

typedef enum
{
    DISPLAY_PROTOCOL_OK = 0U,
    DISPLAY_PROTOCOL_ERROR_BAD_COMMAND = 1U,
    DISPLAY_PROTOCOL_ERROR_BAD_LENGTH = 2U,
    DISPLAY_PROTOCOL_ERROR_SEND_FAILED = 3U
} DisplayProtocol_Error_t;

typedef struct
{
    uint32_t received_packets;
    uint32_t transmitted_packets;
    uint32_t received_responses;
    uint32_t protocol_errors;
    uint8_t last_response_command;
    bool ac_output_enabled;
} DisplayProtocol_Status_t;

/*
 * Measurement response payload (little endian):
 *   u32 updated_at_ms
 *   i32 voltage_mV
 *   i32 current_mA
 *   i32 auxiliary_current_mA
 *   i32 active_power_mW
 *   i32 energy_mWh
 *   i32 frequency_mHz
 *   u8  meter_valid
 */
#define DISPLAY_PROTOCOL_MEASUREMENT_PAYLOAD_SIZE  29U
#define DISPLAY_PROTOCOL_RECEIVE_TIMEOUT_MS         1U

void DisplayProtocol_Init(void);
void DisplayProtocol_Process(void);

HAL_StatusTypeDef DisplayProtocol_SendMeasurements(uint8_t destination_id);
HAL_StatusTypeDef DisplayProtocol_SendPing(uint8_t destination_id);
const DisplayProtocol_Status_t *DisplayProtocol_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_PROTOCOL_H */
