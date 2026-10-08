#include "display_protocol.h"

#include "power_monitor.h"
#include "rs485.h"

#include <limits.h>
#include <string.h>

static DisplayProtocol_Status_t display_status;

static void DisplayProtocol_PutU32(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8U);
    buffer[2] = (uint8_t)(value >> 16U);
    buffer[3] = (uint8_t)(value >> 24U);
}

static int32_t DisplayProtocol_ScaleFloat(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= (float)INT32_MAX)
    {
        return INT32_MAX;
    }
    if (scaled <= (float)INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)scaled;
}

static void DisplayProtocol_PutI32(uint8_t *buffer, int32_t value)
{
    DisplayProtocol_PutU32(buffer, (uint32_t)value);
}

static HAL_StatusTypeDef DisplayProtocol_SendPacket(uint8_t destination_id,
                                                     uint8_t command,
                                                     const uint8_t *data,
                                                     uint8_t data_length)
{
    RS485_Packet_t response;
    HAL_StatusTypeDef result;

    if (data_length > RS485_MAX_PAYLOAD_SIZE)
    {
        return HAL_ERROR;
    }

    memset(&response, 0, sizeof(response));
    response.destination_id = destination_id;
    response.source_id = RS485_GetLocalId();
    response.command = command;
    response.data_length = data_length;
    if ((data != NULL) && (data_length > 0U))
    {
        memcpy(response.data, data, data_length);
    }

    result = RS485_SendPacket(&response);
    if (result == HAL_OK)
    {
        ++display_status.transmitted_packets;
    }
    else
    {
        ++display_status.protocol_errors;
    }
    return result;
}

static void DisplayProtocol_SendError(uint8_t destination_id,
                                      uint8_t rejected_command,
                                      DisplayProtocol_Error_t error)
{
    uint8_t payload[2];

    payload[0] = rejected_command;
    payload[1] = (uint8_t)error;
    (void)DisplayProtocol_SendPacket(destination_id, DISPLAY_CMD_ERROR,
                                     payload, sizeof(payload));
}

void DisplayProtocol_Init(void)
{
    memset(&display_status, 0, sizeof(display_status));
    display_status.ac_output_enabled =
        (HAL_GPIO_ReadPin(AC_Ctrl_GPIO_Port, AC_Ctrl_Pin) == GPIO_PIN_SET);
}

HAL_StatusTypeDef DisplayProtocol_SendMeasurements(uint8_t destination_id)
{
    const PowerMonitor_Data_t *measurement = PowerMonitor_GetData();
    uint8_t payload[DISPLAY_PROTOCOL_MEASUREMENT_PAYLOAD_SIZE];

    DisplayProtocol_PutU32(&payload[0], measurement->updated_at_ms);
    DisplayProtocol_PutI32(&payload[4],
        DisplayProtocol_ScaleFloat(measurement->voltage_rms, 1000.0f));
    DisplayProtocol_PutI32(&payload[8],
        DisplayProtocol_ScaleFloat(measurement->current_rms, 1000.0f));
    DisplayProtocol_PutI32(&payload[12],
        DisplayProtocol_ScaleFloat(measurement->auxiliary_current, 1000.0f));
    DisplayProtocol_PutI32(&payload[16],
        DisplayProtocol_ScaleFloat(measurement->active_power, 1000.0f));
    DisplayProtocol_PutI32(&payload[20],
        DisplayProtocol_ScaleFloat(measurement->energy, 1000.0f));
    DisplayProtocol_PutI32(&payload[24],
        DisplayProtocol_ScaleFloat(measurement->frequency, 1000.0f));
    payload[28] = measurement->meter_valid ? 1U : 0U;

    return DisplayProtocol_SendPacket(
        destination_id,
        (uint8_t)(DISPLAY_CMD_GET_MEASUREMENTS | DISPLAY_CMD_RESPONSE_FLAG),
        payload, sizeof(payload));
}

HAL_StatusTypeDef DisplayProtocol_SendPing(uint8_t destination_id)
{
    uint8_t payload[4];

    DisplayProtocol_PutU32(payload, HAL_GetTick());
    return DisplayProtocol_SendPacket(destination_id, DISPLAY_CMD_PING,
                                      payload, sizeof(payload));
}

const DisplayProtocol_Status_t *DisplayProtocol_GetStatus(void)
{
    return &display_status;
}

void DisplayProtocol_Process(void)
{
    RS485_Packet_t request;
    RS485_FrameStatus_t receive_status;
    uint8_t payload[10];
    bool send_response;

    receive_status = RS485_ReceivePacket(&request,
                                         DISPLAY_PROTOCOL_RECEIVE_TIMEOUT_MS);
    if (receive_status != RS485_FRAME_OK)
    {
        return;
    }

    ++display_status.received_packets;
    send_response = (request.destination_id != RS485_BROADCAST_ID);

    /* Responses are terminal packets. Do not answer them again. */
    if ((request.command & DISPLAY_CMD_RESPONSE_FLAG) != 0U)
    {
        ++display_status.received_responses;
        display_status.last_response_command = request.command;
        return;
    }

    switch (request.command)
    {
        case DISPLAY_CMD_GET_MEASUREMENTS:
            if (request.data_length != 0U)
            {
                ++display_status.protocol_errors;
                if (send_response)
                {
                    DisplayProtocol_SendError(request.source_id, request.command,
                                              DISPLAY_PROTOCOL_ERROR_BAD_LENGTH);
                }
            }
            else if (send_response)
            {
                (void)DisplayProtocol_SendMeasurements(request.source_id);
            }
            break;

        case DISPLAY_CMD_GET_STATUS:
            if (request.data_length != 0U)
            {
                ++display_status.protocol_errors;
                if (send_response)
                {
                    DisplayProtocol_SendError(request.source_id, request.command,
                                              DISPLAY_PROTOCOL_ERROR_BAD_LENGTH);
                }
                break;
            }
            payload[0] = PowerMonitor_IsValid() ? 1U : 0U;
            payload[1] = display_status.ac_output_enabled ? 1U : 0U;
            DisplayProtocol_PutU32(&payload[2], display_status.received_packets);
            DisplayProtocol_PutU32(&payload[6], display_status.protocol_errors);
            if (send_response)
            {
                (void)DisplayProtocol_SendPacket(
                    request.source_id,
                    (uint8_t)(DISPLAY_CMD_GET_STATUS | DISPLAY_CMD_RESPONSE_FLAG),
                    payload, 10U);
            }
            break;

        case DISPLAY_CMD_SET_AC_OUTPUT:
            if ((request.data_length != 1U) || (request.data[0] > 1U))
            {
                ++display_status.protocol_errors;
                if (send_response)
                {
                    DisplayProtocol_SendError(request.source_id, request.command,
                                              DISPLAY_PROTOCOL_ERROR_BAD_LENGTH);
                }
                break;
            }
            display_status.ac_output_enabled = (request.data[0] != 0U);
            HAL_GPIO_WritePin(AC_Ctrl_GPIO_Port, AC_Ctrl_Pin,
                display_status.ac_output_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
            if (send_response)
            {
                payload[0] = display_status.ac_output_enabled ? 1U : 0U;
                (void)DisplayProtocol_SendPacket(
                    request.source_id,
                    (uint8_t)(DISPLAY_CMD_SET_AC_OUTPUT | DISPLAY_CMD_RESPONSE_FLAG),
                    payload, 1U);
            }
            break;

        case DISPLAY_CMD_PING:
            if (send_response)
            {
                (void)DisplayProtocol_SendPacket(
                    request.source_id,
                    (uint8_t)(DISPLAY_CMD_PING | DISPLAY_CMD_RESPONSE_FLAG),
                    request.data, request.data_length);
            }
            break;

        default:
            ++display_status.protocol_errors;
            if (send_response)
            {
                DisplayProtocol_SendError(request.source_id, request.command,
                                          DISPLAY_PROTOCOL_ERROR_BAD_COMMAND);
            }
            break;
    }
}
