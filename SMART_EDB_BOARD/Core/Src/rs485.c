#include "rs485.h"
#include <string.h>

#define RS485_UART_TIMEOUT_MS  100U

extern UART_HandleTypeDef huart2;

static uint8_t rs485_local_id = 1U;

static uint32_t RS485_RemainingTimeout(uint32_t start, uint32_t timeout)
{
    uint32_t elapsed = HAL_GetTick() - start;
    return (elapsed < timeout) ? (timeout - elapsed) : 0U;
}

void RS485_Init(void)
{
    RS485_SetRxMode();
}

void RS485_SetLocalId(uint8_t local_id)
{
    if (local_id != RS485_BROADCAST_ID)
    {
        rs485_local_id = local_id;
    }
}

uint8_t RS485_GetLocalId(void)
{
    return rs485_local_id;
}

void RS485_SetTxMode(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
}

void RS485_SetRxMode(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    RS485_SetTxMode();
    status = HAL_UART_Transmit(&huart2, data, len, RS485_UART_TIMEOUT_MS);
    while ((status == HAL_OK) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == 0U))
    {
    }
    RS485_SetRxMode();
    return status;
}

HAL_StatusTypeDef RS485_Receive(uint8_t *data, uint16_t len, uint32_t timeout)
{
    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    RS485_SetRxMode();
    return HAL_UART_Receive(&huart2, data, len, timeout);
}

uint16_t RS485_CRC16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    if (data == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 1U) != 0U) ?
                  (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

size_t RS485_EncodeFrame(const RS485_Packet_t *packet, uint8_t *frame, size_t capacity)
{
    size_t frame_len;
    uint16_t crc;

    if ((packet == NULL) || (frame == NULL) ||
        (packet->data_length > RS485_MAX_PAYLOAD_SIZE))
    {
        return 0U;
    }

    frame_len = (size_t)packet->data_length + RS485_FRAME_OVERHEAD;
    if (capacity < frame_len)
    {
        return 0U;
    }

    frame[0] = RS485_STX;
    frame[1] = (uint8_t)(3U + packet->data_length);
    frame[2] = packet->destination_id;
    frame[3] = packet->source_id;
    frame[4] = packet->command;
    if (packet->data_length > 0U)
    {
        memcpy(&frame[5], packet->data, packet->data_length);
    }

    crc = RS485_CRC16(&frame[1], (size_t)frame[1] + 1U);
    frame[5U + packet->data_length] = (uint8_t)crc;
    frame[6U + packet->data_length] = (uint8_t)(crc >> 8U);
    frame[7U + packet->data_length] = RS485_ETX;
    return frame_len;
}

RS485_FrameStatus_t RS485_DecodeFrame(const uint8_t *frame, size_t frame_len,
                                      RS485_Packet_t *packet)
{
    uint8_t body_len;
    uint8_t payload_len;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((frame == NULL) || (packet == NULL) || (frame_len < RS485_FRAME_OVERHEAD))
    {
        return RS485_FRAME_INCOMPLETE;
    }
    if ((frame[0] != RS485_STX) || (frame[frame_len - 1U] != RS485_ETX))
    {
        return RS485_FRAME_BAD_FORMAT;
    }

    body_len = frame[1];
    if ((body_len < 3U) || (body_len > (3U + RS485_MAX_PAYLOAD_SIZE)) ||
        (frame_len != ((size_t)body_len + 5U)))
    {
        return RS485_FRAME_BAD_LENGTH;
    }

    received_crc = (uint16_t)frame[2U + body_len] |
                   ((uint16_t)frame[3U + body_len] << 8U);
    calculated_crc = RS485_CRC16(&frame[1], (size_t)body_len + 1U);
    if (received_crc != calculated_crc)
    {
        return RS485_FRAME_BAD_CRC;
    }

    if ((frame[2] != rs485_local_id) && (frame[2] != RS485_BROADCAST_ID))
    {
        return RS485_FRAME_NOT_FOR_THIS_NODE;
    }

    payload_len = (uint8_t)(body_len - 3U);
    packet->destination_id = frame[2];
    packet->source_id = frame[3];
    packet->command = frame[4];
    packet->data_length = payload_len;
    if (payload_len > 0U)
    {
        memcpy(packet->data, &frame[5], payload_len);
    }
    return RS485_FRAME_OK;
}

HAL_StatusTypeDef RS485_SendPacket(const RS485_Packet_t *packet)
{
    uint8_t frame[RS485_MAX_FRAME_SIZE];
    size_t length = RS485_EncodeFrame(packet, frame, sizeof(frame));

    if (length == 0U)
    {
        return HAL_ERROR;
    }
    return RS485_Send(frame, (uint16_t)length);
}

RS485_FrameStatus_t RS485_ReceivePacket(RS485_Packet_t *packet, uint32_t timeout)
{
    uint8_t frame[RS485_MAX_FRAME_SIZE];
    uint8_t byte;
    uint8_t body_len;
    uint32_t start = HAL_GetTick();
    uint32_t remaining;
    size_t tail_len;

    if (packet == NULL)
    {
        return RS485_FRAME_BAD_FORMAT;
    }

    do
    {
        remaining = RS485_RemainingTimeout(start, timeout);
        if (remaining == 0U)
        {
            return RS485_FRAME_INCOMPLETE;
        }
        if (RS485_Receive(&byte, 1U, remaining) != HAL_OK)
        {
            return RS485_FRAME_INCOMPLETE;
        }
    } while ((byte != RS485_STX) && ((HAL_GetTick() - start) < timeout));

    frame[0] = byte;
    remaining = RS485_RemainingTimeout(start, timeout);
    if (remaining == 0U)
    {
        return RS485_FRAME_INCOMPLETE;
    }
    if (RS485_Receive(&body_len, 1U, remaining) != HAL_OK)
    {
        return RS485_FRAME_INCOMPLETE;
    }
    frame[1] = body_len;
    if ((body_len < 3U) || (body_len > (3U + RS485_MAX_PAYLOAD_SIZE)))
    {
        return RS485_FRAME_BAD_LENGTH;
    }

    tail_len = (size_t)body_len + 3U;
    remaining = RS485_RemainingTimeout(start, timeout);
    if (remaining == 0U)
    {
        return RS485_FRAME_INCOMPLETE;
    }
    if (RS485_Receive(&frame[2], (uint16_t)tail_len, remaining) != HAL_OK)
    {
        return RS485_FRAME_INCOMPLETE;
    }
    return RS485_DecodeFrame(frame, (size_t)body_len + 5U, packet);
}
