#ifndef RS485_H
#define RS485_H

#include "main.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS485_STX                 0x02U
#define RS485_ETX                 0x03U
#define RS485_BROADCAST_ID        0xFFU
#define RS485_MAX_PAYLOAD_SIZE    64U
#define RS485_FRAME_OVERHEAD      8U
#define RS485_MAX_FRAME_SIZE      (RS485_MAX_PAYLOAD_SIZE + RS485_FRAME_OVERHEAD)

typedef enum
{
    RS485_FRAME_OK = 0,
    RS485_FRAME_INCOMPLETE,
    RS485_FRAME_BAD_FORMAT,
    RS485_FRAME_BAD_LENGTH,
    RS485_FRAME_BAD_CRC,
    RS485_FRAME_NOT_FOR_THIS_NODE
} RS485_FrameStatus_t;

typedef struct
{
    uint8_t destination_id;
    uint8_t source_id;
    uint8_t command;
    uint8_t data_length;
    uint8_t data[RS485_MAX_PAYLOAD_SIZE];
} RS485_Packet_t;

void RS485_Init(void);
void RS485_SetLocalId(uint8_t local_id);
uint8_t RS485_GetLocalId(void);

void RS485_SetTxMode(void);
void RS485_SetRxMode(void);

HAL_StatusTypeDef RS485_Send(
    const uint8_t *data,
    uint16_t len
);

HAL_StatusTypeDef RS485_Receive(
    uint8_t *data,
    uint16_t len,
    uint32_t timeout
);

uint16_t RS485_CRC16(const uint8_t *data, size_t len);
size_t RS485_EncodeFrame(const RS485_Packet_t *packet, uint8_t *frame, size_t capacity);
RS485_FrameStatus_t RS485_DecodeFrame(const uint8_t *frame, size_t frame_len,
                                      RS485_Packet_t *packet);
HAL_StatusTypeDef RS485_SendPacket(const RS485_Packet_t *packet);
RS485_FrameStatus_t RS485_ReceivePacket(RS485_Packet_t *packet, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* RS485_H */
