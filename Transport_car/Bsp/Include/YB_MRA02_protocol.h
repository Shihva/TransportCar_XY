#ifndef YB_MRA02_PROTOCOL_H
#define YB_MRA02_PROTOCOL_H

#include <stdint.h>

/* YB-MRA02-V1.0: 通信协议.xlsx / 串口通讯协议。
 * 7E 23 | 总长度 | 功能字 | 小端数据 | 累加和低8位；无结束符。
 * 已定义的最长帧是23字节原始数据帧。 */
#define YB_MRA02_MAX_FRAME 23U
#define YB_MRA02_STREAM_CAPACITY (2U * YB_MRA02_MAX_FRAME)

typedef struct {
    uint8_t length;
    uint8_t bytes[YB_MRA02_MAX_FRAME];
} YB_MRA02_Frame;

typedef struct {
    uint8_t bytes[YB_MRA02_STREAM_CAPACITY];
    uint16_t used;
    uint32_t checksum_errors;
    uint32_t discarded_bytes;
} YB_MRA02_Stream;

void YB_MRA02_StreamInit(YB_MRA02_Stream *stream);
void YB_MRA02_StreamPush(YB_MRA02_Stream *stream, uint8_t byte);
uint8_t YB_MRA02_StreamNext(YB_MRA02_Stream *stream, YB_MRA02_Frame *out);
/* 仅接受校验正确的0x26欧拉角帧，将弧度float转为[-180,180]度。 */
uint8_t YB_MRA02_DecodeYaw(const YB_MRA02_Frame *frame, float *yaw_deg);

#endif
