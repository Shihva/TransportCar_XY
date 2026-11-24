#include "YB_MRA02_protocol.h"

#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == sizeof(uint32_t), "IMU requires 32-bit float");

static uint8_t valid_frame(const uint8_t *bytes, uint8_t length)
{
    uint8_t sum = 0U;
    if (length < 5U || length > YB_MRA02_MAX_FRAME ||
        bytes[0] != 0x7EU || bytes[1] != 0x23U || bytes[2] != length)
        return 0U;
    for (uint8_t i = 0U; i < length - 1U; ++i) sum = (uint8_t)(sum + bytes[i]);
    return (uint8_t)(sum == bytes[length - 1U]);
}

static float read_float_le(const uint8_t *bytes)
{
    uint32_t bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                    ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
    float value;
    /* 不依赖缓冲区对齐，也不违反严格别名规则。 */
    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint8_t YB_MRA02_DecodeYaw(const YB_MRA02_Frame *frame, float *yaw_deg)
{
    float yaw;
    if (frame == NULL || yaw_deg == NULL || frame->length != 17U ||
        !valid_frame(frame->bytes, frame->length) || frame->bytes[3] != 0x26U)
        return 0U;
    /* 表格第60~62行：roll/pitch/yaw均为小端float，单位rad。
     * yaw位于完整帧下标12~15；其它功能帧不能刷新航向时间戳。 */
    if (!isfinite(read_float_le(frame->bytes + 4)) ||
        !isfinite(read_float_le(frame->bytes + 8))) return 0U;
    yaw = read_float_le(frame->bytes + 12) * 57.2957795131f;
    if (!isfinite(yaw)) return 0U;
    *yaw_deg = remainderf(yaw, 360.0f);
    return 1U;
}

void YB_MRA02_StreamInit(YB_MRA02_Stream *stream)
{
    memset(stream, 0, sizeof(*stream));
}

static void consume(YB_MRA02_Stream *stream, uint16_t count)
{
    stream->used = (uint16_t)(stream->used - count);
    memmove(stream->bytes, stream->bytes + count, stream->used);
}

void YB_MRA02_StreamPush(YB_MRA02_Stream *stream, uint8_t byte)
{
    if (stream->used == YB_MRA02_STREAM_CAPACITY) {
        consume(stream, 1U);
        ++stream->discarded_bytes;
    }
    stream->bytes[stream->used++] = byte;
}

uint8_t YB_MRA02_StreamNext(YB_MRA02_Stream *stream, YB_MRA02_Frame *out)
{
    out->length = 0U;
    while (stream->used != 0U) {
        if (stream->bytes[0] != 0x7EU) goto discard;
        if (stream->used < 2U) return 0U;
        if (stream->bytes[1] != 0x23U) goto discard;
        if (stream->used < 3U) return 0U;
        uint8_t length = stream->bytes[2];
        if (length < 5U || length > YB_MRA02_MAX_FRAME) goto discard;
        if (stream->used < length) return 0U; /* 半帧留到下一批接收。 */
        if (!valid_frame(stream->bytes, length)) {
            ++stream->checksum_errors;
            goto discard;
        }
        out->length = length;
        memcpy(out->bytes, stream->bytes, length);
        consume(stream, length);
        return 1U;
discard:
        /* 坏帧只丢一个字节，继续寻找下一帧，避免吞掉紧随的正确帧。 */
        consume(stream, 1U);
        ++stream->discarded_bytes;
    }
    return 0U;
}
