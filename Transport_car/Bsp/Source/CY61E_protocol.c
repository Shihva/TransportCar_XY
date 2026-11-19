#include "CY61E_protocol.h"
#include <string.h>

static const uint8_t frame_lengths[] = {30U, 38U, 42U, 44U, 50U, 56U};

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t read_i16(const uint8_t *p)
{
    uint16_t value = read_u16(p);
    return (value & 0x8000U) ? (int32_t)value - 65536L : (int32_t)value;
}

static int32_t read_i32(const uint8_t *p)
{
    uint32_t value = (uint32_t)read_u16(p) | ((uint32_t)read_u16(p + 2) << 16);
    /* 避免未对齐访问、严格别名违规及超范围无符号到有符号转换。 */
    if ((value & 0x80000000UL) != 0U) return -1L - (int32_t)(~value);
    return (int32_t)value;
}

uint16_t CY61E_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;
    for (i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

void CY61E_BuildCommand(uint8_t command, const uint8_t params[4], uint8_t out[8])
{
    uint16_t crc;
    out[0] = 0xFFU;
    out[1] = command;
    memcpy(out + 2, params, 4U);
    crc = CY61E_CRC16(out, 6U);
    out[6] = (uint8_t)crc;
    out[7] = (uint8_t)(crc >> 8);
}

uint8_t CY61E_ValidControl(const uint8_t frame[8])
{
    return (uint8_t)(frame[0] == 0xFFU &&
                     CY61E_CRC16(frame, 6U) == read_u16(frame + 6));
}

static uint8_t valid_telemetry(const uint8_t *frame, uint8_t length)
{
    uint16_t sum = 0U;
    uint8_t i;
    if (frame[0] != 0x80U || frame[1] != 0x01U ||
        frame[length - 2U] != 0x0DU || frame[length - 1U] != 0x0AU) return 0U;
    for (i = 2U; i < length - 4U; ++i) sum = (uint16_t)(sum + frame[i]);
    return (uint8_t)(sum == read_u16(frame + length - 4U));
}

uint8_t CY61E_DecodeTelemetry(const uint8_t *frame, uint8_t length,
                             CY61E_Telemetry *out)
{
    uint8_t i, n, euler, temp, quat = 0U, wide;
    const uint8_t *p;
    if (frame == 0 || out == 0) return 0U;
    for (i = 0U; i < sizeof(frame_lengths); ++i) {
        if (length == frame_lengths[i]) break;
    }
    if (i == sizeof(frame_lengths) || !valid_telemetry(frame, length)) return 0U;
    memset(out, 0, sizeof(*out));
    n = (uint8_t)((length - 10U) / 2U);
    wide = (uint8_t)(n == 16U || n == 20U || n == 23U);
    euler = wide ? 12U : 6U;
    temp = wide ? 15U : 9U;
    if (n == 17U) temp = 13U;
    if (n == 23U) temp = 19U;
    if (n == 14U) quat = 10U;
    if (n == 20U) quat = 16U;
    p = frame + 6;
    for (i = 0U; i < 3U; ++i) {
        if (wide) {
            out->gyro_dps[i] = (float)read_i32(p + 4U * i) / 4194304.0f;
            out->acceleration_mps2[i] = ((float)read_i32(p + 12U + 4U * i) / 65536.0f) * 0.0098f;
        } else {
            out->gyro_dps[i] = (float)read_i16(p + 2U * i) / 64.0f;
            out->acceleration_mps2[i] = (float)read_i16(p + 6U + 2U * i) * 0.0098f;
        }
        out->euler_deg[i] = (float)read_i16(p + 2U * (euler + i)) * 0.01f;
    }
    out->temperature_c = (float)read_i16(p + 2U * temp) * 0.01f;
    if (quat != 0U) {
        for (i = 0U; i < 4U; ++i) out->quaternion_wxyz[i] = (float)read_i16(p + 2U * (quat + i)) / 30000.0f;
        out->quaternion_valid = 1U;
    }
    out->counter = read_u16(frame + 2);
    out->status = read_u16(frame + 4);
    out->payload_words = n;
    memcpy(out->raw_payload, p, (uint16_t)n * 2U);
    return 1U;
}

void CY61E_StreamInit(CY61E_Stream *stream)
{
    memset(stream, 0, sizeof(*stream));
}

static void consume(CY61E_Stream *stream, uint16_t count)
{
    stream->used = (uint16_t)(stream->used - count);
    memmove(stream->bytes, stream->bytes + count, stream->used);
}

void CY61E_StreamPush(CY61E_Stream *stream, uint8_t byte)
{
    if (stream->used == CY61E_STREAM_CAPACITY) {
        consume(stream, 1U);
        stream->discarded_bytes++;
    }
    stream->bytes[stream->used++] = byte;
}

uint8_t CY61E_StreamNext(CY61E_Stream *stream, CY61E_Frame *out,
                        uint8_t force_resync)
{
    uint8_t i, length, bad_sum;
    out->type = CY61E_FRAME_NONE;
    while (stream->used != 0U) {
        length = 0U;
        bad_sum = 0U;
        if (stream->bytes[0] == 0xFFU) {
            if (stream->used < 8U && !force_resync) return 0U;
            if (stream->used >= 8U) {
                if (CY61E_ValidControl(stream->bytes)) {
                    length = 8U;
                    out->type = CY61E_FRAME_CONTROL;
                } else stream->checksum_errors++;
            }
        } else if (stream->bytes[0] == 0x80U) {
            if (stream->used < 2U && !force_resync) return 0U;
            if (stream->used >= 2U && stream->bytes[1] == 0x01U) {
                for (i = 0U; i < sizeof(frame_lengths); ++i) {
                    uint8_t candidate = frame_lengths[i];
                    if (stream->used < candidate) continue;
                    if (valid_telemetry(stream->bytes, candidate)) {
                        length = candidate;
                        out->type = CY61E_FRAME_TELEMETRY;
                        break;
                    }
                    if (stream->bytes[candidate - 2U] == 0x0DU &&
                        stream->bytes[candidate - 1U] == 0x0AU) bad_sum = 1U;
                }
                if (length == 0U && stream->used < CY61E_MAX_FRAME && !force_resync) return 0U;
                if (length == 0U && bad_sum) stream->checksum_errors++;
            }
        }
        if (length != 0U) {
            out->length = length;
            memcpy(out->bytes, stream->bytes, length);
            consume(stream, length);
            return 1U;
        }
        consume(stream, 1U);
        stream->discarded_bytes++;
    }
    return 0U;
}
