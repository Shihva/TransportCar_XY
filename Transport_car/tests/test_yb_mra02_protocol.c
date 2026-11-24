/* 可用宿主C编译器运行；main返回0代表全部通过，否则返回失败行号。
 * cc -std=c11 -Wall -Wextra -Werror -I Bsp/Include tests/test_yb_mra02_protocol.c
 *    Bsp/Source/YB_MRA02_protocol.c -lm -o test_imu
 */
#include "YB_MRA02_protocol.h"
#include <math.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static void checksum(YB_MRA02_Frame *frame)
{
    uint8_t sum = 0U;
    for (unsigned i = 0; i < frame->length - 1U; ++i)
        sum = (uint8_t)(sum + frame->bytes[i]);
    frame->bytes[frame->length - 1U] = sum;
}

static YB_MRA02_Frame euler(void)
{
    /* 小端float: roll=0, pitch=0, yaw=pi/2 rad。 */
    YB_MRA02_Frame frame = {17U, {
        0x7E, 0x23, 0x11, 0x26, 0, 0, 0, 0, 0, 0, 0, 0,
        0xDB, 0x0F, 0xC9, 0x3F, 0
    }};
    checksum(&frame);
    return frame;
}

static unsigned feed(YB_MRA02_Stream *stream, const uint8_t *data,
                     unsigned length, float *yaw)
{
    YB_MRA02_Frame frame;
    unsigned count = 0;
    for (unsigned i = 0; i < length; ++i) {
        YB_MRA02_StreamPush(stream, data[i]);
        while (YB_MRA02_StreamNext(stream, &frame))
            count += YB_MRA02_DecodeYaw(&frame, yaw);
    }
    return count;
}

int main(void)
{
    YB_MRA02_Stream stream;
    YB_MRA02_Frame frame = euler(), decoded;
    float yaw = 0.0f;
    CHECK(YB_MRA02_DecodeYaw(&frame, &yaw));
    CHECK(fabsf(yaw - 90.0f) < 0.001f);
    frame.bytes[15] = 0xBF; /* -pi/2 */
    checksum(&frame);
    CHECK(YB_MRA02_DecodeYaw(&frame, &yaw));
    CHECK(fabsf(yaw + 90.0f) < 0.001f);
    frame = euler();

    /* 每个可能的接收分割点都不能提前产出半帧。 */
    for (unsigned split = 1; split < frame.length; ++split) {
        YB_MRA02_StreamInit(&stream);
        CHECK(feed(&stream, frame.bytes, split, &yaw) == 0U);
        CHECK(feed(&stream, frame.bytes + split, frame.length - split, &yaw) == 1U);
        CHECK(fabsf(yaw - 90.0f) < 0.001f && stream.used == 0U);
    }

    /* 粘包：最长原始数据、四元数、状态回复与欧拉角交错出现。 */
    YB_MRA02_StreamInit(&stream);
    const uint8_t lengths[] = {23, 21, 7, 21};
    const uint8_t functions[] = {0x04, 0x16, 0x81, 0x32};
    for (unsigned i = 0; i < 4; ++i) {
        YB_MRA02_Frame other = {0};
        other.length = lengths[i];
        other.bytes[0] = 0x7E; other.bytes[1] = 0x23;
        other.bytes[2] = lengths[i]; other.bytes[3] = functions[i];
        checksum(&other);
        yaw = 123.0f;
        CHECK(feed(&stream, other.bytes, other.length, &yaw) == 0U);
        CHECK(yaw == 123.0f);
        CHECK(feed(&stream, frame.bytes, frame.length, &yaw) == 1U);
    }

    /* 两帧一起缓存时，Next须能连续取出。 */
    YB_MRA02_StreamInit(&stream);
    for (unsigned n = 0; n < 2; ++n)
        for (unsigned i = 0; i < frame.length; ++i)
            YB_MRA02_StreamPush(&stream, frame.bytes[i]);
    CHECK(YB_MRA02_StreamNext(&stream, &decoded));
    CHECK(YB_MRA02_StreamNext(&stream, &decoded));
    CHECK(!YB_MRA02_StreamNext(&stream, &decoded));

    /* 坏校验、噪声和非法长度后，仍可接收紧随的正确帧。 */
    YB_MRA02_StreamInit(&stream);
    frame.bytes[16] ^= 1U;
    CHECK(feed(&stream, frame.bytes, frame.length, &yaw) == 0U);
    CHECK(stream.checksum_errors == 1U);
    frame = euler();
    const uint8_t noise[] = {0x55, 0x7E, 0x7E, 0x23, 0xFF,
                             0x7E, 0x23, 0x00, 0x7E, 0x23, 0x04};
    CHECK(feed(&stream, noise, sizeof(noise), &yaw) == 0U);
    CHECK(feed(&stream, frame.bytes, frame.length, &yaw) == 1U);
    /* 损坏成合法长度的头部也需最终重新同步。 */
    const uint8_t truncated[] = {0x7E, 0x23, 23, 0x26};
    CHECK(feed(&stream, truncated, sizeof(truncated), &yaw) == 0U);
    unsigned count = feed(&stream, frame.bytes, frame.length, &yaw);
    count += feed(&stream, frame.bytes, frame.length, &yaw);
    CHECK(count == 2U);

    /* 校验正确的NaN/Inf，以及转为度时溢出的值也必须拒绝。 */
    const uint8_t invalid[][4] = {
        {0x00, 0x00, 0xC0, 0x7F}, {0x00, 0x00, 0x80, 0x7F},
        {0xFF, 0xFF, 0x7F, 0x7F}
    };
    for (unsigned i = 0; i < 3; ++i) {
        frame = euler();
        memcpy(frame.bytes + 12, invalid[i], 4);
        checksum(&frame);
        yaw = 123.0f;
        CHECK(!YB_MRA02_DecodeYaw(&frame, &yaw));
        CHECK(yaw == 123.0f);
    }
    frame = euler();
    memcpy(frame.bytes + 4, invalid[0], 4);
    checksum(&frame);
    CHECK(!YB_MRA02_DecodeYaw(&frame, &yaw));
    frame = euler(); frame.length = 16;
    CHECK(!YB_MRA02_DecodeYaw(&frame, &yaw));
    CHECK(!YB_MRA02_DecodeYaw(NULL, &yaw));

    /* 缓冲区溢出后丢弃旧字节，仍能恢复；复位清除残帧。 */
    YB_MRA02_StreamInit(&stream);
    for (unsigned i = 0; i < 100; ++i) YB_MRA02_StreamPush(&stream, 0xAA);
    CHECK(stream.used == YB_MRA02_STREAM_CAPACITY);
    CHECK(stream.discarded_bytes == 100U - YB_MRA02_STREAM_CAPACITY);
    frame = euler();
    CHECK(feed(&stream, frame.bytes, frame.length, &yaw) == 1U);
    YB_MRA02_StreamInit(&stream);
    CHECK(stream.used == 0 && stream.checksum_errors == 0);
    return 0;
}
