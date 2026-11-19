#ifndef CY61E_PROTOCOL_H
#define CY61E_PROTOCOL_H

#include <stdint.h>

#define CY61E_MAX_FRAME 56U
#define CY61E_MAX_PAYLOAD 46U
#define CY61E_STREAM_CAPACITY 112U

#define CY61E_STATUS_ACC_READY       0x0001U
#define CY61E_STATUS_GYRO_READY      0x0002U
#define CY61E_STATUS_CALIBRATING     0x0004U
#define CY61E_STATUS_CALIBRATION_OK  0x0008U
#define CY61E_STATUS_GX_OVER         0x0010U
#define CY61E_STATUS_GY_OVER         0x0020U
#define CY61E_STATUS_GZ_OVER         0x0040U
#define CY61E_STATUS_CAL_REQUEST     0x0080U
#define CY61E_STATUS_CALIBRATING2    0x0100U
#define CY61E_STATUS_CALIBRATION2_OK 0x0200U

/* 完整数据快照；XYZ 沿用模块轴定义，不擅自推断欧拉角旋转顺序。 */
typedef struct {
    float gyro_dps[3];
    float acceleration_mps2[3];
    float euler_deg[3];
    float temperature_c;
    float quaternion_wxyz[4];
    uint16_t counter;
    uint16_t status;
    uint8_t quaternion_valid;
    uint8_t payload_words;
    uint8_t raw_payload[CY61E_MAX_PAYLOAD]; /* 包括扩展包中的未定义字段 */
    uint32_t received_ms; /* 主机收到并解析的时间，不是设备 UTC */
} CY61E_Telemetry;

typedef enum {
    CY61E_FRAME_NONE = 0,
    CY61E_FRAME_TELEMETRY,
    CY61E_FRAME_CONTROL
} CY61E_FrameType;

typedef struct {
    CY61E_FrameType type;
    uint8_t length;
    uint8_t bytes[CY61E_MAX_FRAME];
} CY61E_Frame;

typedef struct {
    uint8_t bytes[CY61E_STREAM_CAPACITY];
    uint16_t used;
    uint32_t checksum_errors;
    uint32_t discarded_bytes;
} CY61E_Stream;

/* CRC-16: 初值 FFFF，多项式 A001，低字节先发。 */
uint16_t CY61E_CRC16(const uint8_t *data, uint16_t length);
void CY61E_BuildCommand(uint8_t command, const uint8_t params[4], uint8_t out[8]);
uint8_t CY61E_ValidControl(const uint8_t frame[8]);
uint8_t CY61E_DecodeTelemetry(const uint8_t *frame, uint8_t length,
                             CY61E_Telemetry *out);
void CY61E_StreamInit(CY61E_Stream *stream);
void CY61E_StreamPush(CY61E_Stream *stream, uint8_t byte);
/* force_resync 仅在接收长时间无字节时使用，释放错误短包后的有效尾随帧。 */
uint8_t CY61E_StreamNext(CY61E_Stream *stream, CY61E_Frame *out,
                        uint8_t force_resync);

#endif
