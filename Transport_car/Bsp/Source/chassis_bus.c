#include "chassis_bus.h"
#include "YB_MRA02_protocol.h"
#include "usart.h"
#include "cmsis_os2.h"

#define CHASSIS_QUERY_TIMEOUT_MS 4U

/* Keil Watch：下标按逻辑轮序（左前、右前、左后、右后）。sent 仅表示主控串口完成了发送。
 * last_rx 保存最近一次有数据的查询开头（看 last_len 个字节），
 * 无回包不会擦掉它；是否仍在收数据请比较 rx_bytes 的增量。 */
typedef struct {
    uint32_t sent, tx_errors, rx_bytes, valid_frames, rx_busy;
    uint8_t last_rx[12], last_len;
} ChassisBus_Diag;

volatile ChassisBus_Diag g_chassis_bus_diag[4];

/* 上一次速度命令可能留下确认帧。每次查询前清理旧字节，再按地址、
 * 功能码、符号和帧尾辨认本次回包；缺一轮就用有效位告诉里程计。 */
static void ChassisRuntime_DrainRx(void)
{
    volatile uint32_t dummy;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
        dummy = huart1.Instance->DR;
        (void)dummy;
    }
}

/* 读取一个驱动器的0x36有符号累计位置应答。 */
bool ChassisBus_ReadPosition(uint8_t wheel, uint8_t address, int64_t *count)
{
    uint8_t query[3] = {address, 0x36U, 0x6BU};
    uint8_t frame[8];
    uint8_t byte;
    uint8_t index = 0U;
    uint32_t start_ms;

    if (count == NULL || wheel >= 4U) return 0U;
    volatile ChassisBus_Diag *diag = &g_chassis_bus_diag[wheel];
    uint8_t captured = 0U;

    ChassisRuntime_DrainRx();
    if (HAL_UART_Transmit(&huart1, query, sizeof(query), 2U) != HAL_OK) {
        ++diag->tx_errors;
        return 0U;
    }
    ++diag->sent;

    start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_ms) < CHASSIS_QUERY_TIMEOUT_MS) {
        HAL_StatusTypeDef rx = HAL_UART_Receive(&huart1, &byte, 1U, 1U);
        if (rx == HAL_BUSY) {
            ++diag->rx_busy;
            return 0U;
        }
        if (rx != HAL_OK) continue;
        ++diag->rx_bytes;
        if (captured < sizeof(diag->last_rx)) {
            diag->last_rx[captured++] = byte;
            diag->last_len = captured;
        }

        if (index == 0U) {
            if (byte == address) frame[index++] = byte;
            continue;
        }
        if (index == 1U) {
            if (byte == 0x36U) {
                frame[index++] = byte;
            } else {
                index = (byte == address) ? 1U : 0U;
            }
            continue;
        }

        frame[index++] = byte;
        if (index == sizeof(frame)) {
            uint32_t magnitude;
            if (frame[7] != 0x6BU || frame[2] > 1U) {
                index = 0U;
                continue;
            }
            magnitude = ((uint32_t)frame[3] << 24) |
                        ((uint32_t)frame[4] << 16) |
                        ((uint32_t)frame[5] << 8)  |
                        ((uint32_t)frame[6]);
            *count = (frame[2] != 0U) ?
                     -(int64_t)magnitude : (int64_t)magnitude;
            ++diag->valid_frames;
            osDelay(1U);
            return 1U;
        }
    }
    return 0U;
}


/* 所有控制帧都阻塞到发送结束，局部缓冲区可以随函数返回释放。
 * 先给每台驱动写“等待同步”的目标，整组成功后才广播启动。 */
static bool SendFrame(uint8_t *bytes, uint16_t length)
{
    return HAL_UART_Transmit(&huart1, bytes, length, 5U) == HAL_OK;
}

bool ChassisBus_StageSpeed(uint8_t address, uint8_t direction, uint16_t rpm)
{
    uint8_t frame[] = {address, 0xF6, direction, rpm >> 8, (uint8_t)rpm, 0, 1, 0x6B};
    return SendFrame(frame, sizeof(frame));
}

bool ChassisBus_StageTurn(uint8_t address, uint8_t direction, uint16_t rpm,
                          uint8_t acceleration, uint32_t pulses)
{
    /* Emm5.0：高字节在前，位置模式 0 表示相对本次起点移动。 */
    uint8_t frame[] = {address, 0xFD, direction, rpm >> 8, (uint8_t)rpm, acceleration,
                       pulses >> 24, (uint8_t)(pulses >> 16), (uint8_t)(pulses >> 8),
                       (uint8_t)pulses, 0, 1, 0x6B};
    return SendFrame(frame, sizeof(frame));
}

bool ChassisBus_StartTogether(void)
{
    uint8_t frame[] = {0, 0xFF, 0x66, 0x6B};
    return SendFrame(frame, sizeof(frame));
}

bool ChassisBus_StopAll(void)
{
    uint8_t frame[] = {0, 0xFE, 0x98, 0, 0x6B};
    return SendFrame(frame, sizeof(frame));
}

/* Transport_car 的 USART6 已配置中断。用 IDLE/收满中断接收连续字节，
 * 残缺帧留给下一批；只有通过协议校验的遥测帧才能刷新航向时间戳。 */
static YB_MRA02_Stream imu_stream;
static ChassisImu_Snapshot imu_state;
static uint8_t imu_rx[2][YB_MRA02_MAX_FRAME];
static uint8_t imu_write_index;

void ChassisImu_Start(void)
{
    YB_MRA02_StreamInit(&imu_stream);
    imu_write_index = 0U;
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart6, imu_rx[0], YB_MRA02_MAX_FRAME);
}

void ChassisImu_Receive(uint16_t size)
{
    YB_MRA02_Frame frame;
    float yaw_deg;
    uint8_t *received = imu_rx[imu_write_index];
    imu_write_index ^= 1U;
    /* 先挂下一块缓冲，再解析刚收到的一块。 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart6, imu_rx[imu_write_index], YB_MRA02_MAX_FRAME);
    for (uint16_t i = 0; i < size && i < YB_MRA02_MAX_FRAME; ++i) {
        YB_MRA02_StreamPush(&imu_stream, received[i]);
        while (YB_MRA02_StreamNext(&imu_stream, &frame)) {
            if (YB_MRA02_DecodeYaw(&frame, &yaw_deg)) {
                imu_state.yaw_deg = yaw_deg;
                imu_state.last_update_ms = HAL_GetTick();
                ++imu_state.frame_count;
            }
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart6) {
        HAL_UART_AbortReceive(uart);
        ChassisImu_Start();
    }
}

void ChassisImu_GetSnapshot(ChassisImu_Snapshot *snapshot)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *snapshot = imu_state;
    __set_PRIMASK(mask);
}
