#ifndef CHASSIS_BUS_H
#define CHASSIS_BUS_H

#include <stdbool.h>
#include <stdint.h>

/* 下列函数只由底盘任务调用：查询与运动命令共用 USART1，不能交叉发帧。 */
bool ChassisBus_ReadPosition(uint8_t wheel, uint8_t address, int64_t *count);
bool ChassisBus_StageSpeed(uint8_t address, uint8_t direction, uint16_t rpm);
bool ChassisBus_StageTurn(uint8_t address, uint8_t direction, uint16_t rpm,
                          uint8_t acceleration, uint32_t pulses);
bool ChassisBus_StartTogether(void);
bool ChassisBus_StopAll(void);

/* USART6 使用接收中断；快照包含最近一帧的航向和接收时间。 */
typedef struct {
    float yaw_deg;
    uint32_t last_update_ms;
    uint32_t frame_count;
} ChassisImu_Snapshot;
void ChassisImu_Start(void);
void ChassisImu_Receive(uint16_t size);
void ChassisImu_GetSnapshot(ChassisImu_Snapshot *snapshot);
#endif
