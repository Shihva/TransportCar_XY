#ifndef CHASSIS_RUNTIME_H
#define CHASSIS_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_nav.h"

typedef enum {
    CHASSIS_RUNTIME_NOT_READY = 0,
    CHASSIS_RUNTIME_IDLE,
    CHASSIS_RUNTIME_RUNNING,
    CHASSIS_RUNTIME_DONE,
    CHASSIS_RUNTIME_FAULT
} ChassisRuntime_Status_t;

typedef enum {
    CHASSIS_RUNTIME_FAULT_NONE = 0,
    CHASSIS_RUNTIME_FAULT_QUEUE_CREATE,
    CHASSIS_RUNTIME_FAULT_TRANSMIT,
    CHASSIS_RUNTIME_FAULT_NAVIGATION
} ChassisRuntime_Fault_t;

/* 转向采用“位置脉冲粗转 -> PD微调”的连续衔接控制。 */
typedef enum {
    CHASSIS_TURN_PHASE_NONE = 0,
    CHASSIS_TURN_PHASE_COARSE,
    CHASSIS_TURN_PHASE_SETTLE, /* 保留旧调试编号，当前正常流程不再进入。 */
    CHASSIS_TURN_PHASE_FINE
} ChassisRuntime_TurnPhase_t;

/* 电机任务调试快照，可直接加入Keil Watch。 */
typedef struct {
    ChassisRuntime_Status_t status;
    ChassisRuntime_Fault_t fault;
    uint8_t nav_initialized;
    uint8_t encoder_valid_mask;
    uint8_t command_queue_ready;
    uint8_t encoder_query_enabled;
    uint32_t cycle_count;
    uint32_t overrun_count;
    uint32_t bus_time_ms;
    uint32_t active_sequence;
    uint32_t completed_sequence;
    int64_t motor_count[CHASSIS_NAV_WHEEL_COUNT];
    float imu_yaw_deg;
    uint8_t imu_valid;
    ChassisRuntime_TurnPhase_t turn_phase;
    uint8_t coarse_stable_cycles;
    uint8_t coarse_direction;
    uint8_t coarse_handoff_early;
    uint32_t coarse_pulse_count;
    uint32_t coarse_guard_ms;
    uint32_t turn_phase_elapsed_ms;
    float turn_target_yaw_deg;
    float turn_error_deg;
    ChassisNav_Output_t output;
    ChassisNav_State_t nav;
} ChassisRuntime_State_t;

extern ChassisRuntime_State_t g_chassis_runtime;

/* 使用方法：等 GetState 的 nav_initialized 为 1，再投递一次目标。
 * 例如 GotoXYLimited(0, 500, 0, 40, &seq)：以当前地图为准向 Y+ 走到
 * 500 mm、保持 0 度、限速 40 RPM。返回 true 只代表队列收到了命令。
 * 周期读取 GetState，DONE 且 completed_sequence == seq 才代表停稳。
 * 等待期间也要检查 FAULT；下一条独立动作应等本条完成后再发。
 * 上电使用原有 chassis.move；要从定点模式返回手动/视觉速度模式，
 * 先更新 chassis.move 的三个速度，再调用下面的切换函数。 */
bool ChassisRuntime_UseBodyVelocity(void);

/* 在创建任务前调用一次，用于建立主任务到电机任务的命令队列。 */
bool ChassisRuntime_Create(void);

/* 仅由20 ms高优先级电机任务调用。 */
void ChassisRuntime_TaskStep(void);

/* 以下接口由主流程任务调用；命令进入队列后由电机任务执行。 */
bool ChassisRuntime_GotoXY(float x_mm, float y_mm, float yaw_deg,
                           uint32_t *sequence);
/* 保持指定航向全向移动，本次独立限速；用于工位离站。 */
bool ChassisRuntime_GotoXYLimited(float x_mm, float y_mm, float yaw_deg,
                                  float max_rpm, uint32_t *sequence);
/* 独立限速并采用路径衔接容差；用于起步保持航向离站。 */
bool ChassisRuntime_GotoXYLimitedTransit(float x_mm, float y_mm, float yaw_deg,
                                         float max_rpm, uint32_t *sequence);
bool ChassisRuntime_TurnTo(float yaw_deg, uint32_t *sequence);
/* 普通路段快速衔接；工位对接仍使用上面的严格接口。 */
bool ChassisRuntime_GotoXYTransit(float x_mm, float y_mm, float yaw_deg,
                                  uint32_t *sequence);
bool ChassisRuntime_TurnToTransit(float yaw_deg, uint32_t *sequence);
bool ChassisRuntime_CorrectPose(float x_mm, float y_mm, float yaw_deg);
/* 带执行序号的位姿修正：入队不等于生效；初始选区后必须等待此序号完成。 */
bool ChassisRuntime_CorrectPoseWithSequence(float x_mm, float y_mm, float yaw_deg,
                                           uint32_t *sequence);
bool ChassisRuntime_Stop(void);

void ChassisRuntime_GetState(ChassisRuntime_State_t *state);

#endif /* CHASSIS_RUNTIME_H */
