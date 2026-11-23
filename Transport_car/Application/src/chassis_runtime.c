#include "chassis_runtime.h"
#include "chassis_config.h"
#include "chassis_bus.h"
#include "Chassis.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "queue.h"
#include "task.h"
#include "usart.h"

#include <math.h>
#include <string.h>

#define CHASSIS_MOTOR_COUNT                 4U
#define CHASSIS_INTERFRAME_GUARD_MS         1U
#define CHASSIS_IMU_TIMEOUT_MS            200U
#define CHASSIS_COMMAND_QUEUE_LENGTH        4U
#define CHASSIS_EXPECTED_PERIOD_MS CHASSIS_PERIOD_MS

/* 粗转先完成大部分角度，余量留给反馈控制收尾。 */
#define CHASSIS_COARSE_RESERVE_DEG         1.0f
#define CHASSIS_COARSE_ENTER_DEG            8.0f
#define CHASSIS_COARSE_SPEED_RPM          120U
#define CHASSIS_COARSE_ACCELERATION       160U
/*
 * 位置脉冲下发后先留出短暂启动保护，期间禁止速度/停止命令覆盖它。
 * 保护时间结束后还必须满足“四轮反馈有效且角速度稳定”才能交给PD，
 * 因此无需使用很长的固定延时。
 */
#define CHASSIS_COARSE_GUARD_BASE_MS       200U
#define CHASSIS_COARSE_GUARD_PER_1K_MS      50U
#define CHASSIS_COARSE_TIMEOUT_MARGIN_MS  1500U
#define CHASSIS_COARSE_PD_HANDOFF_DEG       1.5f
#define CHASSIS_COARSE_STOP_RATE_DEG_S      4.0f
#define CHASSIS_COARSE_STABLE_CYCLES        2U

typedef enum {
    CHASSIS_COMMAND_GOTO_XY = 0,
    CHASSIS_COMMAND_TURN_TO,
    CHASSIS_COMMAND_CORRECT_POSE,
    CHASSIS_COMMAND_BODY_VELOCITY,
    CHASSIS_COMMAND_STOP,
    CHASSIS_COMMAND_GOTO_XY_LIMITED
} ChassisCommand_Type_t;

typedef struct {
    ChassisCommand_Type_t type;
    uint32_t sequence;
    float x_mm;
    float y_mm;
    float yaw_deg;
    float max_rpm;
    uint8_t transit_mode;
} ChassisCommand_t;

/* 一份地址表同时用于回读和发令，避免反馈和输出对应到不同轮子。 */
static const uint8_t s_motor_address[CHASSIS_MOTOR_COUNT] = CHASSIS_MOTOR_ADDRESS_LIST;
static QueueHandle_t s_command_queue;
static uint32_t s_next_sequence;
static uint32_t s_last_sample_ms;
static uint8_t s_stop_sent;
static uint32_t s_turn_phase_start_ms;
static uint32_t s_coarse_guard_ms;
static uint8_t s_turn_transit_mode;


ChassisRuntime_State_t g_chassis_runtime; /* 调试器可直接查看。 */
static ChassisRuntime_State_t s_published; /* 给其他任务的一整个已完成周期。 */
static bool s_body_velocity = true;        /* 兼容原有 chassis.move 速度入口。 */

static float ChassisRuntime_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float ChassisRuntime_WrapDeg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg <= -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static void ChassisRuntime_ZeroWheelOutput(void)
{
    uint8_t index;
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; index++) {
        g_chassis_runtime.output.wheel_rpm[index] = 0.0f;
    }
    g_chassis_runtime.output.status = CHASSIS_NAV_RUNNING;
}

static uint8_t ChassisRuntime_ReadAllPositions(void)
{
    uint8_t valid_mask = 0U;
    uint8_t index;

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index) {
        int64_t count;
        if (ChassisBus_ReadPosition(index, s_motor_address[index], &count) != 0U) {
            g_chassis_runtime.motor_count[index] = count;
            valid_mask |= (uint8_t)(1U << index);
        }
    }
    return valid_mask;
}

static bool ChassisRuntime_SendOneRpm(uint8_t address, float rpm)
{
    uint8_t direction;
    uint16_t magnitude;

    rpm = ChassisRuntime_Clamp(rpm, -65535.0f, 65535.0f);
    direction = (rpm >= 0.0f) ? 1U : 0U;
    magnitude = (uint16_t)(fabsf(rpm) + 0.5f);
    if (!ChassisBus_StageSpeed(address, direction, magnitude)) return false;
    osDelay(CHASSIS_INTERFRAME_GUARD_MS);
    return true;
}

static bool ChassisRuntime_SendWheelRpm(const float wheel_rpm[CHASSIS_MOTOR_COUNT])
{
    uint8_t index;
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index) {
        if (!ChassisRuntime_SendOneRpm(s_motor_address[index], wheel_rpm[index])) return false;
    }
    return ChassisBus_StartTogether();
}

static bool ChassisRuntime_SendCoarseTurn(float heading_error_deg)/* 四轮装载相同相对脉冲并同步启动，形成底盘原地旋转。 */
{
    float coarse_angle_deg = fabsf(heading_error_deg) -
                             CHASSIS_COARSE_RESERVE_DEG;
    uint8_t direction = (heading_error_deg >= 0.0f) ? 0U : 1U;
    uint32_t pulse_count;
    uint8_t index;

    if (coarse_angle_deg < 0.0f) coarse_angle_deg = 0.0f;
    pulse_count = (uint32_t)(coarse_angle_deg *
                             CHASSIS_COARSE_PULSES_PER_DEG + 0.5f);

    g_chassis_runtime.coarse_direction = direction;
    g_chassis_runtime.coarse_pulse_count = pulse_count;
    g_chassis_runtime.coarse_handoff_early = 0U;
    s_coarse_guard_ms = CHASSIS_COARSE_GUARD_BASE_MS +
        (pulse_count * CHASSIS_COARSE_GUARD_PER_1K_MS) / 1000U;
    g_chassis_runtime.coarse_guard_ms = s_coarse_guard_ms;
    g_chassis_runtime.coarse_stable_cycles = 0U;
    g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_COARSE;
    s_turn_phase_start_ms = HAL_GetTick();
    ChassisNav_SetEncoderFaultSuppressed(true);

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; index++) {
        if (!ChassisBus_StageTurn(s_motor_address[index], direction,
                                  CHASSIS_COARSE_SPEED_RPM, CHASSIS_COARSE_ACCELERATION,
                                  pulse_count)) return false;
        osDelay(CHASSIS_INTERFRAME_GUARD_MS);
    }
    return ChassisBus_StartTogether();
}

static void ChassisRuntime_SendStopOnce(void)
{
    if (s_stop_sent == 0U) {
        s_stop_sent = ChassisBus_StopAll() ? 1U : 0U;
    }
}

/* 四轮中任何一帧未发完，本组不再同步启动；停车失败则下个周期重试。 */
static void ChassisRuntime_BusFault(void)
{
    g_chassis_runtime.fault = CHASSIS_RUNTIME_FAULT_TRANSMIT;
    g_chassis_runtime.status = CHASSIS_RUNTIME_FAULT;
    s_stop_sent = 0U;
    ChassisRuntime_SendStopOnce();
}

static uint32_t ChassisRuntime_AllocateSequence(void)
{
    uint32_t sequence;
    taskENTER_CRITICAL();
    s_next_sequence++;
    if (s_next_sequence == 0U) s_next_sequence = 1U;
    sequence = s_next_sequence;
    taskEXIT_CRITICAL();
    return sequence;
}

static bool ChassisRuntime_QueueCommand(ChassisCommand_t *command,
                                        uint32_t *sequence)
{
    if (command == NULL || s_command_queue == NULL) return false;
    taskENTER_CRITICAL();
    bool ready = s_published.nav_initialized && s_published.status != CHASSIS_RUNTIME_FAULT;
    taskEXIT_CRITICAL();
    if (!ready) return false;
    command->sequence = ChassisRuntime_AllocateSequence();
    if (xQueueSend(s_command_queue, command, 0U) != pdPASS) return false;
    if (sequence != NULL) *sequence = command->sequence;
    return true;
}

static void ChassisRuntime_ExecuteCommand(const ChassisCommand_t *command,
                                          const ChassisNav_Feedback_t *feedback)
{
    bool accepted = false;

    if (command == NULL) return;
    s_body_velocity = (command->type == CHASSIS_COMMAND_BODY_VELOCITY);
    switch (command->type) {
    case CHASSIS_COMMAND_GOTO_XY_LIMITED:
        ChassisNav_SetEncoderFaultSuppressed(false);
        g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_NONE;
        accepted = ChassisNav_GotoXYLimited(command->x_mm, command->y_mm,
                                            command->yaw_deg, command->max_rpm);
        break;
    case CHASSIS_COMMAND_GOTO_XY:
        ChassisNav_SetEncoderFaultSuppressed(false);
        g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_NONE;
        accepted = ChassisNav_GotoXY(command->x_mm, command->y_mm,
                                     command->yaw_deg);
        break;
    case CHASSIS_COMMAND_TURN_TO:
        s_turn_transit_mode = command->transit_mode;
        accepted = ChassisNav_TurnTo(command->yaw_deg);
        if (accepted) {
            float heading_error_deg = ChassisRuntime_WrapDeg(
                command->yaw_deg - g_chassis_runtime.nav.pose.yaw_deg);
            g_chassis_runtime.turn_target_yaw_deg = command->yaw_deg;
            g_chassis_runtime.turn_error_deg = heading_error_deg;
            if (fabsf(heading_error_deg) > CHASSIS_COARSE_ENTER_DEG) {
                if (!ChassisRuntime_SendCoarseTurn(heading_error_deg)) {
                    ChassisRuntime_BusFault();
                    return;
                }
            } else {
                g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_FINE;
                s_turn_phase_start_ms = HAL_GetTick();
            }
        }
        break;
    case CHASSIS_COMMAND_CORRECT_POSE:
        ChassisNav_SetEncoderFaultSuppressed(false);
        g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_NONE;
        ChassisNav_CorrectPose(command->x_mm, command->y_mm,
                               command->yaw_deg, feedback->imu_yaw_deg,
                               feedback->imu_valid);
        accepted = true;
        break;
    case CHASSIS_COMMAND_STOP:
        s_stop_sent = 0U;
        ChassisRuntime_SendStopOnce();
        if (!s_stop_sent) {
            ChassisRuntime_BusFault();
            return;
        }
        /* fall through: clear the previous target in both modes */
    case CHASSIS_COMMAND_BODY_VELOCITY:
        ChassisNav_SetEncoderFaultSuppressed(false);
        g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_NONE;
        ChassisNav_Stop();
        accepted = true;
        break;
    default:
        break;
    }

    if (accepted) {
        g_chassis_runtime.active_sequence = command->sequence;
        s_stop_sent = 0U;
        if (command->type == CHASSIS_COMMAND_GOTO_XY ||
            command->type == CHASSIS_COMMAND_GOTO_XY_LIMITED ||
            command->type == CHASSIS_COMMAND_TURN_TO) {
            ChassisNav_SetTransitMode(command->transit_mode != 0U);
            g_chassis_runtime.status = CHASSIS_RUNTIME_RUNNING;
        } else {
            g_chassis_runtime.completed_sequence = command->sequence;
            g_chassis_runtime.status = CHASSIS_RUNTIME_IDLE;
        }
    }
}

bool ChassisRuntime_Create(void)
{
    memset(&g_chassis_runtime, 0, sizeof(g_chassis_runtime));
    s_next_sequence = 0U;
    s_last_sample_ms = 0U;
    s_stop_sent = 0U;
    s_turn_phase_start_ms = 0U;
    s_coarse_guard_ms = 0U;
    s_turn_transit_mode = 0U;
    g_chassis_runtime.encoder_query_enabled = 1U;
    s_command_queue = xQueueCreate(CHASSIS_COMMAND_QUEUE_LENGTH,
                                   sizeof(ChassisCommand_t));
    if (s_command_queue == NULL) {
        g_chassis_runtime.fault = CHASSIS_RUNTIME_FAULT_QUEUE_CREATE;
        g_chassis_runtime.status = CHASSIS_RUNTIME_FAULT;
        return false;
    }
    g_chassis_runtime.command_queue_ready = 1U;
    g_chassis_runtime.status = CHASSIS_RUNTIME_NOT_READY;
    return true;
}

static void ChassisRuntime_RunCycle(void)
{
    ChassisNav_Feedback_t feedback;
    ChassisCommand_t command;
    ChassisImu_Snapshot imu;
    uint32_t cycle_start_ms = HAL_GetTick();
    uint32_t sample_ms;
    float dt_s;

    if (g_chassis_runtime.status == CHASSIS_RUNTIME_FAULT) {
        ChassisRuntime_SendStopOnce();
        return;
    }

    memset(&feedback, 0, sizeof(feedback));
    g_chassis_runtime.encoder_valid_mask = ChassisRuntime_ReadAllPositions();
    memcpy(feedback.motor_count, g_chassis_runtime.motor_count,
           sizeof(feedback.motor_count));
    feedback.encoder_valid_mask = g_chassis_runtime.encoder_valid_mask;

    ChassisImu_GetSnapshot(&imu);
    sample_ms = HAL_GetTick();
    feedback.imu_yaw_deg = imu.yaw_deg;
    feedback.imu_valid = (imu.frame_count != 0U &&
        (uint32_t)(sample_ms - imu.last_update_ms) <= CHASSIS_IMU_TIMEOUT_MS)
        ? 1U : 0U;
    g_chassis_runtime.imu_yaw_deg = feedback.imu_yaw_deg;
    g_chassis_runtime.imu_valid = feedback.imu_valid;

    if (g_chassis_runtime.nav_initialized == 0U) {
        if (feedback.encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID) {
            ChassisNav_Config_t config;
            ChassisNav_GetDefaultConfig(&config);
            if (ChassisNav_Init(&config, &feedback)) {
                g_chassis_runtime.nav_initialized = 1U;
                g_chassis_runtime.status = CHASSIS_RUNTIME_IDLE;
                s_last_sample_ms = sample_ms;
            }
        }
        ChassisRuntime_SendStopOnce();
        if (!s_stop_sent) ChassisRuntime_BusFault();
        g_chassis_runtime.bus_time_ms = HAL_GetTick() - cycle_start_ms;
        g_chassis_runtime.cycle_count++;
        return;
    }

    if (xQueueReceive(s_command_queue, &command, 0U) == pdPASS) {
        ChassisRuntime_ExecuteCommand(&command, &feedback);
        if (g_chassis_runtime.status == CHASSIS_RUNTIME_FAULT) return;
    }

    dt_s = (float)(sample_ms - s_last_sample_ms) * 0.001f;
    s_last_sample_ms = sample_ms;
    if (dt_s <= 0.0f) dt_s = (float)CHASSIS_EXPECTED_PERIOD_MS * 0.001f;

    ChassisNav_Update(&feedback, dt_s, &g_chassis_runtime.output);
    ChassisNav_GetState(&g_chassis_runtime.nav);

    g_chassis_runtime.turn_error_deg = ChassisRuntime_WrapDeg(
        g_chassis_runtime.turn_target_yaw_deg -
        g_chassis_runtime.nav.pose.yaw_deg);
    g_chassis_runtime.turn_phase_elapsed_ms =
        sample_ms - s_turn_phase_start_ms;

    if (g_chassis_runtime.output.status == CHASSIS_NAV_FAULT) {
        ChassisNav_SetEncoderFaultSuppressed(false);
        g_chassis_runtime.fault = CHASSIS_RUNTIME_FAULT_NAVIGATION;
        g_chassis_runtime.status = CHASSIS_RUNTIME_FAULT;
        g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_NONE;
        ChassisRuntime_SendStopOnce();
    } else if (g_chassis_runtime.turn_phase ==
               CHASSIS_TURN_PHASE_COARSE) {
        uint32_t elapsed_ms = sample_ms - s_turn_phase_start_ms;
        uint8_t feedback_ready =
            (g_chassis_runtime.encoder_valid_mask ==
             CHASSIS_NAV_ENCODER_ALL_VALID) ? 1U : 0U;
        uint8_t early_handoff =
            (elapsed_ms >= s_coarse_guard_ms &&
             feedback_ready != 0U &&
             fabsf(g_chassis_runtime.turn_error_deg) <=
                 CHASSIS_COARSE_PD_HANDOFF_DEG) ? 1U : 0U;

        /* 粗转期间只回读编码器和IMU，禁止速度模式覆盖位置脉冲命令。 */
        ChassisRuntime_ZeroWheelOutput();
        g_chassis_runtime.status = CHASSIS_RUNTIME_RUNNING;

        if (elapsed_ms >= s_coarse_guard_ms &&
            feedback_ready != 0U &&
            fabsf(g_chassis_runtime.nav.velocity.wz_deg_s) <=
                CHASSIS_COARSE_STOP_RATE_DEG_S) {
            if (g_chassis_runtime.coarse_stable_cycles < 255U) {
                g_chassis_runtime.coarse_stable_cycles++;
            }
        } else {
            g_chassis_runtime.coarse_stable_cycles = 0U;
        }

        if (early_handoff != 0U ||
            g_chassis_runtime.coarse_stable_cycles >=
                CHASSIS_COARSE_STABLE_CYCLES ||
            elapsed_ms >= (s_coarse_guard_ms +
                           CHASSIS_COARSE_TIMEOUT_MARGIN_MS)) {
            /*
             * 进入目标前1.5度时允许PD接管，以速度命令主动覆盖剩余的
             * 少量位置脉冲，实现连续减速；如果未触发提前接管，则仍可在
             * 正常停稳后切换。只有超时且未停稳时才补发停止命令。
             */
            g_chassis_runtime.coarse_handoff_early = early_handoff;
            if (early_handoff == 0U &&
                g_chassis_runtime.coarse_stable_cycles <
                CHASSIS_COARSE_STABLE_CYCLES) {
                ChassisRuntime_SendStopOnce();
            }

            if (!ChassisNav_TurnTo(
                    g_chassis_runtime.turn_target_yaw_deg)) {
                ChassisNav_SetEncoderFaultSuppressed(false);
                g_chassis_runtime.fault =
                    CHASSIS_RUNTIME_FAULT_NAVIGATION;
                g_chassis_runtime.status = CHASSIS_RUNTIME_FAULT;
                g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_NONE;
            } else {
                /* 粗转重新设置目标会恢复严格模式，需继承本次路径衔接属性。 */
                ChassisNav_SetTransitMode(s_turn_transit_mode != 0U);
                ChassisNav_SetEncoderFaultSuppressed(false);
                g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_FINE;
                s_turn_phase_start_ms = sample_ms;
                s_stop_sent = 0U;
            }
        }
    } else if (g_chassis_runtime.output.status == CHASSIS_NAV_DONE) {
        ChassisRuntime_SendStopOnce();
        if (!s_stop_sent) {
            ChassisRuntime_BusFault();
            return;
        }
        g_chassis_runtime.completed_sequence =
            g_chassis_runtime.active_sequence;
        g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
        g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_NONE;
    } else {
        /* 原项目的视觉/手动速度沿用前进 vx、左移 vy (m/s)、旋转 vw (rad/s)。
         * 定点或转向命令接管后，只有显式切回速度模式才再读取这些值。 */
        if (s_body_velocity && feedback.encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID) {
            Chassis_GetBodyWheelRpm(g_chassis_runtime.output.wheel_rpm);
        }
        s_stop_sent = 0U; /* 下面会重新发速度帧，故障时需要再发停止帧。 */
        if (!ChassisRuntime_SendWheelRpm(g_chassis_runtime.output.wheel_rpm)) {
            ChassisRuntime_BusFault();
            return;
        }
        if (g_chassis_runtime.output.status == CHASSIS_NAV_RUNNING) {
            g_chassis_runtime.status = CHASSIS_RUNTIME_RUNNING;
        } else if (g_chassis_runtime.status != CHASSIS_RUNTIME_RUNNING) {
            g_chassis_runtime.status = CHASSIS_RUNTIME_IDLE;
        }
    }

    g_chassis_runtime.bus_time_ms = HAL_GetTick() - cycle_start_ms;
    if (g_chassis_runtime.bus_time_ms > CHASSIS_EXPECTED_PERIOD_MS) {
        g_chassis_runtime.overrun_count++;
    }
    g_chassis_runtime.cycle_count++;
}

void ChassisRuntime_TaskStep(void)
{
    ChassisRuntime_RunCycle();
    taskENTER_CRITICAL();
    s_published = g_chassis_runtime;
    taskEXIT_CRITICAL();
}

#ifndef CHASSIS_OFFLINE_30CM
void ChassisRuntime_EndWheelTest(void)
{
    /* 仅在诊断已发出停车命令并等待停止之后调用。
     * 丢弃诊断期间的位移和待处理命令，避免恢复后意外执行旧目标。 */
    xQueueReset(s_command_queue);
    chassis.move.vx = 0.0f;
    chassis.move.vy = 0.0f;
    chassis.move.vw = 0.0f;
    s_body_velocity = true;
    s_last_sample_ms = 0U;
    s_stop_sent = 0U;
    g_chassis_runtime.nav_initialized = 0U;
    g_chassis_runtime.status = CHASSIS_RUNTIME_NOT_READY;
    g_chassis_runtime.fault = CHASSIS_RUNTIME_FAULT_NONE;
    g_chassis_runtime.turn_phase = CHASSIS_TURN_PHASE_NONE;
    memset(&g_chassis_runtime.output, 0, sizeof(g_chassis_runtime.output));
    memset(&g_chassis_runtime.nav, 0, sizeof(g_chassis_runtime.nav));
    taskENTER_CRITICAL();
    s_published = g_chassis_runtime;
    taskEXIT_CRITICAL();
}
#endif

bool ChassisRuntime_UseBodyVelocity(void)
{
    ChassisCommand_t command = {0};
    command.type = CHASSIS_COMMAND_BODY_VELOCITY;
    return ChassisRuntime_QueueCommand(&command, NULL);
}

bool ChassisRuntime_GotoXY(float x_mm, float y_mm, float yaw_deg,
                           uint32_t *sequence)
{
    ChassisCommand_t command;
    if (!isfinite(x_mm) || !isfinite(y_mm) || !isfinite(yaw_deg)) return false;
    memset(&command, 0, sizeof(command));
    command.type = CHASSIS_COMMAND_GOTO_XY;
    command.x_mm = x_mm;
    command.y_mm = y_mm;
    command.yaw_deg = yaw_deg;
    return ChassisRuntime_QueueCommand(&command, sequence);
}

bool ChassisRuntime_TurnTo(float yaw_deg, uint32_t *sequence)
{
    ChassisCommand_t command;
    if (!isfinite(yaw_deg)) return false;
    memset(&command, 0, sizeof(command));
    command.type = CHASSIS_COMMAND_TURN_TO;
    command.yaw_deg = yaw_deg;
    return ChassisRuntime_QueueCommand(&command, sequence);
}

static bool ChassisRuntime_QueueLimited(float x_mm, float y_mm, float yaw_deg,
                                        float max_rpm, uint8_t transit_mode,
                                        uint32_t *sequence)
{
    ChassisCommand_t command;
    if (!isfinite(x_mm) || !isfinite(y_mm) || !isfinite(yaw_deg) ||
        !isfinite(max_rpm) || max_rpm <= 0.0f) return false;
    memset(&command, 0, sizeof(command));
    command.type = CHASSIS_COMMAND_GOTO_XY_LIMITED;
    command.x_mm = x_mm;
    command.y_mm = y_mm;
    command.yaw_deg = yaw_deg;
    command.max_rpm = max_rpm;
    command.transit_mode = transit_mode;
    return ChassisRuntime_QueueCommand(&command, sequence);
}

bool ChassisRuntime_GotoXYLimited(float x_mm, float y_mm, float yaw_deg,
                                  float max_rpm, uint32_t *sequence)
{
    return ChassisRuntime_QueueLimited(x_mm, y_mm, yaw_deg, max_rpm, 0U, sequence);
}

bool ChassisRuntime_GotoXYLimitedTransit(float x_mm, float y_mm, float yaw_deg,
                                         float max_rpm, uint32_t *sequence)
{
    return ChassisRuntime_QueueLimited(x_mm, y_mm, yaw_deg, max_rpm, 1U, sequence);
}

bool ChassisRuntime_GotoXYTransit(float x_mm, float y_mm, float yaw_deg,
                                  uint32_t *sequence)
{
    ChassisCommand_t command;
    if (!isfinite(x_mm) || !isfinite(y_mm) || !isfinite(yaw_deg)) return false;
    memset(&command, 0, sizeof(command));
    command.type = CHASSIS_COMMAND_GOTO_XY;
    command.x_mm = x_mm;
    command.y_mm = y_mm;
    command.yaw_deg = yaw_deg;
    command.transit_mode = 1U;
    return ChassisRuntime_QueueCommand(&command, sequence);
}

bool ChassisRuntime_TurnToTransit(float yaw_deg, uint32_t *sequence)
{
    ChassisCommand_t command;
    if (!isfinite(yaw_deg)) return false;
    memset(&command, 0, sizeof(command));
    command.type = CHASSIS_COMMAND_TURN_TO;
    command.yaw_deg = yaw_deg;
    command.transit_mode = 1U;
    return ChassisRuntime_QueueCommand(&command, sequence);
}

bool ChassisRuntime_CorrectPose(float x_mm, float y_mm, float yaw_deg)
{
    return ChassisRuntime_CorrectPoseWithSequence(x_mm, y_mm, yaw_deg, NULL);
}

bool ChassisRuntime_CorrectPoseWithSequence(float x_mm, float y_mm, float yaw_deg,
                                           uint32_t *sequence)
{
    ChassisCommand_t command;
    if (!isfinite(x_mm) || !isfinite(y_mm) || !isfinite(yaw_deg)) return false;
    memset(&command, 0, sizeof(command));
    command.type = CHASSIS_COMMAND_CORRECT_POSE;
    command.x_mm = x_mm;
    command.y_mm = y_mm;
    command.yaw_deg = yaw_deg;
    return ChassisRuntime_QueueCommand(&command, sequence);
}

bool ChassisRuntime_Stop(void)
{
    /* 停车抢占等待中的轨迹。清队列和投递停止放在同一临界区，
     * 避免停下后又取出停车以前的旧目标。仅从任务上下文调用。 */
    ChassisCommand_t command = {0};
    BaseType_t result;
    if (!s_command_queue) return false;
    command.type = CHASSIS_COMMAND_STOP;
    taskENTER_CRITICAL();
    xQueueReset(s_command_queue);
    result = xQueueSend(s_command_queue, &command, 0);
    taskEXIT_CRITICAL();
    return result == pdPASS;
}

void ChassisRuntime_GetState(ChassisRuntime_State_t *state)
{
    if (state == NULL) return;
    taskENTER_CRITICAL();
    *state = s_published;
    taskEXIT_CRITICAL();
}
