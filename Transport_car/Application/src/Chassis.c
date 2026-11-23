#include "Chassis.h"
#include "chassis_config.h"
#include "chassis_runtime.h"
#include "chassis_bus.h"
#include "cmsis_os2.h"
#include <math.h>
#include "IMU.h"

Chassis chassis;

void Chassis_Init(void)
{
    /* 原项目对外用米，移植来的导航算法内部用毫米。只在接口处换单位，
     * 这样视觉任务仍可按 m/s 写 chassis.move，无需了解轮径和脉冲。 */
    chassis.info.wheeltrack = CHASSIS_TRACK_MM / 1000.0f;
    chassis.info.wheelbase = CHASSIS_WHEELBASE_MM / 1000.0f;
    chassis.info.wheelradius = CHASSIS_WHEEL_RADIUS_MM / 1000.0f;
    pid_init(&chassis.Yaw, 0, 0, 0, 1, 8);
    ChassisImu_Start();
    (void)ChassisRuntime_Create(); /* 创建失败会在状态快照中报告 FAULT。 */
}

void Chassis_GetBodyWheelRpm(float rpm[4])
{
    const float rpm_per_mps = 60.0f / (6.283185307f * chassis.info.wheelradius);
    const float lever_m = CHASSIS_TURN_LEVER_MM / 1000.0f;
    const float forward = chassis.move.vx * rpm_per_mps;
    const float left = chassis.move.vy * rpm_per_mps;
    const float turn = chassis.move.vw * lever_m * rpm_per_mps;
    if (!isfinite(forward) || !isfinite(left) || !isfinite(turn)) {
        for (unsigned i = 0; i < 4; ++i) rpm[i] = 0.0f;
        return;
    }
    ChassisNav_MixWheelRpm(forward, left, turn, CHASSIS_MAX_WHEEL_RPM, rpm);
}

/* Watch 写 1 从当前位置沿车头走指定距离；写 2 取消并停车。
 * 测试状态：0 待命、1 行驶、2 已到位、3 未就绪、4 已请求停车、5 故障或超时。
 * 读取后清除命令，以免调试器中保留的 1 让小车反复出发。 */
volatile uint8_t g_chassis_test_cmd;
volatile uint8_t g_chassis_test_status;

#if !defined(CHASSIS_OFFLINE_30CM) && !defined(CHASSIS_OFFLINE_TURN90)
/* 单轮诊断只放在普通 Debug 固件里。Watch 写 cmd=3 启动，写 2 停车。
 * wheel 下标仍为左前、右前、左后、右后，默认测试左前。
 * 5 RPM 转 12 秒，按驱动器的速度单位预计约一圈，实际圈数请看轮胎标记。
 * 不根据编码器计数决定停车，便于发现每圈计数配置错误。
 * status：0 待命，1 转动中，2 完成，3 未就绪，4 已取消，5 通信故障。
 * delta 保存这次测试的原始计数差，不乘方向系数。
 */
volatile uint8_t g_chassis_wheel_test_wheel;
volatile uint8_t g_chassis_wheel_test_status;
volatile int64_t g_chassis_wheel_test_delta[4];
/* fault：0 无故障，1 初始停车发送失败，2 速度发送失败，
 * 3 同步启动发送失败，4 连续五轮查询未收齐，5 结束停车发送失败。
 * last_bad_mask 保留最近一次未收齐的有效位，便于定位缺失的轮子。 */
volatile uint8_t g_chassis_wheel_test_fault;
volatile uint8_t g_chassis_wheel_test_last_bad_mask;

static bool Chassis_TestWheel(void)
{
    static bool active, stopping;
    static uint32_t started_ms, stopped_ms;
    static int64_t initial_count[4];
    static uint8_t missed_cycles;
    static const uint8_t addresses[4] = CHASSIS_MOTOR_ADDRESS_LIST;
    uint8_t cmd = g_chassis_test_cmd;

    if (!active && cmd != 3U) return false;
    /* 可以在 main 断点提前写 3。初始化期间保留命令，并允许正常任务
     * 继续查询编码器；下一周期就绪后再执行，避免先清命令导致漏启动。 */
    if (!active && g_chassis_runtime.status == CHASSIS_RUNTIME_NOT_READY)
        return false;
    g_chassis_test_cmd = 0U;
    if (!active) {
        if (g_chassis_wheel_test_wheel >= 4U ||
            (g_chassis_runtime.status != CHASSIS_RUNTIME_IDLE &&
             g_chassis_runtime.status != CHASSIS_RUNTIME_DONE) ||
            g_chassis_runtime.encoder_valid_mask != CHASSIS_NAV_ENCODER_ALL_VALID) {
            g_chassis_wheel_test_status = 3U;
            return true;
        }
        for (unsigned i = 0; i < 4; ++i) {
            initial_count[i] = g_chassis_runtime.motor_count[i];
            g_chassis_wheel_test_delta[i] = 0;
        }
        active = true;
        stopping = false;
        started_ms = HAL_GetTick();
        missed_cycles = 0U;
        g_chassis_wheel_test_fault = 0U;
        g_chassis_wheel_test_last_bad_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
        g_chassis_wheel_test_status = 1U;
        bool sent = ChassisBus_StopAll();
        if (!sent) g_chassis_wheel_test_fault = 1U;
        osDelay(1U);
        /* 四台驱动都装载目标，防止同步启动时用到上次的残留速度。 */
        for (unsigned i = 0; i < 4; ++i) {
            uint16_t rpm = (i == g_chassis_wheel_test_wheel) ? 5U : 0U;
            if (!ChassisBus_StageSpeed(addresses[i], 1U, rpm)) {
                if (sent) g_chassis_wheel_test_fault = 2U;
                sent = false;
            }
            osDelay(1U);
        }
        if (sent && !ChassisBus_StartTogether()) {
            sent = false;
            g_chassis_wheel_test_fault = 3U;
        }
        if (!sent) {
            g_chassis_wheel_test_status = 5U;
        }
        /* 留出驱动器处理同步命令及发送确认帧的时间，再开始位置查询。 */
        osDelay(5U);
    }

    /* 同一个底盘任务独占总线。诊断期间持续回读四台，核对地址与实物轮序。 */
    uint8_t mask = 0U;
    for (unsigned i = 0; i < 4; ++i) {
        int64_t count;
        if (ChassisBus_ReadPosition(i, addresses[i], &count)) {
            g_chassis_runtime.motor_count[i] = count;
            g_chassis_wheel_test_delta[i] = count - initial_count[i];
            mask |= (uint8_t)(1U << i);
        }
    }
    g_chassis_runtime.encoder_valid_mask = mask;
    if (!stopping && g_chassis_wheel_test_status == 1U) {
        if (mask == CHASSIS_NAV_ENCODER_ALL_VALID) {
            missed_cycles = 0U;
        } else {
            g_chassis_wheel_test_last_bad_mask = mask;
            if (++missed_cycles >= 5U) {
                g_chassis_wheel_test_fault = 4U;
                g_chassis_wheel_test_status = 5U;
            }
        }
    }
    if (cmd == 2U) g_chassis_wheel_test_status = 4U;
    if (!stopping && (HAL_GetTick() - started_ms >= 12000U ||
                      g_chassis_wheel_test_status != 1U)) {
        /* 发送失败时下周期继续尝试停车，避免恢复正常控制覆盖诊断。 */
        if (!ChassisBus_StopAll()) {
            g_chassis_wheel_test_fault = 5U;
            g_chassis_wheel_test_status = 5U;
            return true;
        }
        stopped_ms = HAL_GetTick();
        stopping = true;
    }
    /* 停车后多读半秒，包含停止过程中的计数，再保留结果供 Watch 查看。 */
    if (stopping && HAL_GetTick() - stopped_ms >= 500U) {
        if (g_chassis_wheel_test_status == 1U) g_chassis_wheel_test_status = 2U;
        /* 单轮转动不代表整车正常运动。重新建立里程计起点，
         * 同时重置采样时间，避免把整个诊断时长当成一个控制周期。 */
        ChassisRuntime_EndWheelTest();
        active = false;
    }
    return true;
}
#endif

static void Chassis_TestDistance(void)
{
    static uint32_t sequence, started_ms;
    const ChassisRuntime_State_t *state = &g_chassis_runtime;
    uint8_t cmd = g_chassis_test_cmd;
    /* 只领取本函数的直行/停车命令。调试器可能在单轮检查之后才写入 3，
     * 此时必须留给下一周期的 Chassis_TestWheel，不能在这里吞掉。 */
    if (cmd == 1U || cmd == 2U) g_chassis_test_cmd = 0;
#if defined(CHASSIS_OFFLINE_30CM) || defined(CHASSIS_OFFLINE_TURN90)
    /* 离线测试：每次上电或复位满 3 秒，只尝试出发一次。
     * 与 Watch 写 1 共用下面的四轮反馈检查。若当时未就绪，保持停车。
     * 倒计时期间手动启动或取消，也会用掉本次自动出发机会。 */
    static bool boot_test_consumed;
    if (cmd == 1U || cmd == 2U) boot_test_consumed = true;
    if (!boot_test_consumed && HAL_GetTick() >= 3000U) {
        boot_test_consumed = true;
        cmd = 1U;
    }
#endif
    if (cmd == 2U) {
        g_chassis_test_status = ChassisRuntime_Stop() ? 4U : 5U;
        return;
    }
    if (g_chassis_test_status == 1U) {
        if (state->status == CHASSIS_RUNTIME_FAULT) {
            g_chassis_test_status = 5U;
        } else if (state->status == CHASSIS_RUNTIME_DONE &&
                   state->completed_sequence == sequence) {
            g_chassis_test_status = 2U;
        } else if (HAL_GetTick() - started_ms >= CHASSIS_TEST_TIMEOUT_MS) {
            (void)ChassisRuntime_Stop();
            g_chassis_test_status = 5U;
        }
        return;
    }
    if (cmd != 1U) return;
    if (!state->nav_initialized || state->encoder_valid_mask != CHASSIS_NAV_ENCODER_ALL_VALID ||
        (state->status != CHASSIS_RUNTIME_IDLE && state->status != CHASSIS_RUNTIME_DONE)) {
        g_chassis_test_status = 3U;
        return;
    }
    /* 转向测试从当前航向逆时针转 90 度；沿用同一状态和超时停车逻辑。 */
#ifdef CHASSIS_OFFLINE_TURN90
    if (ChassisRuntime_TurnTo(state->nav.pose.yaw_deg + 90.0f, &sequence)) {
        started_ms = HAL_GetTick();
        g_chassis_test_status = 1U;
    } else {
        g_chassis_test_status = 5U;
    }
#else
    /* 地图航向 0 度朝 +Y。将车头方向投影到地图，重复测试也从当前点出发。 */
    float yaw = state->nav.pose.yaw_deg;
    float radians = yaw * 0.01745329252f;
    float x = state->nav.pose.x_mm - CHASSIS_TEST_DISTANCE_MM * sinf(radians);
    float y = state->nav.pose.y_mm + CHASSIS_TEST_DISTANCE_MM * cosf(radians);
    if (ChassisRuntime_GotoXYLimited(x, y, yaw, 30.0f, &sequence)) {
        started_ms = HAL_GetTick();
        g_chassis_test_status = 1U;
    } else {
        g_chassis_test_status = 5U;
    }
#endif
}

void OS_ChassisCallback(void *argument)
{
    (void)argument;
    Chassis_Init();
    uint32_t wake = osKernelGetTickCount();
    const uint32_t period = CHASSIS_PERIOD_MS * osKernelGetTickFreq() / 1000U;
    /* 唯一占用底盘 USART1 的任务。以绝对时刻唤醒，把本轮读写总线的
     * 时间算进 20 ms；若已超期，从现在重新计时，避免连续补发旧命令。 */
    for (;;) {
#if !defined(CHASSIS_OFFLINE_30CM) && !defined(CHASSIS_OFFLINE_TURN90)
        if (!Chassis_TestWheel()) {
#endif
        ChassisRuntime_TaskStep();
        Chassis_TestDistance();
#if !defined(CHASSIS_OFFLINE_30CM) && !defined(CHASSIS_OFFLINE_TURN90)
        }
#endif
        for (unsigned i = 0; i < 4; ++i)
            chassis.motor[i].targetSpeed = g_chassis_runtime.output.wheel_rpm[i];
        chassis.move.vx_real = g_chassis_runtime.nav.velocity.vx_mm_s / 1000.0f;
        chassis.move.vy_real = g_chassis_runtime.nav.velocity.vy_mm_s / 1000.0f;
        chassis.move.vw_real = g_chassis_runtime.nav.velocity.wz_deg_s * 0.017453293f;
        chassis.move.yawAngle_real = g_chassis_runtime.nav.pose.yaw_deg * 0.017453293f;
        if (g_chassis_runtime.imu_valid) INS.yaw = g_chassis_runtime.imu_yaw_deg * 0.017453293f;
        wake += period;
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(wake - now) <= 0) wake = now + 1U;
        osDelayUntil(wake);
    }
}
