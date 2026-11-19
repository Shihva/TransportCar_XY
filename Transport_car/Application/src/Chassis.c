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
    const float lever_m = 0.5f * (chassis.info.wheeltrack + chassis.info.wheelbase);
    const float forward = chassis.move.vx * rpm_per_mps;
    const float left = chassis.move.vy * rpm_per_mps;
    const float turn = chassis.move.vw * lever_m * rpm_per_mps;
    if (!isfinite(forward) || !isfinite(left) || !isfinite(turn)) {
        for (unsigned i = 0; i < 4; ++i) rpm[i] = 0.0f;
        return;
    }
    ChassisNav_MixWheelRpm(forward, left, turn, CHASSIS_MAX_WHEEL_RPM, rpm);
}

/* Watch 写 1 从当前位置沿车头走一次 300 mm；写 2 取消并停车。
 * 测试状态：0 待命、1 行驶、2 已到位、3 未就绪、4 已请求停车、5 故障或超时。
 * 读取后清除命令，以免调试器中保留的 1 让小车反复出发。 */
volatile uint8_t g_chassis_test_cmd;
volatile uint8_t g_chassis_test_status;

static void Chassis_Test30cm(void)
{
    static uint32_t sequence, started_ms;
    const ChassisRuntime_State_t *state = &g_chassis_runtime;
    uint8_t cmd = g_chassis_test_cmd;
    if (cmd) g_chassis_test_cmd = 0;
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
        } else if (HAL_GetTick() - started_ms >= 8000U) {
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
    /* 地图航向 0 度朝 +Y。将车头方向投影到地图，重复测试也从当前点出发。 */
    float yaw = state->nav.pose.yaw_deg;
    float radians = yaw * 0.01745329252f;
    float x = state->nav.pose.x_mm - 300.0f * sinf(radians);
    float y = state->nav.pose.y_mm + 300.0f * cosf(radians);
    if (ChassisRuntime_GotoXYLimited(x, y, yaw, 30.0f, &sequence)) {
        started_ms = HAL_GetTick();
        g_chassis_test_status = 1U;
    } else {
        g_chassis_test_status = 5U;
    }
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
        ChassisRuntime_TaskStep();
        Chassis_Test30cm();
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
