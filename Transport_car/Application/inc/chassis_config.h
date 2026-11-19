#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 底盘标定入口。尺寸沿用 Transport_car 的原值，尚未实测。
 * 轮距：左、右轮中心间距；轴距：前、后轮中心间距，全部用毫米。
 * 下层驱动与里程计始终共用同一轮序：左前、右前、左后、右后。 */
#define CHASSIS_WHEEL_RADIUS_MM     30.0f
#define CHASSIS_TRACK_MM           300.0f
#define CHASSIS_WHEELBASE_MM       300.0f
#define CHASSIS_MOTOR_ADDRESS_LIST {1U, 2U, 3U, 4U}

/* 这里按 Emm5.0 固件、直连轮配置。0x36 回读一整轮的计数，
 * 与发给位置模式的细分脉冲数是两个量。有减速器时还要换算传动比。 */
#define CHASSIS_COUNTS_PER_WHEEL_REV 65536.0f
#define CHASSIS_ENCODER_SIGN        (-1)
#define CHASSIS_MAX_WHEEL_RPM       170.0f
/* 以下为参考车试车值：车体转 1 度所需的电机命令脉冲数。 */
#define CHASSIS_COARSE_PULSES_PER_DEG 43.5f
#define CHASSIS_PERIOD_MS            20U

#endif
