#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <stdint.h>

#define ARM_CTRL_AXIS_COUNT            6U
#define ARM_CTRL_AXIS_J1               0U
#define ARM_CTRL_AXIS_J2               1U
#define ARM_CTRL_AXIS_J3               2U
#define ARM_CTRL_AXIS_J4               3U
#define ARM_CTRL_AXIS_J5               4U
#define ARM_CTRL_AXIS_J6               5U

#define ARM_CTRL_MODE_NORMAL           0U
#define ARM_CTRL_MODE_GRAVITY          1U
#define ARM_CTRL_MODE_IMPEDANCE        2U
#define ARM_CTRL_MODE_ADMITTANCE       3U
#define ARM_CTRL_MODE_IMP_ADMITTANCE   4U
#define ARM_CTRL_MODE_EXT_PID          5U
#define ARM_CTRL_MODE_PID_ADMITTANCE   6U

typedef struct {
    /* 单个关节的重力补偿开关和拟合参数。 */
    volatile uint8_t enable;
    /* 补偿比例：0.0 表示不输出，1.0 表示使用完整拟合模型。 */
    volatile float scale;
    /* 三角函数模型系数。未使用的项保留，用于保持 Debug 结构体布局稳定。 */
    volatile float a;
    volatile float b;
    volatile float c;
    volatile float d;
    volatile float e;
    volatile float f;
    volatile float g;
    volatile float h;
    volatile float i;
    /* gc_range_limit_enable 打开时使用的角度有效范围，单位 rad。 */
    volatile float min_rad;
    volatile float max_rad;
} ArmCtrl_GravityDebug_t;

typedef struct {
    /* 关节级导纳：外部力矩改变目标位置。 */
    volatile uint8_t enable;
    /* 导纳使能轴掩码。bit1/3/4 分别表示 J2/J4/J5。 */
    volatile uint8_t mask;
    /* 虚拟惯量。数值越大，目标位置移动越慢。 */
    volatile float mass[ARM_CTRL_AXIS_COUNT];
    /* 虚拟阻尼。数值越大，越能抑制漂移和振荡。 */
    volatile float damping[ARM_CTRL_AXIS_COUNT];
    /* 虚拟弹簧刚度。0 表示松手后停在新目标附近。 */
    volatile float stiffness[ARM_CTRL_AXIS_COUNT];
    /* 外部力矩死区，单位 N*m。 */
    volatile float deadband[ARM_CTRL_AXIS_COUNT];
    /* 导纳积分前的外部力矩限幅，单位 N*m。 */
    volatile float tau_limit[ARM_CTRL_AXIS_COUNT];
    /* 目标位置变化速度限幅，单位 rad/s。 */
    volatile float vel_limit[ARM_CTRL_AXIS_COUNT];
    /* 外部力矩低通滤波系数，范围 0..0.99。 */
    volatile float filter_alpha[ARM_CTRL_AXIS_COUNT];
    /* 软件目标位置限幅，单位 rad。 */
    volatile float pos_min[ARM_CTRL_AXIS_COUNT];
    volatile float pos_max[ARM_CTRL_AXIS_COUNT];
    /* 导纳运行状态，方便 Debug 观察。 */
    volatile float target[ARM_CTRL_AXIS_COUNT];
    volatile float vel_cmd[ARM_CTRL_AXIS_COUNT];
    volatile float tau_ext[ARM_CTRL_AXIS_COUNT];
    volatile float tau_ext_filt[ARM_CTRL_AXIS_COUNT];
} ArmCtrl_AdmittanceDebug_t;

typedef struct {
    /* 外部 PID 力矩控制：MIT 内部 Kp/Kd 置 0，只发送 torque_des。 */
    volatile float kp[ARM_CTRL_AXIS_COUNT];
    volatile float ki[ARM_CTRL_AXIS_COUNT];
    volatile float kd[ARM_CTRL_AXIS_COUNT];
    volatile float deadband[ARM_CTRL_AXIS_COUNT];
    volatile float i_limit[ARM_CTRL_AXIS_COUNT];
    volatile float out_limit[ARM_CTRL_AXIS_COUNT];
    volatile float integral[ARM_CTRL_AXIS_COUNT];
    volatile float output[ARM_CTRL_AXIS_COUNT];
} ArmCtrl_PidDebug_t;

typedef struct {
    /* Keil Watch 里只添加这一个变量，就能调重力补偿、阻抗和导纳。 */
    /* 每个轴的控制模式：0普通MIT，1重力补偿，2重力补偿+阻抗，3重力补偿+导纳，4重力补偿+阻抗+导纳，5重力补偿+外部PID，6重力补偿+外部PID+导纳。 */
    volatile uint8_t axis_mode[ARM_CTRL_AXIS_COUNT];
    volatile uint8_t gc_range_limit_enable;
    /* 阻抗总开关，通常保持 1；具体轴是否阻抗由 axis_mode 决定。 */
    volatile uint8_t impedance_enable;
    /* 外部关节阻抗增益。阻抗模式下 MIT 内部 Kp/Kd 置 0，由 STM32 计算力矩。 */
    volatile float imp_kp[ARM_CTRL_AXIS_COUNT];
    volatile float imp_kd[ARM_CTRL_AXIS_COUNT];
    /* J4 保留独立命名增益，兼容之前单独调 J4 的习惯。 */
    volatile float j4_imp_kp;
    volatile float j4_imp_kd;
    /* 重力补偿参数。 */
    ArmCtrl_GravityDebug_t j2_gc;
    ArmCtrl_GravityDebug_t j4_gc;
    ArmCtrl_GravityDebug_t j5_gc;
    /* 外部 PID 参数和运行状态。 */
    ArmCtrl_PidDebug_t pid;
    /* 导纳参数和运行状态。 */
    ArmCtrl_AdmittanceDebug_t adm;
} ArmCtrl_Debug_t;

extern volatile ArmCtrl_Debug_t ArmCtrl_Debug;

#define J2ID_GcRangeLimitEnable    (ArmCtrl_Debug.gc_range_limit_enable)
#define J2ID_ImpedanceEnable       (ArmCtrl_Debug.impedance_enable)
#define J2ID_ImpKp                 (ArmCtrl_Debug.imp_kp)
#define J2ID_ImpKd                 (ArmCtrl_Debug.imp_kd)
#define J4ID_ImpKp                 (ArmCtrl_Debug.j4_imp_kp)
#define J4ID_ImpKd                 (ArmCtrl_Debug.j4_imp_kd)

#define J2ID_GcEnable              (ArmCtrl_Debug.j2_gc.enable)
#define J2ID_GcScale               (ArmCtrl_Debug.j2_gc.scale)
#define J2ID_GcA                   (ArmCtrl_Debug.j2_gc.a)
#define J2ID_GcB                   (ArmCtrl_Debug.j2_gc.b)
#define J2ID_GcC                   (ArmCtrl_Debug.j2_gc.c)
#define J2ID_GcD                   (ArmCtrl_Debug.j2_gc.d)
#define J2ID_GcE                   (ArmCtrl_Debug.j2_gc.e)
#define J2ID_GcH                   (ArmCtrl_Debug.j2_gc.h)
#define J2ID_GcI                   (ArmCtrl_Debug.j2_gc.i)
#define J2ID_GcMinRad              (ArmCtrl_Debug.j2_gc.min_rad)
#define J2ID_GcMaxRad              (ArmCtrl_Debug.j2_gc.max_rad)

#define J4ID_GcEnable              (ArmCtrl_Debug.j4_gc.enable)
#define J4ID_GcScale               (ArmCtrl_Debug.j4_gc.scale)
#define J4ID_GcA                   (ArmCtrl_Debug.j4_gc.a)
#define J4ID_GcB                   (ArmCtrl_Debug.j4_gc.b)
#define J4ID_GcC                   (ArmCtrl_Debug.j4_gc.c)
#define J4ID_GcD                   (ArmCtrl_Debug.j4_gc.d)
#define J4ID_GcE                   (ArmCtrl_Debug.j4_gc.e)
#define J4ID_GcF                   (ArmCtrl_Debug.j4_gc.f)
#define J4ID_GcG                   (ArmCtrl_Debug.j4_gc.g)
#define J4ID_GcMinRad              (ArmCtrl_Debug.j4_gc.min_rad)
#define J4ID_GcMaxRad              (ArmCtrl_Debug.j4_gc.max_rad)

#define J5ID_GcEnable              (ArmCtrl_Debug.j5_gc.enable)
#define J5ID_GcScale               (ArmCtrl_Debug.j5_gc.scale)
#define J5ID_GcA                   (ArmCtrl_Debug.j5_gc.a)
#define J5ID_GcB                   (ArmCtrl_Debug.j5_gc.b)
#define J5ID_GcC                   (ArmCtrl_Debug.j5_gc.c)
#define J5ID_GcD                   (ArmCtrl_Debug.j5_gc.d)
#define J5ID_GcE                   (ArmCtrl_Debug.j5_gc.e)
#define J5ID_GcMinRad              (ArmCtrl_Debug.j5_gc.min_rad)
#define J5ID_GcMaxRad              (ArmCtrl_Debug.j5_gc.max_rad)

#define J2ID_AdmittanceEnable      (ArmCtrl_Debug.adm.enable)
#define J2ID_AdmittanceMask        (ArmCtrl_Debug.adm.mask)
#define J2ID_AdmMass               (ArmCtrl_Debug.adm.mass)
#define J2ID_AdmDamping            (ArmCtrl_Debug.adm.damping)
#define J2ID_AdmStiffness          (ArmCtrl_Debug.adm.stiffness)
#define J2ID_AdmDeadband           (ArmCtrl_Debug.adm.deadband)
#define J2ID_AdmTauLimit           (ArmCtrl_Debug.adm.tau_limit)
#define J2ID_AdmVelLimit           (ArmCtrl_Debug.adm.vel_limit)
#define J2ID_AdmFilterAlpha        (ArmCtrl_Debug.adm.filter_alpha)
#define J2ID_AdmPosMin             (ArmCtrl_Debug.adm.pos_min)
#define J2ID_AdmPosMax             (ArmCtrl_Debug.adm.pos_max)
#define J2ID_AdmTarget             (ArmCtrl_Debug.adm.target)
#define J2ID_AdmVelCmd             (ArmCtrl_Debug.adm.vel_cmd)
#define J2ID_AdmTauExt             (ArmCtrl_Debug.adm.tau_ext)
#define J2ID_AdmTauExtFilt         (ArmCtrl_Debug.adm.tau_ext_filt)

#define J2ID_AxisMode              (ArmCtrl_Debug.axis_mode)
#define J2ID_ExtPidKp              (ArmCtrl_Debug.pid.kp)
#define J2ID_ExtPidKi              (ArmCtrl_Debug.pid.ki)
#define J2ID_ExtPidKd              (ArmCtrl_Debug.pid.kd)
#define J2ID_ExtPidDeadband        (ArmCtrl_Debug.pid.deadband)
#define J2ID_ExtPidILimit          (ArmCtrl_Debug.pid.i_limit)
#define J2ID_ExtPidOutLimit        (ArmCtrl_Debug.pid.out_limit)
#define J2ID_ExtPidIntegral        (ArmCtrl_Debug.pid.integral)
#define J2ID_ExtPidOutput          (ArmCtrl_Debug.pid.output)

void ArmCtrl_SelectMitGains(uint8_t axis,
                            float normal_kp,
                            float normal_kd,
                            float *kp,
                            float *kd);
float ArmCtrl_CalcGravityTau(uint8_t axis, float q2, float q4, float q5);
float ArmCtrl_CalcImpedanceTau(uint8_t axis, float target, float pos, float vel);
void ArmCtrl_ExtPidReset(uint8_t axis);
float ArmCtrl_CalcExtPidTau(uint8_t axis, float dt, float target, float pos, float vel);
void ArmCtrl_AdmittanceReset(const float target[ARM_CTRL_AXIS_COUNT]);
void ArmCtrl_AdmittanceStep(float dt,
                            float target[ARM_CTRL_AXIS_COUNT],
                            const float pos[ARM_CTRL_AXIS_COUNT],
                            const float vel[ARM_CTRL_AXIS_COUNT],
                            const float tor[ARM_CTRL_AXIS_COUNT],
                            const float gravity_tau[ARM_CTRL_AXIS_COUNT]);

#endif /* ARM_CONTROL_H */
