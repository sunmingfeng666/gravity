#include "DM_test.h"
#include "All_Init.h"
#include "Arm_Control.h"
#include "VOFA.h"
#include <math.h>

#define J2ID_MOTOR_J1_ID              0x01U
#define J2ID_MOTOR_J2_ID              0x02U
#define J2ID_MOTOR_J3_ID              0x03U
#define J2ID_MOTOR_J4_ID              0x04U
#define J2ID_MOTOR_J5_ID              0x05U
#define J2ID_MOTOR_J6_ID              0x06U

#define J2ID_AXIS_J1                  0U
#define J2ID_AXIS_J2                  1U
#define J2ID_AXIS_J3                  2U
#define J2ID_AXIS_J4                  3U
#define J2ID_AXIS_J5                  4U
#define J2ID_AXIS_J6                  5U

#define J2ID_J2_MIN_RAD              (-0.532349f)
#define J2ID_J2_MAX_RAD              ( 3.67875862f)
#define J2ID_J4_MIN_RAD              (-3.1415926f)
#define J2ID_J4_MAX_RAD              ( 3.1415926f)
#define J2ID_REMOTE_SCALE             660.0f
#define J2ID_ENABLE_REFRESH_TICKS     100U
#define J2ID_VOFA_DIVIDER             4U

volatile uint8_t J2ID_Enable = 1;
volatile uint8_t J2ID_UseDebugTarget = 0;
volatile uint8_t J2ID_PoseId = 0;
volatile uint8_t J2ID_TaskPeriodMs = 5;
volatile uint8_t J2ID_ControlMask = 0x1FU;
volatile uint8_t J2ID_DisableMask = 0x00U;
volatile uint8_t J2ID_IsolationMode = 1;
volatile uint8_t J2ID_TargetSource = 0;
volatile uint8_t J2ID_SampleAxis = J2ID_AXIS_J3;
volatile uint8_t J2ID_TargetLimitEnable = 0;
volatile uint8_t J2ID_VofaMode = 0;

volatile float J2ID_Kp[6] = {0.0f, 40.0f, 3.0f, 0.0f, 20.0f, 2.0f};
volatile float J2ID_Kd[6] = {0.0f, 0.40f, 0.15f, 0.0f, 0.50f, 0.10f};
volatile float J1ID_Kp = 0.0f;
volatile float J1ID_Kd = 0.0f;
volatile float J1ID_PvSpeedRadps = 2.0f;
volatile float J4ID_Kp = 8.0f;
volatile float J4ID_Kd = 0.30f;
volatile float J2ID_Tff[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_TargetRad[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

volatile float J2ID_J2_TargetRad = 0.0f;
volatile float J2ID_RemoteMaxSpeedRadps = 0.25f;
volatile float J2ID_RemoteDeadband = 80.0f;
volatile float J2ID_StableVelRadps = 0.03f;
volatile float J2ID_StableErrRad = 0.03f;

volatile float J2ID_DebugRemoteRaw = 0.0f;
volatile float J2ID_DebugRemoteNorm = 0.0f;
volatile float J2ID_DebugJ2Err = 0.0f;
volatile float J2ID_DebugControl3J2 = 0.0f;
volatile float J2ID_DebugSampleTarget = 0.0f;
volatile float J2ID_DebugSamplePos = 0.0f;
volatile float J2ID_DebugSampleVel = 0.0f;
volatile float J2ID_DebugSampleTor = 0.0f;
volatile float J2ID_DebugSampleErr = 0.0f;
volatile float J2ID_DebugGcTau = 0.0f;
volatile float J4ID_DebugGcTau = 0.0f;
volatile float J2ID_DebugAxisTarget[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_DebugAxisPos[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_DebugAxisVel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_DebugAxisTor[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_DebugAxisErr[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_DebugAxisGcTau[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_DebugAxisKp[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_DebugAxisKd[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float J2ID_DebugAxisTff[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile int8_t J5ID_DebugModeStatus = 0;
volatile int8_t J5ID_DebugMitStatus = 0;
volatile uint8_t J2ID_DebugStable = 0;

static uint8_t j2id_inited = 0;

static float j2id_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float j2id_deadband(float value, float deadband)
{
    if (value > -deadband && value < deadband) {
        return 0.0f;
    }
    return value;
}

static float j2id_get_axis_target_from_control3(uint8_t axis)
{
    switch (axis) {
    case J2ID_AXIS_J1:
        return ALL_CONTROL.control[3].MECHANICAL_ARM.J1_8009p;
    case J2ID_AXIS_J2:
        return ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P;
    case J2ID_AXIS_J3:
        return ALL_CONTROL.control[3].MECHANICAL_ARM.J3_4340;
    case J2ID_AXIS_J4:
        return ALL_CONTROL.control[3].MECHANICAL_ARM.J4_4340;
    case J2ID_AXIS_J5:
        return ALL_CONTROL.control[3].MECHANICAL_ARM.J5_4310;
    case J2ID_AXIS_J6:
        return ALL_CONTROL.control[3].MECHANICAL_ARM.J6_4310;
    default:
        return 0.0f;
    }
}

static void j2id_set_axis_target_to_control3(uint8_t axis, float target)
{
    switch (axis) {
    case J2ID_AXIS_J1:
        ALL_CONTROL.control[3].MECHANICAL_ARM.J1_8009p = target;
        break;
    case J2ID_AXIS_J2:
        ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P = target;
        break;
    case J2ID_AXIS_J3:
        ALL_CONTROL.control[3].MECHANICAL_ARM.J3_4340 = target;
        break;
    case J2ID_AXIS_J4:
        ALL_CONTROL.control[3].MECHANICAL_ARM.J4_4340 = target;
        break;
    case J2ID_AXIS_J5:
        ALL_CONTROL.control[3].MECHANICAL_ARM.J5_4310 = target;
        break;
    case J2ID_AXIS_J6:
        ALL_CONTROL.control[3].MECHANICAL_ARM.J6_4310 = target;
        break;
    default:
        break;
    }
}

static float j2id_clamp_axis_target(uint8_t axis, float target)
{
    if (J2ID_TargetLimitEnable == 0U) {
        return target;
    }

    if (axis == J2ID_AXIS_J2) {
        return j2id_clampf(target, J2ID_J2_MIN_RAD, J2ID_J2_MAX_RAD);
    }
    if (axis == J2ID_AXIS_J4) {
        return j2id_clampf(target, J2ID_J4_MIN_RAD, J2ID_J4_MAX_RAD);
    }
    return target;
}

static void j2id_enable_all_motors(void)
{
    if ((J2ID_DisableMask & (1U << J2ID_AXIS_J1)) == 0U) {
        motor_mode(&hfdcan2, J2ID_MOTOR_J1_ID, POS_MODE, DM_CMD_MOTOR_MODE);
    } else {
        motor_mode(&hfdcan2, J2ID_MOTOR_J1_ID, POS_MODE, DM_CMD_RESET_MODE);
    }
    osDelay(1);
    if ((J2ID_DisableMask & (1U << J2ID_AXIS_J2)) == 0U) {
        motor_mode(&hfdcan2, J2ID_MOTOR_J2_ID, MIT_MODE, DM_CMD_MOTOR_MODE);
    } else {
        motor_mode(&hfdcan2, J2ID_MOTOR_J2_ID, MIT_MODE, DM_CMD_RESET_MODE);
    }
    osDelay(1);
    if ((J2ID_DisableMask & (1U << J2ID_AXIS_J3)) == 0U) {
        motor_mode(&hfdcan2, J2ID_MOTOR_J3_ID, MIT_MODE, DM_CMD_MOTOR_MODE);
    } else {
        motor_mode(&hfdcan2, J2ID_MOTOR_J3_ID, MIT_MODE, DM_CMD_RESET_MODE);
    }
    osDelay(1);
    if ((J2ID_DisableMask & (1U << J2ID_AXIS_J4)) == 0U) {
        motor_mode(&hfdcan3, J2ID_MOTOR_J4_ID, MIT_MODE, DM_CMD_MOTOR_MODE);
    } else {
        motor_mode(&hfdcan3, J2ID_MOTOR_J4_ID, MIT_MODE, DM_CMD_RESET_MODE);
    }
    osDelay(1);
    if ((J2ID_DisableMask & (1U << J2ID_AXIS_J5)) == 0U) {
        J5ID_DebugModeStatus = motor_mode(&hfdcan3, J2ID_MOTOR_J5_ID, MIT_MODE, DM_CMD_MOTOR_MODE);
    } else {
        J5ID_DebugModeStatus = motor_mode(&hfdcan3, J2ID_MOTOR_J5_ID, MIT_MODE, DM_CMD_RESET_MODE);
    }
    osDelay(1);
    if ((J2ID_DisableMask & (1U << J2ID_AXIS_J6)) == 0U) {
        motor_mode(&hfdcan3, J2ID_MOTOR_J6_ID, MIT_MODE, DM_CMD_MOTOR_MODE);
    } else {
        motor_mode(&hfdcan3, J2ID_MOTOR_J6_ID, MIT_MODE, DM_CMD_RESET_MODE);
    }
    osDelay(1);
}

static float j2id_get_remote_raw(void)
{
    if (VT13.CRC_flag != 0U || VT13.ONLINE_JUDGE_TIME > 5) {
        return (float)VT13.Remote.Channel[1];
    }
    return (float)WHW_V_DBUS.Remote.CH1_int16;
}

static DM_MOTOR_Typdef *j2id_get_motor(uint8_t axis)
{
    switch (axis) {
    case J2ID_AXIS_J1:
        return &Motors.DM_8009_J1;
    case J2ID_AXIS_J2:
        return &Motors.DM_8009_J2;
    case J2ID_AXIS_J3:
        return &Motors.DM_4340_J3;
    case J2ID_AXIS_J4:
        return &Motors.DM_4340_J4;
    case J2ID_AXIS_J5:
        return &Motors.DM_4310_J5;
    case J2ID_AXIS_J6:
        return &Motors.DM_4310_J6;
    default:
        return &Motors.DM_8009_J2;
    }
}

static void j2id_capture_targets_from_feedback(void)
{
    J2ID_TargetRad[J2ID_AXIS_J1] = Motors.DM_8009_J1.DATA.pos;
    J2ID_TargetRad[J2ID_AXIS_J2] = Motors.DM_8009_J2.DATA.pos;
    J2ID_TargetRad[J2ID_AXIS_J3] = Motors.DM_4340_J3.DATA.pos;
    J2ID_TargetRad[J2ID_AXIS_J4] = Motors.DM_4340_J4.DATA.pos;
    J2ID_TargetRad[J2ID_AXIS_J5] = Motors.DM_4310_J5.DATA.pos;
    J2ID_TargetRad[J2ID_AXIS_J6] = Motors.DM_4310_J6.DATA.pos;
    J2ID_J2_TargetRad = J2ID_TargetRad[J2ID_AXIS_J2];
    J2ID_J2_TargetRad = j2id_clamp_axis_target(J2ID_AXIS_J2, J2ID_J2_TargetRad);
    J2ID_TargetRad[J2ID_AXIS_J2] = J2ID_J2_TargetRad;
    j2id_set_axis_target_to_control3(J2ID_AXIS_J1, J2ID_TargetRad[J2ID_AXIS_J1]);
    ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P = J2ID_J2_TargetRad;
    j2id_set_axis_target_to_control3(J2ID_AXIS_J3, J2ID_TargetRad[J2ID_AXIS_J3]);
    j2id_set_axis_target_to_control3(J2ID_AXIS_J4, J2ID_TargetRad[J2ID_AXIS_J4]);
}

static void j2id_update_axis_debug(void)
{
    uint8_t axis;

    for (axis = 0U; axis < 6U; axis++) {
        DM_MOTOR_Typdef *motor = j2id_get_motor(axis);
        J2ID_DebugAxisTarget[axis] = J2ID_TargetRad[axis];
        J2ID_DebugAxisPos[axis] = motor->DATA.pos;
        J2ID_DebugAxisVel[axis] = motor->DATA.vel;
        J2ID_DebugAxisTor[axis] = motor->DATA.tor;
        J2ID_DebugAxisErr[axis] = J2ID_DebugAxisTarget[axis] - J2ID_DebugAxisPos[axis];
    }
}

static void j2id_send_j1_pv(void)
{
    Motors.DM_8009_J1.PV.POS = J2ID_TargetRad[J2ID_AXIS_J1];
    Motors.DM_8009_J1.PV.SPE = J1ID_PvSpeedRadps;
    J2ID_DebugAxisKp[J2ID_AXIS_J1] = J1ID_Kp;
    J2ID_DebugAxisKd[J2ID_AXIS_J1] = J1ID_Kd;
    J2ID_DebugAxisTff[J2ID_AXIS_J1] = 0.0f;
    J2ID_DebugAxisGcTau[J2ID_AXIS_J1] = 0.0f;
    pos_speed_ctrl(&hfdcan2, J2ID_MOTOR_J1_ID, &Motors.DM_8009_J1.PV);
}

static void j2id_send_axis_mit(uint8_t axis)
{
    DM_MOTOR_Typdef *motor = j2id_get_motor(axis);
    hcan_t *hcan = &hfdcan2;
    uint16_t motor_id = J2ID_MOTOR_J1_ID;

    if (axis == J2ID_AXIS_J1) {
        j2id_send_j1_pv();
        return;
    }

    switch (axis) {
    case J2ID_AXIS_J1:
        hcan = &hfdcan2;
        motor_id = J2ID_MOTOR_J1_ID;
        break;
    case J2ID_AXIS_J2:
        hcan = &hfdcan2;
        motor_id = J2ID_MOTOR_J2_ID;
        break;
    case J2ID_AXIS_J3:
        hcan = &hfdcan2;
        motor_id = J2ID_MOTOR_J3_ID;
        break;
    case J2ID_AXIS_J4:
        hcan = &hfdcan3;
        motor_id = J2ID_MOTOR_J4_ID;
        break;
    case J2ID_AXIS_J5:
        hcan = &hfdcan3;
        motor_id = J2ID_MOTOR_J5_ID;
        break;
    case J2ID_AXIS_J6:
        hcan = &hfdcan3;
        motor_id = J2ID_MOTOR_J6_ID;
        break;
    default:
        return;
    }

    motor->MIT.position_des = J2ID_TargetRad[axis];
    motor->MIT.velocity_des = 0.0f;
    if (axis == J2ID_AXIS_J4) {
        J2ID_Kp[J2ID_AXIS_J4] = J4ID_Kp;
        J2ID_Kd[J2ID_AXIS_J4] = J4ID_Kd;
    }
    ArmCtrl_SelectMitGains(axis, J2ID_Kp[axis], J2ID_Kd[axis], &motor->MIT.Kp, &motor->MIT.Kd);
    if (J2ID_AxisMode[axis] == ARM_CTRL_MODE_IMPEDANCE ||
        J2ID_AxisMode[axis] == ARM_CTRL_MODE_IMP_ADMITTANCE ||
        J2ID_AxisMode[axis] == ARM_CTRL_MODE_EXT_PID ||
        J2ID_AxisMode[axis] == ARM_CTRL_MODE_PID_ADMITTANCE) {
        motor->MIT.Kp = 0.0f;
        motor->MIT.Kd = 0.0f;
    }
    motor->MIT.torque_des = J2ID_Tff[axis];
    J2ID_DebugAxisGcTau[axis] = ArmCtrl_CalcGravityTau(axis,
                                                       Motors.DM_8009_J2.DATA.pos,
                                                       Motors.DM_4340_J4.DATA.pos,
                                                       Motors.DM_4310_J5.DATA.pos);
    motor->MIT.torque_des += J2ID_DebugAxisGcTau[axis];
    motor->MIT.torque_des += ArmCtrl_CalcImpedanceTau(axis,
                                                      J2ID_TargetRad[axis],
                                                      motor->DATA.pos,
                                                      motor->DATA.vel);
    motor->MIT.torque_des += ArmCtrl_CalcExtPidTau(axis,
                                                   ((float)J2ID_TaskPeriodMs) * 0.001f,
                                                   J2ID_TargetRad[axis],
                                                   motor->DATA.pos,
                                                   motor->DATA.vel);
    if (axis == J2ID_AXIS_J2) {
        J2ID_DebugGcTau = J2ID_DebugAxisGcTau[axis];
    } else if (axis == J2ID_AXIS_J4) {
        J4ID_DebugGcTau = J2ID_DebugAxisGcTau[axis];
    }
    J2ID_DebugAxisKp[axis] = motor->MIT.Kp;
    J2ID_DebugAxisKd[axis] = motor->MIT.Kd;
    J2ID_DebugAxisTff[axis] = motor->MIT.torque_des;
    if (axis == J2ID_AXIS_J5) {
        J5ID_DebugMitStatus = mit_ctrl(hcan, motor_id, &motor->MIT);
    } else {
        mit_ctrl(hcan, motor_id, &motor->MIT);
    }
}

void DM_test_init(void)
{
    j2id_enable_all_motors();
    j2id_capture_targets_from_feedback();
    ArmCtrl_AdmittanceReset((const float *)J2ID_TargetRad);
    j2id_inited = 1;
}

void DM_test(IMU_Data_t *imu_data)
{
    (void)imu_data;
    J2_Identify_TaskStep();
}

void J2_Identify_TaskStep(void)
{
    static uint32_t tick = 0U;
    const float dt = ((float)J2ID_TaskPeriodMs) * 0.001f;
    float remote_raw;
    float remote_norm;
    uint8_t axis;

    if (j2id_inited == 0U) {
        DM_test_init();
    }

    if ((tick % J2ID_ENABLE_REFRESH_TICKS) == 0U) {
        j2id_enable_all_motors();
    }

    remote_raw = j2id_get_remote_raw();
    remote_norm = j2id_deadband(remote_raw, J2ID_RemoteDeadband) / J2ID_REMOTE_SCALE;
    remote_norm = j2id_clampf(remote_norm, -1.0f, 1.0f);

    J2ID_DebugRemoteRaw = remote_raw;
    J2ID_DebugRemoteNorm = remote_norm;

    J2ID_DebugControl3J2 = ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P;

    if (J2ID_UseDebugTarget != 0U) {
        J2ID_TargetSource = 2U;
    }

    if (J2ID_TargetSource == 0U) {
        J2ID_J2_TargetRad = ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P;
    } else if (J2ID_TargetSource == 1U) {
        J2ID_J2_TargetRad += remote_norm * J2ID_RemoteMaxSpeedRadps * dt;
    }
    J2ID_J2_TargetRad = j2id_clamp_axis_target(J2ID_AXIS_J2, J2ID_J2_TargetRad);
    if (J2ID_TargetSource == 0U) {
        ALL_CONTROL.control[3].MECHANICAL_ARM.J2_8009P = J2ID_J2_TargetRad;
    }
    J2ID_TargetRad[J2ID_AXIS_J2] = J2ID_J2_TargetRad;

    if (J2ID_TargetSource == 0U) {
        J2ID_TargetRad[J2ID_AXIS_J1] =
            j2id_clamp_axis_target(J2ID_AXIS_J1, j2id_get_axis_target_from_control3(J2ID_AXIS_J1));
        J2ID_TargetRad[J2ID_AXIS_J3] =
            j2id_clamp_axis_target(J2ID_AXIS_J3, j2id_get_axis_target_from_control3(J2ID_AXIS_J3));
        J2ID_TargetRad[J2ID_AXIS_J4] =
            j2id_clamp_axis_target(J2ID_AXIS_J4, j2id_get_axis_target_from_control3(J2ID_AXIS_J4));
        J2ID_TargetRad[J2ID_AXIS_J5] =
            j2id_clamp_axis_target(J2ID_AXIS_J5, j2id_get_axis_target_from_control3(J2ID_AXIS_J5));
        j2id_set_axis_target_to_control3(J2ID_AXIS_J1, J2ID_TargetRad[J2ID_AXIS_J1]);
        j2id_set_axis_target_to_control3(J2ID_AXIS_J3, J2ID_TargetRad[J2ID_AXIS_J3]);
        j2id_set_axis_target_to_control3(J2ID_AXIS_J4, J2ID_TargetRad[J2ID_AXIS_J4]);
        j2id_set_axis_target_to_control3(J2ID_AXIS_J5, J2ID_TargetRad[J2ID_AXIS_J5]);
    }

    {
        float axis_pos[6];
        float axis_vel[6];
        float axis_tor[6];
        float gravity_tau[6];

        for (axis = 0U; axis < 6U; axis++) {
            DM_MOTOR_Typdef *motor = j2id_get_motor(axis);
            axis_pos[axis] = motor->DATA.pos;
            axis_vel[axis] = motor->DATA.vel;
            axis_tor[axis] = motor->DATA.tor;
            gravity_tau[axis] = ArmCtrl_CalcGravityTau(axis,
                                                       Motors.DM_8009_J2.DATA.pos,
                                                       Motors.DM_4340_J4.DATA.pos,
                                                       Motors.DM_4310_J5.DATA.pos);
        }
        ArmCtrl_AdmittanceStep(dt, (float *)J2ID_TargetRad, axis_pos, axis_vel, axis_tor, gravity_tau);
        if (J2ID_TargetSource == 0U) {
            j2id_set_axis_target_to_control3(J2ID_AXIS_J2, J2ID_TargetRad[J2ID_AXIS_J2]);
            j2id_set_axis_target_to_control3(J2ID_AXIS_J4, J2ID_TargetRad[J2ID_AXIS_J4]);
            j2id_set_axis_target_to_control3(J2ID_AXIS_J5, J2ID_TargetRad[J2ID_AXIS_J5]);
        }
    }

    if (J2ID_Enable != 0U) {
        for (axis = 0U; axis < 6U; axis++) {
            if (((J2ID_ControlMask & (1U << axis)) != 0U) &&
                ((J2ID_DisableMask & (1U << axis)) == 0U)) {
                j2id_send_axis_mit(axis);
                osDelay(1);
            }
        }
    }

    j2id_update_axis_debug();

    {
        DM_MOTOR_Typdef *sample_motor = j2id_get_motor(J2ID_SampleAxis);
        J2ID_DebugSampleTarget = J2ID_TargetRad[J2ID_SampleAxis];
        J2ID_DebugSamplePos = sample_motor->DATA.pos;
        J2ID_DebugSampleVel = sample_motor->DATA.vel;
        J2ID_DebugSampleTor = sample_motor->DATA.tor;
        J2ID_DebugSampleErr = J2ID_DebugSampleTarget - J2ID_DebugSamplePos;
    }

    J2ID_DebugJ2Err = J2ID_J2_TargetRad - Motors.DM_8009_J2.DATA.pos;
    J2ID_DebugStable = (uint8_t)((fabsf(J2ID_DebugSampleVel) < J2ID_StableVelRadps) &&
                                 (fabsf(J2ID_DebugSampleErr) < J2ID_StableErrRad));

    if ((tick % J2ID_VOFA_DIVIDER) == 0U) {
        if (J2ID_VofaMode == 1U) {
            const float j2_err = J2ID_TargetRad[J2ID_AXIS_J2] - Motors.DM_8009_J2.DATA.pos;
            const float j4_err = J2ID_TargetRad[J2ID_AXIS_J4] - Motors.DM_4340_J4.DATA.pos;
            const float j5_err = J2ID_TargetRad[J2ID_AXIS_J5] - Motors.DM_4310_J5.DATA.pos;
            VOFA_justfloat(j2_err,
                           j4_err,
                           j5_err,
                           Motors.DM_8009_J2.DATA.tor,
                           Motors.DM_4340_J4.DATA.tor,
                           Motors.DM_4310_J5.DATA.tor,
                           J2ID_DebugGcTau,
                           J4ID_DebugGcTau,
                           (float)J2ID_ImpedanceEnable,
                           Motors.DM_8009_J1.DATA.pos);
        } else {
            VOFA_justfloat(Motors.DM_8009_J1.DATA.pos,
                           Motors.DM_8009_J2.DATA.pos,
                           Motors.DM_4340_J4.DATA.pos,
                           Motors.DM_4310_J5.DATA.pos,
                           J2ID_DebugSampleTarget,
                           J2ID_DebugSamplePos,
                           J2ID_DebugSampleVel,
                           J2ID_DebugSampleTor,
                           (float)J2ID_DebugStable,
                           (float)J2ID_SampleAxis);
        }
    }

    tick++;
}

void StartJ2IdentifyTask(void const *argument)
{
    (void)argument;
    osDelay(1000);
    DM_test_init();

    for (;;) {
        J2_Identify_TaskStep();
        osDelay(J2ID_TaskPeriodMs);
    }
}


