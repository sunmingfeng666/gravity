#include "Arm_Control.h"
#include <math.h>


volatile ArmCtrl_Debug_t ArmCtrl_Debug = {
    .axis_mode = {
        ARM_CTRL_MODE_NORMAL,
        ARM_CTRL_MODE_IMPEDANCE,
        ARM_CTRL_MODE_NORMAL,
        ARM_CTRL_MODE_IMPEDANCE,
        ARM_CTRL_MODE_IMPEDANCE,
        ARM_CTRL_MODE_NORMAL,
    },
    .gc_range_limit_enable = 0,
    .impedance_enable = 1,
    /* MIT 模式使用的柔顺关节阻抗增益 */
    .imp_kp = {0.0f, 12.0f, 3.0f, 0.0f, 5.0f, 2.0f},
    .imp_kd = {0.0f, 4.0f, 0.15f, 0.0f, 0.18f, 0.10f},
    .j4_imp_kp = 6.0f,
    .j4_imp_kd = 0.25f,
    /* J2: A*sin(q2)+B*cos(q2)+D*sin(q2+q4)+E*cos(q2+q4)+C。 */
    .j2_gc = {
        .enable = 1,
        .scale = 1.0f,
        .a = 1.7062f,
        .b = -0.0653f,
        .c = -0.0309f,
        .d = 0.4963f,
        .e = -0.1270f,
        .h = 0.0f,
        .i = 0.0f,
        .min_rad = 0.48f,
        .max_rad = 3.25f,
    },
    /* J4: 包含 q2、q4、q2+q4 的耦合重力模型。 */
    .j4_gc = {
        .enable = 1,
        .scale = 1.0f,
        .a = -1.3902f,
        .b = 0.1950f,
        .c = 0.9557f,
        .d = -0.0593f,
        .e = -1.6175f,
        .f = 1.1773f,
        .g = -0.2333f,
        .min_rad = -1.80f,
        .max_rad = 1.74f,
    },
    /* J5: 包含局部 q5 和 q4+q5 的重力模型。 */
    .j5_gc = {
        .enable = 1,
        .scale = 1.0f,
        .a = 0.6250f,
        .b = 0.8513f,
        .c = -0.0244f,
        .d = 0.0375f,
        .e = -0.5463f,
        .min_rad = -1.59f,
        .max_rad = 1.60f,
    },
    .pid = {
        .kp = {0.0f, 4.0f, 0.0f, 2.0f, 1.0f, 0.0f},
        .ki = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .kd = {0.0f, 0.20f, 0.0f, 0.10f, 0.05f, 0.0f},
        .deadband = {0.0f, 0.005f, 0.0f, 0.005f, 0.005f, 0.0f},
        .i_limit = {0.0f, 0.3f, 0.0f, 0.2f, 0.1f, 0.0f},
        .out_limit = {0.0f, 2.0f, 0.0f, 1.2f, 0.6f, 0.0f},
    },
    /* 关节级导纳默认关闭。重力补偿稳定后再打开测试。 */
    .adm = {
        .enable = 0,
        .mask = (1U << ARM_CTRL_AXIS_J2) | (1U << ARM_CTRL_AXIS_J4) | (1U << ARM_CTRL_AXIS_J5),
        .mass = {1.0f, 1.0f, 1.0f, 1.0f, 0.8f, 1.0f},
        .damping = {0.0f, 3.0f, 0.0f, 2.0f, 1.0f, 0.0f},
        .stiffness = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .deadband = {0.0f, 0.15f, 0.0f, 0.08f, 0.05f, 0.0f},
        .tau_limit = {0.0f, 1.5f, 0.0f, 0.8f, 0.4f, 0.0f},
        .vel_limit = {0.0f, 0.45f, 0.0f, 0.50f, 0.60f, 0.0f},
        .filter_alpha = {0.0f, 0.90f, 0.0f, 0.90f, 0.90f, 0.0f},
        .pos_min = {0.0f, -0.532349f, 0.0f, -3.1415926f, -1.59f, 0.0f},
        .pos_max = {0.0f, 3.67875862f, 0.0f, 3.1415926f, 1.60f, 0.0f},
    },
};

static float armctrl_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float armctrl_deadband(float value, float deadband)
{
    if (value > -deadband && value < deadband) {
        return 0.0f;
    }
    return value;
}

static float armctrl_calc_j2_gravity_tau(float q2, float q4)
{
    if (J2ID_GcRangeLimitEnable != 0U &&
        (q2 < J2ID_GcMinRad || q2 > J2ID_GcMaxRad)) {
        return 0.0f;
    }

    return J2ID_GcScale * (J2ID_GcA * sinf(q2) +
                           J2ID_GcB * cosf(q2) +
                           J2ID_GcD * sinf(q2 + q4) +
                           J2ID_GcE * cosf(q2 + q4) +
                           J2ID_GcC);
}

static float armctrl_calc_j4_gravity_tau(float q2, float q4)
{
    if (J2ID_GcRangeLimitEnable != 0U &&
        (q4 < J4ID_GcMinRad || q4 > J4ID_GcMaxRad)) {
        return 0.0f;
    }

    return J4ID_GcScale * (J4ID_GcA * sinf(q2) +
                           J4ID_GcB * cosf(q2) +
                           J4ID_GcD * sinf(q4) +
                           J4ID_GcE * cosf(q4) +
                           J4ID_GcF * sinf(q2 + q4) +
                           J4ID_GcG * cosf(q2 + q4) +
                           J4ID_GcC);
}

static float armctrl_calc_j5_gravity_tau(float q4, float q5)
{
    if (J2ID_GcRangeLimitEnable != 0U &&
        (q5 < J5ID_GcMinRad || q5 > J5ID_GcMaxRad)) {
        return 0.0f;
    }

    return J5ID_GcScale * (J5ID_GcA * sinf(q5) +
                           J5ID_GcB * cosf(q5) +
                           J5ID_GcD * sinf(q4 + q5) +
                           J5ID_GcE * cosf(q4 + q5) +
                           J5ID_GcC);
}

void ArmCtrl_SelectMitGains(uint8_t axis,
                            float normal_kp,
                            float normal_kd,
                            float *kp,
                            float *kd)
{
    /* 阻抗模式只覆盖 J2/J4/J5；模式3导纳不使用阻抗增益。 */
    if (J2ID_ImpedanceEnable != 0U &&
        (ArmCtrl_Debug.axis_mode[axis] == ARM_CTRL_MODE_IMPEDANCE ||
         ArmCtrl_Debug.axis_mode[axis] == ARM_CTRL_MODE_IMP_ADMITTANCE) &&
        (axis == ARM_CTRL_AXIS_J2 || axis == ARM_CTRL_AXIS_J4 || axis == ARM_CTRL_AXIS_J5)) {
        if (axis == ARM_CTRL_AXIS_J4) {
            *kp = J4ID_ImpKp;
            *kd = J4ID_ImpKd;
        } else {
            *kp = J2ID_ImpKp[axis];
            *kd = J2ID_ImpKd[axis];
        }
    } else {
        *kp = normal_kp;
        *kd = normal_kd;
    }
}

float ArmCtrl_CalcGravityTau(uint8_t axis, float q2, float q4, float q5)
{
    /* 统一重力补偿入口，同时用于前馈和导纳外力估计。 */
    if (ArmCtrl_Debug.axis_mode[axis] < ARM_CTRL_MODE_GRAVITY) {
        return 0.0f;
    }

    if (axis == ARM_CTRL_AXIS_J2 && J2ID_GcEnable != 0U) {
        return armctrl_calc_j2_gravity_tau(q2, q4);
    }
    if (axis == ARM_CTRL_AXIS_J4 && J4ID_GcEnable != 0U) {
        return armctrl_calc_j4_gravity_tau(q2, q4);
    }
    if (axis == ARM_CTRL_AXIS_J5 && J5ID_GcEnable != 0U) {
        return armctrl_calc_j5_gravity_tau(q4, q5);
    }
    return 0.0f;
}

float ArmCtrl_CalcImpedanceTau(uint8_t axis, float target, float pos, float vel)
{
    float kp;
    float kd;

    if (axis >= ARM_CTRL_AXIS_COUNT ||
        (ArmCtrl_Debug.axis_mode[axis] != ARM_CTRL_MODE_IMPEDANCE &&
         ArmCtrl_Debug.axis_mode[axis] != ARM_CTRL_MODE_IMP_ADMITTANCE)) {
        return 0.0f;
    }

    if (axis == ARM_CTRL_AXIS_J4) {
        kp = J4ID_ImpKp;
        kd = J4ID_ImpKd;
    } else {
        kp = J2ID_ImpKp[axis];
        kd = J2ID_ImpKd[axis];
    }

    return kp * (target - pos) - kd * vel;
}

void ArmCtrl_ExtPidReset(uint8_t axis)
{
    if (axis >= ARM_CTRL_AXIS_COUNT) {
        return;
    }
    J2ID_ExtPidIntegral[axis] = 0.0f;
    J2ID_ExtPidOutput[axis] = 0.0f;
}

float ArmCtrl_CalcExtPidTau(uint8_t axis, float dt, float target, float pos, float vel)
{
    float err;
    float out;

    if (axis >= ARM_CTRL_AXIS_COUNT || dt <= 0.0f) {
        return 0.0f;
    }

    if (ArmCtrl_Debug.axis_mode[axis] != ARM_CTRL_MODE_EXT_PID &&
        ArmCtrl_Debug.axis_mode[axis] != ARM_CTRL_MODE_PID_ADMITTANCE) {
        ArmCtrl_ExtPidReset(axis);
        return 0.0f;
    }

    err = target - pos;
    if (err > -J2ID_ExtPidDeadband[axis] && err < J2ID_ExtPidDeadband[axis]) {
        err = 0.0f;
    }

    J2ID_ExtPidIntegral[axis] += err * dt;
    J2ID_ExtPidIntegral[axis] = armctrl_clampf(J2ID_ExtPidIntegral[axis],
                                               -J2ID_ExtPidILimit[axis],
                                                J2ID_ExtPidILimit[axis]);

    out = J2ID_ExtPidKp[axis] * err +
          J2ID_ExtPidKi[axis] * J2ID_ExtPidIntegral[axis] -
          J2ID_ExtPidKd[axis] * vel;
    out = armctrl_clampf(out, -J2ID_ExtPidOutLimit[axis], J2ID_ExtPidOutLimit[axis]);
    J2ID_ExtPidOutput[axis] = out;
    return out;
}

void ArmCtrl_AdmittanceReset(const float target[ARM_CTRL_AXIS_COUNT])
{
    uint8_t axis;

    for (axis = 0U; axis < ARM_CTRL_AXIS_COUNT; axis++) {
        J2ID_AdmTarget[axis] = target[axis];
        J2ID_AdmVelCmd[axis] = 0.0f;
        J2ID_AdmTauExt[axis] = 0.0f;
        J2ID_AdmTauExtFilt[axis] = 0.0f;
    }
}

void ArmCtrl_AdmittanceStep(float dt,
                            float target[ARM_CTRL_AXIS_COUNT],
                            const float pos[ARM_CTRL_AXIS_COUNT],
                            const float vel[ARM_CTRL_AXIS_COUNT],
                            const float tor[ARM_CTRL_AXIS_COUNT],
                            const float gravity_tau[ARM_CTRL_AXIS_COUNT])
{
    uint8_t axis;

    if (J2ID_AdmittanceEnable == 0U || dt <= 0.0f) {
        ArmCtrl_AdmittanceReset(target);
        return;
    }

    for (axis = 0U; axis < ARM_CTRL_AXIS_COUNT; axis++) {
        float tau_ext;
        float alpha;
        float acc_cmd;
        float mass;

        if ((J2ID_AdmittanceMask & (1U << axis)) == 0U ||
            (ArmCtrl_Debug.axis_mode[axis] != ARM_CTRL_MODE_ADMITTANCE &&
             ArmCtrl_Debug.axis_mode[axis] != ARM_CTRL_MODE_IMP_ADMITTANCE &&
             ArmCtrl_Debug.axis_mode[axis] != ARM_CTRL_MODE_PID_ADMITTANCE)) {
            J2ID_AdmTarget[axis] = target[axis];
            J2ID_AdmVelCmd[axis] = 0.0f;
            J2ID_AdmTauExt[axis] = 0.0f;
            J2ID_AdmTauExtFilt[axis] = 0.0f;
            continue;
        }

        /* 外部力矩估计：电机反馈力矩减去重力补偿模型。 */
        tau_ext = tor[axis] - gravity_tau[axis];
        tau_ext = armctrl_deadband(tau_ext, J2ID_AdmDeadband[axis]);
        tau_ext = armctrl_clampf(tau_ext, -J2ID_AdmTauLimit[axis], J2ID_AdmTauLimit[axis]);

        alpha = armctrl_clampf(J2ID_AdmFilterAlpha[axis], 0.0f, 0.99f);
        J2ID_AdmTauExt[axis] = tau_ext;
        J2ID_AdmTauExtFilt[axis] = alpha * J2ID_AdmTauExtFilt[axis] + (1.0f - alpha) * tau_ext;

        mass = J2ID_AdmMass[axis];
        if (mass < 0.01f) {
            mass = 0.01f;
        }

        /* 导纳方程：M*qdd + B*qdot + K*(q_cmd-q_ref) = tau_ext。 */
        acc_cmd = (J2ID_AdmTauExtFilt[axis] -
                   J2ID_AdmDamping[axis] * J2ID_AdmVelCmd[axis] -
                   J2ID_AdmStiffness[axis] * (J2ID_AdmTarget[axis] - target[axis])) / mass;

        J2ID_AdmVelCmd[axis] += acc_cmd * dt;
        J2ID_AdmVelCmd[axis] = armctrl_clampf(J2ID_AdmVelCmd[axis],
                                              -J2ID_AdmVelLimit[axis],
                                               J2ID_AdmVelLimit[axis]);
        J2ID_AdmTarget[axis] += J2ID_AdmVelCmd[axis] * dt;
        J2ID_AdmTarget[axis] = armctrl_clampf(J2ID_AdmTarget[axis],
                                              J2ID_AdmPosMin[axis],
                                              J2ID_AdmPosMax[axis]);

        target[axis] = J2ID_AdmTarget[axis];
        (void)pos;
        (void)vel;
    }
}
