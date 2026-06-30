#ifndef DM_TEST_H
#define DM_TEST_H

#include "DM_Motor.h"
#include "IMU_Task.h"
#include "Arm_Control.h"
#include "stdint.h"

extern volatile uint8_t J2ID_Enable;
extern volatile uint8_t J2ID_UseDebugTarget;
extern volatile uint8_t J2ID_PoseId;
extern volatile uint8_t J2ID_TaskPeriodMs;
extern volatile uint8_t J2ID_ControlMask;
extern volatile uint8_t J2ID_DisableMask;
extern volatile uint8_t J2ID_IsolationMode;
extern volatile uint8_t J2ID_TargetSource;
extern volatile uint8_t J2ID_SampleAxis;
extern volatile uint8_t J2ID_TargetLimitEnable;
extern volatile uint8_t J2ID_VofaMode;

extern volatile float J2ID_Kp[6];
extern volatile float J2ID_Kd[6];
extern volatile float J1ID_Kp;
extern volatile float J1ID_Kd;
extern volatile float J1ID_PvSpeedRadps;
extern volatile float J4ID_Kp;
extern volatile float J4ID_Kd;
extern volatile float J2ID_Tff[6];
extern volatile float J2ID_TargetRad[6];
extern volatile float J2ID_J2_TargetRad;
extern volatile float J2ID_RemoteMaxSpeedRadps;
extern volatile float J2ID_RemoteDeadband;
extern volatile float J2ID_StableVelRadps;
extern volatile float J2ID_StableErrRad;

extern volatile float J2ID_DebugRemoteRaw;
extern volatile float J2ID_DebugRemoteNorm;
extern volatile float J2ID_DebugJ2Err;
extern volatile float J2ID_DebugControl3J2;
extern volatile float J2ID_DebugSampleTarget;
extern volatile float J2ID_DebugSamplePos;
extern volatile float J2ID_DebugSampleVel;
extern volatile float J2ID_DebugSampleTor;
extern volatile float J2ID_DebugSampleErr;
extern volatile float J2ID_DebugGcTau;
extern volatile float J4ID_DebugGcTau;
extern volatile float J2ID_DebugAxisTarget[6];
extern volatile float J2ID_DebugAxisPos[6];
extern volatile float J2ID_DebugAxisVel[6];
extern volatile float J2ID_DebugAxisTor[6];
extern volatile float J2ID_DebugAxisErr[6];
extern volatile float J2ID_DebugAxisGcTau[6];
extern volatile float J2ID_DebugAxisKp[6];
extern volatile float J2ID_DebugAxisKd[6];
extern volatile float J2ID_DebugAxisTff[6];
extern volatile int8_t J5ID_DebugModeStatus;
extern volatile int8_t J5ID_DebugMitStatus;
extern volatile uint8_t J2ID_DebugStable;

void DM_test_init(void);
void DM_test(IMU_Data_t *imu_data);
void J2_Identify_TaskStep(void);
void StartJ2IdentifyTask(void const *argument);

#endif /* DM_TEST_H */
