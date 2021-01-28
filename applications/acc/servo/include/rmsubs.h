extern void OpenRM(void);
extern void UpdateRM(int first, int last);
extern void RMTimestamp(void);
/* WARNING: RMSafeWrite will set the first char of name to NULL if it
 * gets an RM_NAME_INVALID return.  This flag will avoid multiple error
 * messages.
 */
extern void RMSafeWrite(char *name, void *value);

/* define the global variables needed, the initializers for the rv struct
 * and the sizes of things
 */
extern int useFakePadID;
extern int azRockerCWLimit, azRockerCCWLimit;
extern int scbEPROMVersion, scbSRAMVersion;
extern int airPressureSwitch;
extern int checkCollisionLimitSwitch;
extern int azTachDivisor, elTachDivisor;
extern float azIntegralGain, azProportionalGain, azDerivativeGain, azTorqueBias;
extern float elIntegralGain, elProportionalGain, elDerivativeGain;
extern float trErrorBoxtime;
extern enum ENCTYPE elEncType, azEncType;
extern int azEncoderOffset, elEncoderOffset;
extern int azEncoderReversed, elEncoderReversed;
extern float az_amax_f;

#define NUM_INIT_VALUES 32
#define INIT_VALUES \
{BI, TSSHM_OFFSET(padID), "RM_PAD_ID_B"}, \
{BI, ADDRESS(useFakePadID ), "RM_PAD_ID_IS_FAKE_B"}, \
{BI, ADDRESS(checkCollisionLimitSwitch), "RM_CHECK_EL_COLLISION_LIMIT_B"}, \
{FIM, TSSHM_OFFSET(padAzOffset), "RM_PAD_AZ_OFFSET_DEG_F"}, \
{BI, ADDRESS(airPressureSwitch), "RM_SCB_AZIM_BRAKE_RELEASED_B"}, \
{FDM, TSSHM_OFFSET(lowerLimit), "RM_SCB_LOW_LIMIT_F"}, \
{FDM, TSSHM_OFFSET(upperLimit), "RM_SCB_UP_LIMIT_F"}, \
{FDM, TSSHM_OFFSET(cwLimit), "RM_SCB_CW_LIMIT_F"}, \
{FDM, TSSHM_OFFSET(ccwLimit), "RM_SCB_CCW_LIMIT_F"}, \
{FIM, ADDRESS(azRockerCWLimit), "RM_AZ_ROCKER_CW_LIMIT_F"}, \
{FIM, ADDRESS(azRockerCCWLimit), "RM_AZ_ROCKER_CCW_LIMIT_F"}, \
{BI, ADDRESS(scbEPROMVersion), "RM_SCB_EPROM_VERSION_B"}, \
{BI, ADDRESS(scbSRAMVersion), "RM_SCB_SRAM_VERSION_B"}, \
{LI, ADDRESS(azTachDivisor), "RM_AZ_TACHDIVISOR_L"}, \
{LI, ADDRESS(elTachDivisor), "RM_EL_TACHDIVISOR_L"}, \
{FF, ADDRESS(azIntegralGain), "RM_AZ_INTEGRAL_GAIN_F"}, \
{FF, ADDRESS(azProportionalGain), "RM_AZ_PROPORTIONAL_GAIN_F"}, \
{FF, ADDRESS(azDerivativeGain), "RM_AZ_DERIVATIVE_GAIN_F"}, \
{FF, ADDRESS(azTorqueBias), "RM_AZ_TORQUE_BIAS_F"}, \
{FF, ADDRESS(elIntegralGain), "RM_EL_INTEGRAL_GAIN_F"}, \
{FF, ADDRESS(elProportionalGain), "RM_EL_PROPORTIONAL_GAIN_F"}, \
{FF, ADDRESS(elDerivativeGain), "RM_EL_DERIVATIVE_GAIN_F"}, \
{FF, ADDRESS(trErrorBoxtime), "RM_TRACKING_ERROR_BOXTIME_SEC_F"}, \
{BI, ADDRESS(azEncType), "RM_AZ_ENCODER_TYPE_B"}, \
{LI, ADDRESS(azEncoderOffset), "RM_AZ_ENCODER_OFFSET_L"}, \
{SI, ADDRESS(azEncoderReversed), "RM_AZ_ENCODER_REVERSED_S"}, \
{BI, ADDRESS(elEncType), "RM_EL_ENCODER_TYPE_B"}, \
{LI, ADDRESS(elEncoderOffset), "RM_EL_ENCODER_OFFSET_L"}, \
{SI, ADDRESS(elEncoderReversed), "RM_EL_ENCODER_REVERSED_S"}, \
{FF, ADDRESS(az_amax_f), "RM_AZ_AMAX_F"}, \
{LI, ADDRESS(elLimEncZero), "RM_EL_LIM_ENCODER_ZERO_L"}, \
{LI, ADDRESS(azLimEncZero), "RM_AZ_LIM_ENCODER_ZERO_L"}

#define LAST_INIT (NUM_INIT_VALUES - 1)

extern int driveState;
extern enum states azState, elState;
enum M3STATE m3State;
extern int prevNumResets;
extern int ttTimeoutCount;
extern int irig_error_count;
extern int nakCount;
extern int azRockerBits;
extern unsigned int scbShutdownRequest;
extern unsigned int scbStowRequest;
extern int prelimBypassCycles, softLimBypassCycles;
#define FIRST_M_S (NUM_INIT_VALUES)
#define NUM_MONITOR_AND_STATE 28
#define LAST_M_S (FIRST_M_S + NUM_MONITOR_AND_STATE - 1)
#define MONITOR_AND_STATE \
{FF, TSSHM_OFFSET(elMotCurrent), "RM_EL_MOT_CUR_AMP_F"}, \
{FF, TSSHM_OFFSET(azMot1Current), "RM_AZ1_MOT_CUR_AMP_F"}, \
{FF, TSSHM_OFFSET(azMot2Current), "RM_AZ2_MOT_CUR_AMP_F"}, \
{FF, TSSHM_OFFSET(elMotTemp), "RM_EL_MOT_TEMP_C_F"}, \
{FF, TSSHM_OFFSET(azMot1Temp), "RM_AZ1_MOT_TEMP_C_F"}, \
{FF, TSSHM_OFFSET(azMot2Temp), "RM_AZ2_MOT_TEMP_C_F"}, \
{BI, TSSHM_OFFSET(irigLock), "RM_IRIG_LOCK_ERROR_B"}, \
{LI, TSSHM_OFFSET(scbFaultWord), "RM_SCB_FAULTWORD_L"}, \
{BI, TSSHM_OFFSET(scbStatus), "RM_SCB_STATUS_B"}, \
{LI, ADDRESS(prevNumResets), "RM_SCB_RESTARTS_L"}, \
{LI, ADDRESS(ttTimeoutCount), "RM_TRUETIME_TIMEOUT_COUNT_L"}, \
{LI, ADDRESS(irig_error_count), "RM_IRIG_ERROR_COUNT_L"}, \
{LI, ADDRESS(nakCount), "RM_SCB_NAK_COUNT_L"}, \
{BI, TSSHM_OFFSET(fault), "RM_SERVO_FAULT_STATE_B"}, \
{BI, ADDRESS(driveState), "RM_ANTENNA_DRIVE_STATUS_B"}, \
{BI, TSSHM_OFFSET(azCmd), "RM_AZ_DRV_CMD_B"}, \
{BI, TSSHM_OFFSET(elCmd), "RM_EL_DRV_CMD_B"}, \
{BI, ADDRESS(azState), "RM_AZ_DRV_STATE_B"}, \
{BI, ADDRESS(elState), "RM_EL_DRV_STATE_B"}, \
{BI, ADDRESS(azRockerBits), "RM_AZ_ROCKER_BITS_B"}, \
{BI, ADDRESS(scbShutdownRequest), "RM_SCB_SHUTDOWN_REQUEST_B"}, \
{BI, ADDRESS(scbStowRequest), "RM_SCB_STOW_REQUEST_B"}, \
{SI, ADDRESS(prelimBypassCycles), "RM_SCB_BYPASS_PRELIMS_S"}, \
{SI, ADDRESS(softLimBypassCycles), "RM_SCB_BYPASS_SOFTLIMS_S"}, \
{SI, ADDRESS(presentSunSafeMinutes), "RM_PRESENT_SUN_SAFE_MINUTES_S"}, \
{SI, ADDRESS(posInSunAvoid), "RM_SERVO_POS_IN_SUN_AVOID_S"}, \
{SI, ADDRESS(avoidingSun), "RM_SERVO_AVOIDING_SUN_S"}, \
{BI, ADDRESS(m3State), "RM_M3STATE_B"}

extern float azTrErrorArcSec, elTrErrorArcSec, totTrErrorArcSec;
extern int elLimEncZero, azLimEncZero;
#define FIRST_T_P (FIRST_M_S + NUM_MONITOR_AND_STATE)
#define NUM_TIME_POSITION 18
#define LAST_T_P (FIRST_T_P + NUM_TIME_POSITION - 1)
#define TIME_POSITION \
{LI, TSSHM_OFFSET(msecCmd), "RM_MSEC_TRACK_CMD_L"}, \
{LI, TSSHM_OFFSET(msecAccept), "RM_MSEC_SERVO_ACCEPT_L"}, \
{LI, TSSHM_OFFSET(msec), "RM_MSEC_NOW_L"}, \
{FDM, TSSHM_OFFSET(az), "RM_TRACK_AZ_F"}, \
{FDM, TSSHM_OFFSET(el), "RM_TRACK_EL_F"}, \
{FDM, TSSHM_OFFSET(azVel), "RM_TRACK_AZ_VEL_F"}, \
{FDM, TSSHM_OFFSET(elVel), "RM_TRACK_EL_VEL_F"}, \
{FIM, TSSHM_OFFSET(cmdAz), "RM_SHAPED_CMD_AZ_F"}, \
{FIM, TSSHM_OFFSET(cmdEl), "RM_SHAPED_CMD_EL_F"}, \
{FIM, TSSHM_OFFSET(encAz), "RM_ENCODER_AZ_F"}, \
{FIM, TSSHM_OFFSET(encEl), "RM_ENCODER_EL_F"}, \
{FIM, TSSHM_OFFSET(tachAzVel), "RM_TACH_AZ_VEL_F"}, \
{FIM, TSSHM_OFFSET(tachElVel), "RM_TACH_EL_VEL_F"}, \
{FDM, TSSHM_OFFSET(limAz), "RM_LIM_ENCODER_AZ_F"}, \
{FDM, TSSHM_OFFSET(limEl), "RM_LIM_ENCODER_EL_F"}, \
{FF, ADDRESS(azTrErrorArcSec), "RM_AZ_TRACKING_ERROR_F"}, \
{FF, ADDRESS(elTrErrorArcSec), "RM_EL_TRACKING_ERROR_F"}, \
{FF, TSSHM_OFFSET(avgTrErrorArcSec), "RM_TRACKING_ERROR_ARCSEC_F"}

#define RM_WRITE_ALL UpdateRM(0, LAST_T_P)
#define UPDATE_INIT UpdateRM(0, LAST_INIT)
#define UPDATE_SOME_INIT UpdateRM(0, 5)
#define UPDATE_MONITOR_AND_STATE UpdateRM(FIRST_M_S, LAST_M_S)
#define UPDATE_TIME_POSITION UpdateRM(FIRST_T_P, LAST_T_P)
