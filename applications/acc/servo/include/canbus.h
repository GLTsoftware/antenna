#define GET_ACU_MODE  0x00040022
#define GET_AZ_POSN   0x00040012
#define GET_EL_POSN   0x00040002
#define GET_ACU_ERROR 0x0004002F

#define ACU_MODE_CMD	0x041022
#define AZ_TRAJ_CMD	0x041012
#define CLEAR_FAULT_CMD	0x041021
#define EL_TRAJ_CMD	0x041002
#define RESET_ACU_CMD	0x04102F
#define SET_AZ_AUX_MODE	0x041016
#define SET_AZ_BRAKE	0x041014
#define SET_AZ_SERVO_COEFF_0	0x042020
#define SET_AZ_SERVO_DEFAULT	0x041017
#define SET_EL_AUX_MODE	0x041006
#define SET_EL_BRAKE	0x041004
#define SET_EL_SERVO_COEFF_0	0x042010
#define SET_EL_SERVO_DEFAULT	0x041007
#define SET_IDLE_STOW_TIME	0x041025
#define SET_SHUTTER	0x04102E
#define SET_STOW_PIN	0x04102D

extern void SetupCanBus(void);
extern int ReadCANValue(int msgID, void *value, int len);
extern int SetCANValue(int msgID, void *data, int len);
