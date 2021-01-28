/* response to command "171" to get level 2 data from radiometer */
/* See RPG_MWR_STD_Software_Manual_2015.pdf page 114, section 4.26.2 */
typedef struct __attribute__ ((packed)) radiometerData {
    char commandID;
    int totalLength;
    short flags;
    int lengthATN;
    int timeATN;
    char ATN_RF;
    int noATN;
    float ATN_El[1];
    float ATN_Az[1];
    float ATN_Frq[1];
    float ATN[1];
} radiometerData; 

typedef struct __attribute__ ((packed)) radiometerStatus {
    char commandID;
    int totalLength;
    int timeRAD;
    char RAD_Stat;
    char Cal_Stat;
    char Axis_Stat;
} radiometerStatus;

typedef struct __attribute__ ((packed)) radiometerHouseKeeping {
    char commandID;
    int totalLength;
    char Available;
    int timeHKD;
    char HKD_Alarm;
    char Flags;
    float HKD_Long;
    float HKD_Lat;
    float HKD_Amb1T;
    float HKD_Amb2T;
    float HKD_Rec1T;
    float HKD_Rec2T;
    float HKD_Rec1St;
    float HKD_Rec2St;
    int HKD_DskCap;
    int HKD_Qual;
    int HKD_Dig;
} radiometerHouseKeeping;
