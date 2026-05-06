#ifndef MAIN_H
#define MAIN_H

#include "ConfigManager.h"
#include "WiFiManager.h"
#include "MQTTService.h"
#include "IOBoard.h"
#include "DAC7678Manager.h"
#include "ADS1015Manager.h"


#include "TimeService.h"
#include "CANService.h"

#include "IOExpander.h"

#define AGENT_MODEL_MAX_LEN 32
#define AGENT_VERSION_MAX_LEN 16
#define AGENT_TIME_MAX_LEN 24
#define AGENT_SERIAL_MAX_LEN 32
#define BROKER_NAME_MAX_LEN 64
#define W5_IP_LEN 4
#define W5_MAC_LEN 6
#define DAC_ONOFF_LEN 13
#define AUTO_FLAGS_LEN 21
#define ACC_FLAGS_LEN 4
#define FLASH_FLAGS_LEN 4
#define ADC_ARRAY_LEN 16
#define ADC_N_ARRAY_LEN 16
#define THERMOCOUPLE_LEN 20
#define THERMISTOR_LEN 20
#define PHOTOCOPLER_LEN 15
#define TTLINPUT_LEN 3
#define CONTACTIN_LEN 3
#define DAC_CTL_V_LEN 16
#define MCU_DAC_LEN 2
#define FET_LEN 24
#define RELAY_LEN 3
#define SSR_LEN 7
#define TR_LEN 4
#define PWM_LEN 8
#define BUFFER_24V_LEN 2
#define INV_LEN 20
#define COGEN_MFM_LEN 6
#define FET_EXP_0_LEN 8
#define PCO_EXP_0_LEN 4
#define TH_EXP_0_LEN 8
#define FET_EXP_1_LEN 8
#define PCO_EXP_1_LEN 4
#define TH_EXP_1_LEN 8
#define EXP_1_COND_LEN 3
#define SULF_SENS_LEN 2
#define FET_BAGO_LEN 7
#define WARNING_LEN 6
#define SHUTDOWN_LEN 6
#define SD_EN_LEN 6
#define ERR_HIS_LEN 6
#define DEMO_SD_LEN 3
#define CONSUMP_DAYS_LEN 3

// 시스템 정보 구조체
typedef struct {
    int hw_bd_id_num;  // 하드웨어 보드 ID 번호 (ioEx_IO 입력핀 0,1,2,3에서 읽음)
} system_t;

typedef struct {
    char Model[AGENT_MODEL_MAX_LEN];
    char Version[AGENT_VERSION_MAX_LEN];
    char Current_Time[AGENT_TIME_MAX_LEN];
    int wday;
    char Time_crc[32];
    int wday_crc;
    char JSONVersion[8];
    char SerialNumber[AGENT_SERIAL_MAX_LEN];
    uint8_t w5_ip[W5_IP_LEN];
    uint8_t w5_mac[W5_MAC_LEN];
    uint8_t w5_gw[W5_IP_LEN];
    uint8_t w5_sn[W5_IP_LEN];
    uint8_t w5_dns[W5_IP_LEN];
    uint8_t brk_ip_dir[W5_IP_LEN];
    int brk_port_dir;
    uint8_t brk_ip_dns[W5_IP_LEN];
    int brk_port_dns;
    int brk_isdns;
    char brk_name[BROKER_NAME_MAX_LEN];
    uint8_t day[6];
    uint8_t month[6];
    uint16_t year[6];
    int BootCNT;
    int SaveCNT;
    int FlashSIZE;
    int SEQ_no;
    int SEQ_time;
    int antif_s_no;
    int antif_s_time;
    int rm_onoff;
    int dac_onoff[DAC_ONOFF_LEN];
    int auto_flags[AUTO_FLAGS_LEN];
    int rtc_flag;
    int p101_c_stat;
    int p101_lo_duty;
    int pit201_d_min;
    int pit201_d_max;
    int burner_mode;
    int line_onoff;
    int line_mode;
    int stack_load;
    int target_load;
    float st05;
    float st11;
    float st16;
    float sthex;
    float st302;
    float tt301_aim;
    int pit201_sp;
    int p301_hz_sp;
    int acc_flags[ACC_FLAGS_LEN];
    int flash_flags[FLASH_FLAGS_LEN];
    int nc309_egen;
    uint8_t vcntn[6];
} AgentInfo_t;
typedef struct {
    float mcutemp;
    char RUN_Time[16];
    int devid;
    int msg_length;
    float adc_f[ADC_ARRAY_LEN];
    float adc_v[ADC_ARRAY_LEN];
    float adc_n[ADC_N_ARRAY_LEN];
    float s_adc_v[ADC_ARRAY_LEN];
    float s_adc_n[ADC_N_ARRAY_LEN];
    float thermocouple[THERMOCOUPLE_LEN];
    float thermistor[THERMISTOR_LEN];
    int photocoupler[PHOTOCOPLER_LEN];
    int ttlinput[TTLINPUT_LEN];
    int contactin[CONTACTIN_LEN];
    float dac_ctl[DAC_CTL_V_LEN];
    float dac_ctl_v[DAC_CTL_V_LEN];
    float dac_12bit[DAC_CTL_V_LEN];
    float mcudac[MCU_DAC_LEN];
    int p202_duty;
    int fet[FET_LEN];
    int relay[RELAY_LEN];
    int ssr[SSR_LEN];
    int tr[TR_LEN];
    int pwm[PWM_LEN];
    int buffer_24v[BUFFER_24V_LEN];
    int inv[INV_LEN];
    float stack_i;
    float cogen_mfm[COGEN_MFM_LEN];
    int heat_gen;
    float elec_eff;
    float heat_eff;
    int p202_drv;
    int p203_mf_t_v;
    int p301_hz;
    int fet_exp_0[FET_EXP_0_LEN];
    int pco_exp_0[PCO_EXP_0_LEN];
    float th_exp_0[TH_EXP_0_LEN];
    int pci_exp_0;
    int fet_exp_1[FET_EXP_1_LEN];
    int pco_exp_1[PCO_EXP_1_LEN];
    float th_exp_1[TH_EXP_1_LEN];
    int pci_exp_1;
    int exp_1_cond[EXP_1_COND_LEN];
    int sulf_sens[SULF_SENS_LEN];
    int fet_bopdrv[FET_BAGO_LEN];
    int wcogen_c;
} GlobalMeasure_t;
typedef struct {
    int d_fet[2];
    int d_ssr[2];
    int d_relay[2];
    int d_tr[2];
    int d_ttl[2];
    int d_photoc[2];
    int d_adc[2];
    int adc[2];
    int thermocouple[2];
    int thermistor[2];
    int t_result[2];
} Result_t;
typedef struct {
    int ERR_code;
    int Warning[WARNING_LEN];
    int ShutDown[SHUTDOWN_LEN];
    int SD_EN[SD_EN_LEN];
    int ERR_his[ERR_HIS_LEN];
    int DEMO_SD[DEMO_SD_LEN];
    int demo_ok;
    int VCNTW;
    int VCNTS;
    int VCNTH;
    int consump_days[CONSUMP_DAYS_LEN];
} WnSD_t;
typedef struct {
    AgentInfo_t Agent;
    GlobalMeasure_t GlobalMeasure;
    Result_t Result;
    WnSD_t WnSD;
} LogData_t;

extern ConfigManager configManager;
extern WiFiManager wifiManager;
extern MQTTService mqttService;
extern IOBoard ioBoard;
extern IOBoard ioBoardBackup;
extern DAC7678Manager dac7678Manager;
extern ADS1015Manager ads1015Manager;
extern system_t rSys;  // 시스템 정보 전역 변수
extern bool g_i2cError;            // true이면 I²C 경로가 문제가 생겼음

#endif
