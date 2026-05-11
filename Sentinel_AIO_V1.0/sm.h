#ifndef SM_H
#define SM_H

#include <stdint.h>
#include <stdbool.h>


#define STM_CREDENTIAL_SSID_SIZE     32
#define STM_CREDENTIAL_PASSWORD_SIZE 63

/* ── States ─────────────────────────────────────────────── */
typedef enum {
    SM_STATE_INIT,
    SM_STATE_CHARGING,
    SM_STATE_POWER_STM,
    SM_STATE_IDLE,
    SM_STATE_CRITICAL_FAULT
} SM_State_t;

/* ── Wake reason (drives STM_MCU_IO1) ───────────────────── */
typedef enum {
    SM_WAKE_NORMAL,
    SM_WAKE_SETUP
} SM_WakeReason_t;

/* ── Fault source (drives CRITICAL_FAULT behaviour) ─────── */
typedef enum {
    SM_FAULT_NONE,
    SM_FAULT_I2C_BUS,         // I2C failed at INIT — skip I2C in fault state
    SM_FAULT_GAUGE,           // Safety status flagged by gauge
    SM_FAULT_CHARGER,         // Safety status flagged by charger
    SM_FAULT_INIT_FAILED      // Generic init failure
} SM_FaultSource_t;
/* ── Helper Structs ──────────────────────────────────────── */
typedef struct {
    uint16_t vbus_mv;
    uint16_t vbat_mv;
    uint8_t stat1;
    bool adapter_present;
    uint8_t chg_stat;
    bool charger_done;
    bool is_critical_low;
    bool is_charging;
} SM_PowerContext_t;

typedef struct {
    uint16_t year;                    // full year (e.g. 2026)
    uint8_t  month;                   // 1-12
    uint8_t  day;                     // 1-31
    uint8_t  hour;                    // 0-23
    uint8_t  minute;                  // 0-59
    uint8_t  second;                  // 0-59
 
} SM_RTCConfig_t;

typedef struct {
       uint8_t  wake_interval_minutes;   // wake period ;
} SM_PeriodConfig_t;

typedef struct {
    uint16_t vreg_mV;       // Charge regulation voltage
    uint16_t ichg_mA;       // Fast charge current 
    uint16_t iindpm_mA;     // Input current limit (IINDPM)
    uint16_t vindpm_mV;     // Input voltage limit (VINDPM)  
    uint16_t vsysmin_mV;    // System minimum voltage  
    uint16_t iprechg_mA;    // Pre-charge current            
    uint16_t iterm_mA;      // Termination current       
} SM_ChargerConfig_t;

typedef struct {
    uint8_t mode;  // 0=LTE, 1=WiFi
} SM_STMConnectivity_t;

typedef struct {
    uint8_t communication;    // 0=USART, 1=USB
    uint8_t baudrate_index;   
    uint8_t network_provider; // 0=Roaming, 1=Local
} SM_STMLte_t;

typedef struct {
    uint8_t resolution;
    uint8_t framerate;
    uint8_t compression;
} SM_STMCamera_t;

typedef struct {
    uint8_t log_to_card;
    uint8_t log_to_usart;
} SM_STMLogging_t;

typedef struct {
    SM_STMConnectivity_t connectivity;
    SM_STMLte_t          lte;
    SM_STMCamera_t       camera;
    SM_STMLogging_t      logging;
} SM_STMConfig_t;

typedef struct {
    char ap_ssid        [STM_CREDENTIAL_SSID_SIZE];
    char ap_password    [STM_CREDENTIAL_PASSWORD_SIZE];
    char device_name    [STM_CREDENTIAL_SSID_SIZE];
    char device_password[STM_CREDENTIAL_PASSWORD_SIZE];
} SM_STMCredentials_t;

/* ── Context ─────────────────────────────────────────────── */
typedef struct {
    SM_State_t       current;
    SM_State_t       previous;
    SM_WakeReason_t  wake_reason;
    SM_FaultSource_t fault_source;
    bool             sm_paused;
    bool             entry_done;
    uint32_t         minute_counter;
    uint32_t         second_counter;
    uint32_t         stm_power_on_s;
    uint32_t         sleep_entry_minute;   
    uint32_t         last_safety_check_s;     
    uint32_t         fault_retry_s; 
    bool             stm_data_sent;
    uint32_t         last_io2_activity_s;  
    uint32_t         last_stm_periodic_minute;
    bool             critical_msg_sent;
    uint32_t         last_charging_tick;
    SM_ChargerConfig_t   sm_charger_config;
    SM_RTCConfig_t       sm_rtc_config;
    SM_STMConfig_t      stm_config;
    SM_STMCredentials_t stm_credentials;
    SM_PeriodConfig_t   stm_wake_period;
    bool                stm_config_received;
    bool                stm_credentials_received;
    bool                wake_interval_configured;
    bool                charger_configured;
    bool                has_pending_response;
} SM_Context_t;

extern SM_Context_t sm_context;


/* ── Public API ──────────────────────────────────────────── */
void        SM_Init(void);
void        SM_Run(void);
SM_State_t  SM_GetState(void);
const char* SM_GetStateString(void);
void        SM_Transition(SM_State_t new_state);
bool SM_SafetyCheck(void);
bool SM_ChargingSafetyCheck(void);
void RTC_DisablePrescaler(void);
void SM_EnablePrescaler(void);


void RTC_GetTime(SM_RTCConfig_t *out);
bool RTC_SetTime(const SM_RTCConfig_t *in);

#endif
