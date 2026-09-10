#ifndef SM_PROTOCOL_H
#define SM_PROTOCOL_H

#include <stdint.h>
#include "sm.h"

/* ── Message Types ─────────────────────────────────────────────────────── */
typedef enum {
    MSG_OFFER    = 0x01,   // MSPM0 → STM32 : "Ready, here is my state"
    MSG_REQUEST  = 0x02,   // STM32 → MSPM0 : "Send me this"
    MSG_DATA     = 0x03,   // MSPM0 → STM32 : response to REQUEST
    MSG_CONFIG   = 0x04,   // STM32 → MSPM0 : apply config
    MSG_ACK      = 0x05,   // MSPM0 → STM32 : done
    MSG_NACK     = 0x06,   // MSPM0 → STM32 : failed
    MSG_SHUTDOWN = 0x07    // STM32 → MSPM0 : power me down
} SM_MsgType_t;

/* ── Payload Identifiers ───────────────────────────────────────────────── */
typedef enum {
    PID_TELEMETRY   = 0x01,
    PID_RTC_GET     = 0x02,
    PID_RTC_SET     = 0x03,
    PID_CHARGER_CFG = 0x04,
    PID_PERIOD_SET  = 0x05,
    PID_STM_CFG     = 0x06,
    PID_STM_CREDENTIALS = 0x07,
    PID_KEEP_ALIVE  = 0x08,
    PID_POWER_CYCLE = 0x09,
    PID_AE_SEED     = 0x0A,
    PID_STREAM_SEED = 0x0B,
    PID_ILLUM_CFG   = 0x0C,
    PID_WAKE_REASON = 0x0D
} SM_PayloadId_t;

#define SM_AE_SEED_FSBL_MAGIC_0  0xA5U
#define SM_AE_SEED_FSBL_MAGIC_1  0x5AU
#define SM_AE_SEED_FSBL_MAGIC_2  0xAEU
#define SM_AE_SEED_FSBL_MAGIC_3  0x5DU
#define SM_AE_SEED_FSBL_TICK_OFFSET  4U

/* ── Packet Header (4 bytes) ───────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     // SM_MsgType_t
    uint8_t  payload_id;   // SM_PayloadId_t
    uint16_t length;       // bytes of payload that follows
} SM_MsgHeader_t;

/* ── Payload types ─────────────────────────────────────────────────────── */
typedef SM_RTCConfig_t     SM_RtcConfigPayload_t;
typedef SM_ChargerConfig_t SM_ChargerConfigPayload_t;
typedef SM_STMConfig_t  SM_STMConfigPayload_t ;
typedef SM_STMCredentials_t SM_STMCredentialsPayload_t;
typedef SM_PeriodConfig_t SM_PeriodConfigPayload_t;

typedef struct {
    char json[508];        // null-terminated JSON (header takes 4 bytes → 508 left)
} SM_TelemetryPayload_t;

typedef SM_RTCConfig_t     SM_RtcDataPayload_t;

typedef struct {
    uint8_t reserved;
} SM_AckPayload_t;

typedef struct {
    uint8_t reserved;
} SM_KeepAlivePayload_t;

typedef struct __attribute__((packed)) {
    uint32_t exposure_us;      /* score_exposure_sep() → IMX335_SetExposure()  */
    uint32_t gain_mdB;         /* score_gain_sep()     → IMX335_SetGain()      */
    uint32_t awb_color_temp;   /* must equal an ISP referenceColorTemp entry   */
    uint32_t lux_milli;        /* measured lux x1000 — logging / retraining    */
    uint16_t als_ch0;          /* raw visible+IR                               */
    uint16_t als_ch1;          /* raw IR                                       */
    uint16_t als_settle_ms;    /* extra wait needed for the ALS conversion;    */
    uint8_t  valid;            /* 0 = no usable reading; ignore everything above */
    uint8_t  als_gain;         /* ALS gain the counts were taken at (1..96)    */
    uint32_t pir_elapsed_ms;
} SM_AeSeedPayload_t;

typedef struct __attribute__((packed)) {
    uint32_t exposure_us;     
    uint32_t gain_mdB;        
    uint32_t awb_color_temp;   /* exact ISP referenceColorTemp[] entry         */
    uint32_t lux_milli;        /* lux x1000, always matching led_on: ambient */
    uint8_t  led_on;           /* CONFIRMED physical state, not a request      */
    uint8_t  valid;            /* 0 = no usable set-point from this sample;
                                * hold the previous one, do not apply zeros.
                                * The only way a bad light reading is reported  */
} SM_StreamSeedPayload_t;

typedef SM_IllumConfig_t SM_IllumConfigPayload_t;

typedef struct __attribute__((packed)) {
    uint8_t        wake_reason;     /* SM_WakeReason_t: 0=NORMAL, 1=SETUP, 2=PIR, 3=LIFELINE */
    uint8_t        wake_mode;       /* 0=Periodic, 1=PIR */
    uint8_t        low_battery;     /* 1 = Critical low (< 3.0V, limited power budget), 0 = Normal */
    uint8_t        reserved;        /* Alignment padding for 16-bit aligned time.year */
    SM_RTCConfig_t time;            /* Current RTC calendar time (year, month, day, hour, min, sec) */
} SM_WakeReasonPayload_t;

/* ── Full 512-byte SPI packet overlay ──────────────────────────────────── */
typedef union {
    uint8_t raw[512];

    struct __attribute__((packed)) {
        SM_MsgHeader_t header;
        union {
            SM_RtcConfigPayload_t     rtc_config;
            SM_ChargerConfigPayload_t charger_config;
            SM_TelemetryPayload_t     telemetry;
            SM_RtcDataPayload_t       rtc_data;
            SM_AckPayload_t           ack;
            SM_STMConfigPayload_t     stm_config;
            SM_STMCredentialsPayload_t  stm_credentials;  
            SM_PeriodConfigPayload_t    stm_wake_period;  
            SM_KeepAlivePayload_t     keep_alive;
            SM_AeSeedPayload_t        ae_seed;
            SM_StreamSeedPayload_t    stream_seed;
            SM_IllumConfigPayload_t   illum_config;
            SM_WakeReasonPayload_t    wake_reason;
            uint8_t                   raw_payload[508];
        } payload;
    } pkt;
} SM_SpiPacket_t;

#endif // SM_PROTOCOL_H