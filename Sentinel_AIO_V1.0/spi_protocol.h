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
    PID_STM_CREDENTIALS = 0x07
} SM_PayloadId_t;

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
            uint8_t                   raw_payload[508];
        } payload;
    } pkt;
} SM_SpiPacket_t;

#endif // SM_PROTOCOL_H