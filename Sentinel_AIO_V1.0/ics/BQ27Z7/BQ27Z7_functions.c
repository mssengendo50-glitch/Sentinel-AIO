/*
 * BQ27Z746 High-Level Driver
 * Sits on top of gauge.c / gauge.h (mid-level) and HAL/i2c.h (hardware)
 *
 * Layer structure:
 *   BQ27Z746_functions.c  ← this file
 *       ↓
 *   gauge.c / gauge.h     (gauge_cmd_read, gauge_write, gauge_read, etc.)
 *       ↓
 *   HAL/i2c.c / i2c.h     (I2C_ReadDevice, I2C_WriteDevice)
 */

#include "BQ27Z7_functions.h"
#include <string.h>

// ================================================================
// Internal telemetry cache
// ================================================================
static BQ27Z746_Telemetry_t g_telem = {0};

// ================================================================
// Internal helper: raw 16-bit register read
// Returns the raw uint16_t from the gauge standard command register.
// ================================================================
static uint16_t read_reg16(I2C_Regs *i2c, uint8_t reg)
{
    return (uint16_t)gauge_cmd_read(i2c, reg);
}

// ================================================================
// Internal helper: Kelvin (0.1 K units) to Celsius
// BQ27Z746 reports temperature as tenths of Kelvin (e.g. 2981 = 25.1 C)
// ================================================================
static int16_t kelvin_to_celsius(uint16_t raw_01K)
{
    return (int16_t)((int32_t)raw_01K / 10 - 273);
}

// ================================================================
// MAC Layer
// ================================================================

/*
 * BQ27Z746_MAC_Read
 *
 * Protocol (from Zephyr driver / BQ27Z746 TRM):
 * 1. Write the 16-bit MAC command to ALTMANUFACTURERACCESS (0x3E)
 * 2. Read 36 bytes starting from ALTMANUFACTURERACCESS:
 *      [0..1]  = echoed command word (little-endian) — used for verification
 *      [2..33] = payload data (up to 32 bytes)
 *      [34]    = checksum: 0xFF - (sum of bytes [0..33])
 *      [35]    = total length including cmd + data + overhead
 * 3. Verify echoed command matches what was sent
 * 4. Verify checksum
 * 5. Return payload bytes [2..2+datalen] to caller
 */
bool BQ27Z746_MAC_Read(I2C_Regs *i2c, uint16_t cmd, uint8_t *pData, uint8_t *pLen)
{
    uint8_t frame[BQ27Z746_MAC_FRAME_LEN];

    /* Step 1: write command to ALTMANUFACTURERACCESS */
    uint8_t cmd_bytes[2];
    cmd_bytes[0] = (uint8_t)(cmd & 0xFFu);
    cmd_bytes[1] = (uint8_t)((cmd >> 8u) & 0xFFu);

    if (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, cmd_bytes, 2) != 2)
        return false;

    /* Step 2: read back the full 36-byte frame */
    if (gauge_read(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, frame, BQ27Z746_MAC_FRAME_LEN)
            != BQ27Z746_MAC_FRAME_LEN)
        return false;

    /* Step 3: verify echoed command (little-endian in bytes [0..1]) */
    uint16_t echoed_cmd = (uint16_t)(frame[0] | ((uint16_t)frame[1] << 8u));
    if (echoed_cmd != cmd)
        return false;

    /* Step 4: verify checksum
     * checksum = 0xFF - (sum of bytes [0..33])
     * byte [34] holds the checksum written by the gauge */
    uint8_t num_bytes = frame[35] - 2u;  // checksum covers (length - 2) bytes
    uint8_t sum = 0u;
    for (uint8_t i = 0u; i < num_bytes; i++)
        sum += frame[i];
    uint8_t expected_checksum = (uint8_t)(0xFFu - sum);
    if (expected_checksum != frame[34])
        return false;

    /* Step 5: extract payload
     * byte [35] = total length (cmd bytes + data + overhead)
     * actual data length = frame[35] - BQ27Z746_MAC_OVERHEAD */
    uint8_t data_len = frame[35] - (uint8_t)BQ27Z746_MAC_OVERHEAD;
    if (data_len > BQ27Z746_MAC_DATA_LEN)
        data_len = BQ27Z746_MAC_DATA_LEN;

    memcpy(pData, &frame[2], data_len);
    if (pLen != NULL)
        *pLen = data_len;

    return true;
}

/*
 * BQ27Z746_MAC_Send
 * For commands that trigger an action and return no data (reset, seal, etc.)
 */
bool BQ27Z746_MAC_Send(I2C_Regs *i2c, uint16_t cmd)
{
    uint8_t cmd_bytes[2];
    cmd_bytes[0] = (uint8_t)(cmd & 0xFFu);
    cmd_bytes[1] = (uint8_t)((cmd >> 8u) & 0xFFu);
    return (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, cmd_bytes, 2) == 2);
}

/*
 * BQ27Z746_MAC_Write
 *
 * Protocol (for data-bearing MAC commands):
 * 1. Build frame: [cmd_lo, cmd_hi, data..., checksum, length]
 * 2. Checksum = 0xFF - sum(cmd bytes + data bytes)
 * 3. Length   = 2 (cmd) + data_len + 2 (checksum + length field)
 * 4. Write entire frame to ALTMANUFACTURERACCESS (0x3E)
 *
 * Note: For command-only operations use BQ27Z746_MAC_Send instead.
 */
bool BQ27Z746_MAC_Write(I2C_Regs *i2c, uint16_t cmd, uint8_t *pData, uint8_t data_len)
{
    if (data_len == 0u || data_len > BQ27Z746_MAC_DATA_LEN)
        return false;

    uint8_t frame[BQ27Z746_MAC_FRAME_LEN];
    uint8_t frame_idx = 0u;

    /* Bytes [0..1]: command word little-endian */
    frame[frame_idx++] = (uint8_t)(cmd & 0xFFu);
    frame[frame_idx++] = (uint8_t)((cmd >> 8u) & 0xFFu);

    /* Bytes [2..2+data_len]: payload */
    for (uint8_t i = 0u; i < data_len; i++)
        frame[frame_idx++] = pData[i];

    /* Checksum: 0xFF - sum(cmd bytes + data bytes) */
    uint8_t sum = 0u;
    for (uint8_t i = 0u; i < frame_idx; i++)
        sum += frame[i];
    frame[frame_idx++] = (uint8_t)(0xFFu - sum);

    /* Length: cmd(2) + data + checksum(1) + length(1) */
    frame[frame_idx++] = (uint8_t)(2u + data_len + 2u);

    return (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, frame, frame_idx) == frame_idx);
}


// ================================================================
// Diagnostic reads (MAC-based)
// ================================================================

bool BQ27Z746_GetDeviceType(I2C_Regs *i2c, uint16_t *pType)
{
    uint8_t data[BQ27Z746_MAC_DATA_LEN];
    uint8_t len = 0u;
    if (!BQ27Z746_MAC_Read(i2c, BQ27Z746_MAC_DEVICETYPE, data, &len))
        return false;
    if (len < 2u) return false;
    *pType = (uint16_t)(data[0] | ((uint16_t)data[1] << 8u));
    return true;
}

bool BQ27Z746_GetFirmwareVersion(I2C_Regs *i2c, uint16_t *pVersion)
{
    uint8_t data[BQ27Z746_MAC_DATA_LEN];
    uint8_t len = 0u;
    if (!BQ27Z746_MAC_Read(i2c, BQ27Z746_MAC_FIRMWAREVERSION, data, &len))
        return false;
    if (len < 2u) return false;
    *pVersion = (uint16_t)(data[0] | ((uint16_t)data[1] << 8u));
    return true;
}

bool BQ27Z746_GetChemID(I2C_Regs *i2c, uint16_t *pChemID)
{
    uint8_t data[BQ27Z746_MAC_DATA_LEN];
    uint8_t len = 0u;
    if (!BQ27Z746_MAC_Read(i2c, BQ27Z746_MAC_CHEMID, data, &len))
        return false;
    if (len < 2u) return false;
    *pChemID = (uint16_t)(data[0] | ((uint16_t)data[1] << 8u));
    return true;
}

bool BQ27Z746_GetOperationStatus(I2C_Regs *i2c, uint32_t *pStatus)
{
    uint8_t data[BQ27Z746_MAC_DATA_LEN];
    uint8_t len = 0u;
    if (!BQ27Z746_MAC_Read(i2c, BQ27Z746_MAC_OPERATIONSTATUS, data, &len))
        return false;
    if (len < 4u) return false;
    *pStatus = (uint32_t)(data[0])
             | ((uint32_t)data[1] << 8u)
             | ((uint32_t)data[2] << 16u)
             | ((uint32_t)data[3] << 24u);
    return true;
}

bool BQ27Z746_GetChargingStatus(I2C_Regs *i2c, uint8_t *pTempRange, uint16_t *pChgStatus)
{
    uint8_t data[BQ27Z746_MAC_DATA_LEN];
    uint8_t len = 0u;

    if (!BQ27Z746_MAC_Read(i2c, BQ27Z746_MAC_CHARGINGSTATUS, data, &len))
        return false;
    if (len < 3u) return false;
    *pTempRange = data[0];
    *pChgStatus = (uint16_t)(data[1] | ((uint16_t)data[2] << 8u));

    return true;
}

bool BQ27Z746_GetGaugingStatus(I2C_Regs *i2c, uint32_t *pStatus)
{
    uint8_t data[BQ27Z746_MAC_DATA_LEN];
    uint8_t len = 0u;
    if (!BQ27Z746_MAC_Read(i2c, BQ27Z746_MAC_GAUGINGSTATUS, data, &len))
        return false;
    if (len < 4u) return false;
    *pStatus = (uint32_t)(data[0])
             | ((uint32_t)data[1] << 8u)
             | ((uint32_t)data[2] << 16u)
             | ((uint32_t)data[3] << 24u);
    return true;
}

bool BQ27Z746_GetSafetyStatus(I2C_Regs *i2c, uint32_t *pStatus)
{
    uint8_t data[BQ27Z746_MAC_DATA_LEN];
    uint8_t len = 0u;
    if (!BQ27Z746_MAC_Read(i2c, BQ27Z746_MAC_SAFETYSTATUS, data, &len))
    {
        return false;
    }
    if (len < 4u) 
    {
        return false;
    }

    // Assemble Little-Endian
    *pStatus = (uint32_t)(data[0])
             | ((uint32_t)data[1] << 8u)
             | ((uint32_t)data[2] << 16u)
             | ((uint32_t)data[3] << 24u);

    return true;
}

// ================================================================
// Init
// ================================================================

bool BQ27Z746_Init(I2C_Regs *i2c)
{
    gauge_address(i2c, GAUGE_I2C_ADDR);

    uint16_t device_type = 0u;
    if (!BQ27Z746_GetDeviceType(i2c, &device_type))
        return false;
    return true;
}

// ================================================================
// Live register reads
// ================================================================

uint16_t BQ27Z746_ReadVoltage_mV(I2C_Regs *i2c)
{
    return read_reg16(i2c, BQ27Z746_REG_VOLTAGE);
}

int16_t BQ27Z746_ReadCurrent_mA(I2C_Regs *i2c)
{
    return (int16_t)read_reg16(i2c, BQ27Z746_REG_CURRENT);
}

int16_t BQ27Z746_ReadAvgCurrent_mA(I2C_Regs *i2c)
{
    return (int16_t)read_reg16(i2c, BQ27Z746_REG_AVERAGECURRENT);
}

uint8_t BQ27Z746_ReadSOC_pct(I2C_Regs *i2c)
{
    return (uint8_t)read_reg16(i2c, BQ27Z746_REG_RELATIVESTATEOFCHARGE);
}

uint16_t BQ27Z746_ReadRemainingCap_mAh(I2C_Regs *i2c)
{
    return read_reg16(i2c, BQ27Z746_REG_REMAININGCAPACITY);
}

uint16_t BQ27Z746_ReadFullChargeCap_mAh(I2C_Regs *i2c)
{
    return read_reg16(i2c, BQ27Z746_REG_FULLCHARGECAPACITY);
}

uint8_t BQ27Z746_ReadStateOfHealth_pct(I2C_Regs *i2c)
{
    return (uint8_t)read_reg16(i2c, BQ27Z746_REG_STATEOFHEALTH);
}

int16_t BQ27Z746_ReadTemperature_C(I2C_Regs *i2c)
{
    return kelvin_to_celsius(read_reg16(i2c, BQ27Z746_REG_TEMPERATURE));
}

int16_t BQ27Z746_ReadInternalTemp_C(I2C_Regs *i2c)
{
    return kelvin_to_celsius(read_reg16(i2c, BQ27Z746_REG_INTERNALTEMPERATURE));
}

uint16_t BQ27Z746_ReadTimeToEmpty_min(I2C_Regs *i2c)
{
    return read_reg16(i2c, BQ27Z746_REG_AVERAGETIMETOEMPTY);
}

uint16_t BQ27Z746_ReadTimeToFull_min(I2C_Regs *i2c)
{
    return read_reg16(i2c, BQ27Z746_REG_AVERAGETIMETOFULL);
}

uint16_t BQ27Z746_ReadCycleCount(I2C_Regs *i2c)
{
    return read_reg16(i2c, BQ27Z746_REG_CYCLECOUNT);
}

int16_t BQ27Z746_ReadAvgPower_mW(I2C_Regs *i2c)
{
    return (int16_t)read_reg16(i2c, BQ27Z746_REG_AVERAGEPOWER);
}

uint16_t BQ27Z746_ReadBatteryStatus(I2C_Regs *i2c)
{
    return read_reg16(i2c, BQ27Z746_REG_BATTERYSTATUS);
}

// ================================================================
// Cache: UpdateTelemetry + getters
// ================================================================

void BQ27Z746_UpdateTelemetry(I2C_Regs *i2c)
{
    g_telem.voltage_mV         = BQ27Z746_ReadVoltage_mV(i2c);
    g_telem.current_mA         = BQ27Z746_ReadCurrent_mA(i2c);
    g_telem.avgCurrent_mA      = BQ27Z746_ReadAvgCurrent_mA(i2c);
    g_telem.soc_pct            = BQ27Z746_ReadSOC_pct(i2c);
    g_telem.remainingCap_mAh   = BQ27Z746_ReadRemainingCap_mAh(i2c);
    g_telem.fullChargeCap_mAh  = BQ27Z746_ReadFullChargeCap_mAh(i2c);
    g_telem.stateOfHealth_pct  = BQ27Z746_ReadStateOfHealth_pct(i2c);
    g_telem.temperature_C      = BQ27Z746_ReadTemperature_C(i2c);
    g_telem.internalTemp_C     = BQ27Z746_ReadInternalTemp_C(i2c);
    g_telem.timeToEmpty_min    = BQ27Z746_ReadTimeToEmpty_min(i2c);
    g_telem.timeToFull_min     = BQ27Z746_ReadTimeToFull_min(i2c);
    g_telem.cycleCount         = BQ27Z746_ReadCycleCount(i2c);
    g_telem.avgPower_mW        = BQ27Z746_ReadAvgPower_mW(i2c);
    g_telem.batteryStatus      = BQ27Z746_ReadBatteryStatus(i2c);
}

uint16_t BQ27Z746_Get_Voltage_mV(void)        { return g_telem.voltage_mV; }
int16_t  BQ27Z746_Get_Current_mA(void)        { return g_telem.current_mA; }
int16_t  BQ27Z746_Get_AvgCurrent_mA(void)     { return g_telem.avgCurrent_mA; }
uint8_t  BQ27Z746_Get_SOC_pct(void)           { return g_telem.soc_pct; }
uint16_t BQ27Z746_Get_RemainingCap_mAh(void)  { return g_telem.remainingCap_mAh; }
uint16_t BQ27Z746_Get_FullChargeCap_mAh(void) { return g_telem.fullChargeCap_mAh; }
uint8_t  BQ27Z746_Get_StateOfHealth_pct(void) { return g_telem.stateOfHealth_pct; }
int16_t  BQ27Z746_Get_Temperature_C(void)     { return g_telem.temperature_C; }
int16_t  BQ27Z746_Get_InternalTemp_C(void)    { return g_telem.internalTemp_C; }
uint16_t BQ27Z746_Get_TimeToEmpty_min(void)   { return g_telem.timeToEmpty_min; }
uint16_t BQ27Z746_Get_TimeToFull_min(void)    { return g_telem.timeToFull_min; }
uint16_t BQ27Z746_Get_CycleCount(void)        { return g_telem.cycleCount; }
int16_t  BQ27Z746_Get_AvgPower_mW(void)       { return g_telem.avgPower_mW; }
uint16_t BQ27Z746_Get_BatteryStatus(void)     { return g_telem.batteryStatus; }

// ================================================================
// Status bit helpers (operate on cached BatteryStatus)
// ================================================================

bool BQ27Z746_IsFullyCharged(void)
{
    return (g_telem.batteryStatus & BQ27Z746_STATUS_FC) != 0u;
}

bool BQ27Z746_IsFullyDischarged(void)
{
    return (g_telem.batteryStatus & BQ27Z746_STATUS_FD) != 0u;
}

bool BQ27Z746_IsDischarging(void)
{
    return (g_telem.batteryStatus & BQ27Z746_STATUS_DSG) != 0u;
}

// ================================================================
// Golden Image wrapper (one-liner usage)
// ================================================================
bool BQ27Z746_LoadGoldenImage(I2C_Regs *i2c, const char *fs_string)
{
    char *result = gauge_execute_fs(i2c, (char *)fs_string);
    return (result == NULL || *result == '\0');
}


#define FET_OPTIONS_ADDR 0x45C0
#define DF_FRAME_SIZE    34  


bool BQ27Z746_GetFETOptions(I2C_Regs *i2c, uint16_t *pOutValue)
{
    uint8_t tx_addr[2];
    uint8_t rx_frame[36];
    // 1. Point the gauge to the DF address
    tx_addr[0] = (uint8_t)(FET_OPTIONS_ADDR & 0xFF);
    tx_addr[1] = (uint8_t)((FET_OPTIONS_ADDR >> 8) & 0xFF);

    if (gauge_write(i2c, 0x3E, tx_addr, 2) != 2)
        return false;
    DL_Common_delayCycles(64000); // ~2ms at 32MHz
    if (gauge_read(i2c, 0x3E, rx_frame, 36) != 36)
        return false;
    *pOutValue = (uint16_t)rx_frame[2] | ((uint16_t)rx_frame[3] << 8);

    return true;
}


#define TEMP_CONFIG_ADDR  0x46B1 



bool BQ27Z746_GetTempConfig(I2C_Regs *i2c, uint8_t *pTempEnable)
{
    uint8_t tx_addr[2];
    uint8_t rx_frame[DF_FRAME_SIZE];

    if (pTempEnable == NULL)
        return false;
    tx_addr[0] = (uint8_t)(TEMP_CONFIG_ADDR & 0xFF);
    tx_addr[1] = (uint8_t)((TEMP_CONFIG_ADDR >> 8) & 0xFF);

    if (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, tx_addr, 2) != 2)
        return false;

    DL_Common_delayCycles(64000); /* ~2 ms at 32 MHz */
    
    if (gauge_read(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, rx_frame, DF_FRAME_SIZE) != DF_FRAME_SIZE)
        return false;
    *pTempEnable = rx_frame[2];

    return true;
}

/*
 * Integration patch — how to guard data-flash writes with the security layer
 *
 * Add #include "BQ27Z746_security.h" to BQ27Z746_functions.c, then
 * wrap each DF write function as shown below.
 *
 * The pattern is always:
 *   1. EnsureUnsealed  — check/open the device
 *   2. Do the DF work
 *   3. Seal            — lock it again (omit in dev/calibration flows)
 *
 * Only the two functions that write to data flash are shown; read-only
 * functions (BQ27Z746_GetFETOptions, BQ27Z746_GetTempConfig) do not
 * need unsealing because DF reads work in SEALED mode via MAC reads.
 * Direct physical-address DF reads via 0x3E DO require UNSEALED — so
 * those are guarded here too.
 */

/* ----------------------------------------------------------------
 * BQ27Z746_SetUTFET_Direct  (replaces existing version)
 * ---------------------------------------------------------------- */
bool BQ27Z746_SetUTFET_Direct(I2C_Regs *i2c, bool enable)
{
    /* Gate: must be unsealed to access DF at physical address */
    if (!BQ27Z746_EnsureUnsealed(i2c,
                                  BQ27Z746_UNSEAL_KEY1,
                                  BQ27Z746_UNSEAL_KEY2))
        return false;

    uint8_t tx_addr[2];
    uint8_t rx_frame[DF_FRAME_SIZE];

    tx_addr[0] = (uint8_t)(FET_OPTIONS_ADDR & 0xFF);
    tx_addr[1] = (uint8_t)((FET_OPTIONS_ADDR >> 8) & 0xFF);

    if (gauge_write(i2c, 0x3E, tx_addr, 2) != 2)
        goto fail;

    DL_Common_delayCycles(64000);

    if (gauge_read(i2c, 0x3E, rx_frame, DF_FRAME_SIZE) != DF_FRAME_SIZE)
        goto fail;

    uint16_t current_val = (uint16_t)rx_frame[2] | ((uint16_t)rx_frame[3] << 8);

    if (enable)
        current_val |=  (1u << 1);
    else
        current_val &= ~(1u << 1);

    uint8_t write_buffer[4];
    write_buffer[0] = tx_addr[0];
    write_buffer[1] = tx_addr[1];
    write_buffer[2] = (uint8_t)(current_val & 0xFF);
    write_buffer[3] = (uint8_t)(current_val >> 8);

    if (gauge_write(i2c, 0x3E, write_buffer, 4) != 4)
        goto fail;

    DL_Common_delayCycles(64000);

    /* Verify */
    if (gauge_write(i2c, 0x3E, tx_addr, 2) != 2)
        goto fail;

    DL_Common_delayCycles(64000);

    uint8_t verify_frame[DF_FRAME_SIZE];
    if (gauge_read(i2c, 0x3E, verify_frame, DF_FRAME_SIZE) != DF_FRAME_SIZE)
        goto fail;

    uint16_t verify_val = (uint16_t)verify_frame[2] | ((uint16_t)verify_frame[3] << 8);

    bool ok = (verify_val == current_val);

    /* Re-seal now that DF work is done */
    BQ27Z746_Seal(i2c);
    return ok;

fail:
    BQ27Z746_Seal(i2c);
    return false;
}

/* ----------------------------------------------------------------
 * BQ27Z746_SetTempSensorConfig  (replaces existing version)
 * ---------------------------------------------------------------- */
bool BQ27Z746_SetTempSensorConfig(I2C_Regs *i2c, bool enable_internal, bool enable_ts1)
{
    if (!BQ27Z746_EnsureUnsealed(i2c,
                                  BQ27Z746_UNSEAL_KEY1,
                                  BQ27Z746_UNSEAL_KEY2))
        return false;

    uint8_t tx_addr[2];
    uint8_t rx_frame[DF_FRAME_SIZE];

    tx_addr[0] = (uint8_t)(TEMP_CONFIG_ADDR & 0xFF);
    tx_addr[1] = (uint8_t)((TEMP_CONFIG_ADDR >> 8) & 0xFF);

    if (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, tx_addr, 2) != 2)
        goto fail;

    DL_Common_delayCycles(64000);

    if (gauge_read(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, rx_frame, DF_FRAME_SIZE) != DF_FRAME_SIZE)
        goto fail;

    uint8_t temp_enable = rx_frame[2];

    if (enable_internal)
        temp_enable |=  (1u << 0);
    else
        temp_enable &= ~(1u << 0);

    if (enable_ts1)
        temp_enable |=  (1u << 1);
    else
        temp_enable &= ~(1u << 1);

    uint8_t write_buf[3];
    write_buf[0] = tx_addr[0];
    write_buf[1] = tx_addr[1];
    write_buf[2] = temp_enable;

    if (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, write_buf, 3) != 3)
        goto fail;

    DL_Common_delayCycles(64000);

    /* Verify */
    if (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, tx_addr, 2) != 2)
        goto fail;

    DL_Common_delayCycles(64000);

    uint8_t verify_frame[DF_FRAME_SIZE];
    if (gauge_read(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, verify_frame, DF_FRAME_SIZE) != DF_FRAME_SIZE)
        goto fail;

    bool ok = (verify_frame[2] == temp_enable);

    BQ27Z746_Seal(i2c);
    return ok;

fail:
    BQ27Z746_Seal(i2c);
    return false;
}


bool BQ27Z746_UseInternalTempOnly(I2C_Regs *i2c)
{
    return BQ27Z746_SetTempSensorConfig(i2c, true, false);
}


/*
 * BQ27Z746 Security / Unseal Layer
 *
 * References:
 *   TRM §10.3   — Security Modes (SEALED / UNSEALED / FULL ACCESS)
 *   TRM §15.2.35 — OperationStatus (MAC 0x0054), SEC1:SEC0 at bits [9:8]
 *   TRM §15.2.27 — SecurityKeys    (MAC 0x0035)
 *   TRM §10.3.2  — SEALED to UNSEALED two-step key sequence
 */

/* ----------------------------------------------------------------
 * Internal: OperationStatusA bit positions
 * The 32-bit OperationStatus word has SEC1:SEC0 at bits [9:8],
 * which sit in byte 1 (the second byte) of the little-endian word.
 * ---------------------------------------------------------------- */
#define OPSTATUS_SEC_SHIFT   8u
#define OPSTATUS_SEC_MASK    0x03u

/* MAC command codes used here */
#define MAC_SEAL_DEVICE      0x0030u

/* How long to wait after sending a key word before the gauge
 * processes it (not formally specified; 2 ms is conservative). */
#define KEY_DELAY_US         2000u

/* usleep helper — matches the one in gauge.c */
static void security_usleep(uint32_t us)
{
    delay_cycles(us * 32u);   /* 32 MHz system clock */
}

/* ================================================================
 * BQ27Z746_GetSecurityMode
 * ================================================================ */
uint8_t BQ27Z746_GetSecurityMode(I2C_Regs *i2c)
{
    uint8_t data[BQ27Z746_MAC_DATA_LEN];
    uint8_t len = 0u;

    if (!BQ27Z746_MAC_Read(i2c, BQ27Z746_MAC_OPERATIONSTATUS, data, &len))
        return 0xFFu;   /* I2C error sentinel */

    /* OperationStatus is a 32-bit LE value; SEC1:SEC0 are bits [9:8].
     * Byte 0 = bits[7:0], Byte 1 = bits[15:8].
     * So SEC1:SEC0 = (byte1 >> 0) & 0x03. */
    if (len < 2u)
        return 0xFFu;

    return (uint8_t)((data[1] >> (OPSTATUS_SEC_SHIFT - 8u)) & OPSTATUS_SEC_MASK);
}

/* ================================================================
 * BQ27Z746_IsSealed
 * ================================================================ */
bool BQ27Z746_IsSealed(I2C_Regs *i2c)
{
    return (BQ27Z746_GetSecurityMode(i2c) == BQ27Z746_SEC_SEALED);
}

/* ================================================================
 * BQ27Z746_Unseal
 *
 * Protocol (TRM §10.3.2):
 *   Step 1 — Write KEY_WORD1 as a 2-byte little-endian payload to
 *             AltManufacturerAccess (0x3E). No other write in between.
 *   Step 2 — Write KEY_WORD2 the same way.
 *   The gauge transitions to UNSEALED if both words match.
 *
 * We use gauge_write directly (not BQ27Z746_MAC_Write) because this
 * is a raw two-byte word write to 0x3E — not a framed MAC command
 * with a checksum/length trailer.
 * ================================================================ */
bool BQ27Z746_Unseal(I2C_Regs *i2c, uint16_t key1, uint16_t key2)
{
    uint8_t mode = BQ27Z746_GetSecurityMode(i2c);

    /* Already open — nothing to do */
    if (mode == BQ27Z746_SEC_UNSEALED || mode == BQ27Z746_SEC_FULL_ACCESS)
        return true;

    /* Can't unseal if we can't read status */
    if (mode == 0xFFu)
        return false;

    /* --- Step 1: send first key word --- */
    uint8_t word[2];
    word[0] = (uint8_t)(key1 & 0xFFu);
    word[1] = (uint8_t)((key1 >> 8u) & 0xFFu);

    if (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, word, 2u) != 2)
        return false;

    security_usleep(KEY_DELAY_US);

    /* --- Step 2: send second key word --- */
    word[0] = (uint8_t)(key2 & 0xFFu);
    word[1] = (uint8_t)((key2 >> 8u) & 0xFFu);

    if (gauge_write(i2c, BQ27Z746_REG_ALTMANUFACTURERACCESS, word, 2u) != 2)
        return false;

    security_usleep(KEY_DELAY_US);

    /* --- Verify the transition occurred --- */
    mode = BQ27Z746_GetSecurityMode(i2c);
    return (mode == BQ27Z746_SEC_UNSEALED || mode == BQ27Z746_SEC_FULL_ACCESS);
}

/* ================================================================
 * BQ27Z746_Seal
 *
 * Sends MAC 0x0030 SealDevice.
 * TRM §10.3.1: after sealing, only a hardware reset returns to
 * SEALED on next power-up; the device immediately becomes SEALED.
 * ================================================================ */
bool BQ27Z746_Seal(I2C_Regs *i2c)
{
    uint8_t mode = BQ27Z746_GetSecurityMode(i2c);

    /* Already sealed */
    if (mode == BQ27Z746_SEC_SEALED)
        return true;

    if (!BQ27Z746_MAC_Send(i2c, MAC_SEAL_DEVICE))
        return false;

    security_usleep(KEY_DELAY_US);

    return (BQ27Z746_GetSecurityMode(i2c) == BQ27Z746_SEC_SEALED);
}

/* ================================================================
 * BQ27Z746_EnsureUnsealed
 * ================================================================ */
bool BQ27Z746_EnsureUnsealed(I2C_Regs *i2c, uint16_t key1, uint16_t key2)
{
    uint8_t mode = BQ27Z746_GetSecurityMode(i2c);

    if (mode == BQ27Z746_SEC_UNSEALED || mode == BQ27Z746_SEC_FULL_ACCESS)
        return true;   /* already open */

    if (mode != BQ27Z746_SEC_SEALED)
        return false;  /* I2C error or reserved state */

    return BQ27Z746_Unseal(i2c, key1, key2);
}