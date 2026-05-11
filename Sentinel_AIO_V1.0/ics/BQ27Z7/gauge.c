#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "gauge.h"

// ================================================================
// Low-level I2C
// ================================================================
static uint8_t g_gaugeAddr = GAUGE_I2C_ADDR;

void gauge_address(I2C_Regs *i2c, uint8_t nAddress)
{
    if (nAddress != 0) g_gaugeAddr = nAddress;
    (void)i2c;
}

int gauge_write(I2C_Regs *i2c, uint8_t nRegister, uint8_t *pData, uint8_t nLength)
{
    if (nLength == 0 || pData == NULL || i2c == NULL) return 0;
    I2C_Status status = I2C_WriteDevice(i2c, g_gaugeAddr, nRegister, pData, nLength);
    return (status == I2C_SUCCESS) ? (int)nLength : 0;
}

int gauge_read(I2C_Regs *i2c, uint8_t nRegister, uint8_t *pData, uint8_t nLength)
{
    if (nLength == 0 || pData == NULL || i2c == NULL) return 0;
    I2C_Status status = I2C_ReadDevice(i2c, g_gaugeAddr, nRegister, pData, nLength);
    return (status == I2C_SUCCESS) ? (int)nLength : 0;
}

// ================================================================
// usleep helper
// ================================================================
static void usleep(uint32_t us)
{
    // At 32 MHz: 32 cycles per us
    delay_cycles(us * 32);
}

// ================================================================
// High-level functions (SLUA801)
// ================================================================
unsigned int gauge_control(I2C_Regs *i2c, unsigned int nSubCmd)
{
    uint8_t pData[2];
    pData[0] = nSubCmd & 0xFF;
    pData[1] = (nSubCmd >> 8) & 0xFF;
    gauge_write(i2c, 0x00, pData, 2);
    gauge_read(i2c, 0x00, pData, 2);
    return ((unsigned int)pData[1] << 8) | pData[0];
}

unsigned int gauge_cmd_read(I2C_Regs *i2c, uint8_t nCmd)
{
    uint8_t pData[2];
    gauge_read(i2c, nCmd, pData, 2);
    return ((unsigned int)pData[1] << 8) | pData[0];
}

unsigned int gauge_cmd_write(I2C_Regs *i2c, uint8_t nCmd, unsigned int nData)
{
    uint8_t pData[2];
    pData[0] = nData & 0xFF;
    pData[1] = (nData >> 8) & 0xFF;
    return gauge_write(i2c, nCmd, pData, 2);
}

#define MAX_ATTEMPTS 5
bool gauge_cfg_update(I2C_Regs *i2c)
{
    unsigned int nFlags; int nAttempts = 0;
    gauge_control(i2c, SET_CFGUPDATE);
    do {
        nFlags = gauge_cmd_read(i2c, CMD_FLAGS);
        if (!(nFlags & CFGUPD)) usleep(500000);
    } while (!(nFlags & CFGUPD) && (nAttempts++ < MAX_ATTEMPTS));
    return (nAttempts < MAX_ATTEMPTS);
}

bool gauge_exit(I2C_Regs *i2c, unsigned int nCmd)
{
    unsigned int nFlags; int nAttempts = 0;
    gauge_control(i2c, nCmd);
    do {
        nFlags = gauge_cmd_read(i2c, CMD_FLAGS);
        if (nFlags & CFGUPD) usleep(500000);
    } while ((nFlags & CFGUPD) && (nAttempts++ < MAX_ATTEMPTS));
    return (nAttempts < MAX_ATTEMPTS);
}

// ----------------------------------------------------------------
int gauge_read_data_class(I2C_Regs *i2c, uint8_t nDataClass, uint8_t *pData, uint8_t nLength)
{
    unsigned char nRemainder = nLength;
    unsigned char nDataBlock = 0x00;
    unsigned int  nData;

    if (nLength < 1) return 0;

    do {
        unsigned char blockLen = (nRemainder > 32) ? 32 : nRemainder;
        nRemainder -= blockLen;

        nData = ((unsigned int)nDataBlock << 8) | nDataClass;
        gauge_cmd_write(i2c, CMD_DATA_CLASS, nData);

        if (gauge_read(i2c, CMD_BLOCK_DATA, pData, blockLen) != blockLen)
            return -1;

        pData += blockLen;
        nDataBlock++;
    } while (nRemainder > 0);

    return 0;
}

static unsigned char check_sum(uint8_t *pData, uint8_t nLength)
{
    unsigned char nSum = 0x00;
    for (uint8_t n = 0; n < nLength; n++) nSum += pData[n];
    return 0xFF - nSum;
}

int gauge_write_data_class(I2C_Regs *i2c, uint8_t nDataClass, uint8_t *pData, uint8_t nLength)
{
    unsigned char nRemainder = nLength;
    unsigned char nDataBlock = 0x00;
    uint8_t pCheckSum[2];
    unsigned int nData;

    if (nLength < 1) return 0;

    do {
        unsigned char blockLen = (nRemainder > 32) ? 32 : nRemainder;
        nRemainder -= blockLen;

        nData = ((unsigned int)nDataBlock << 8) | nDataClass;
        gauge_cmd_write(i2c, CMD_DATA_CLASS, nData);

        if (gauge_write(i2c, CMD_BLOCK_DATA, pData, blockLen) != blockLen)
            return -1;

        pCheckSum[0] = check_sum(pData, blockLen);
        gauge_write(i2c, CMD_CHECK_SUM, pCheckSum, 1);
        usleep(10000);

        gauge_cmd_write(i2c, CMD_DATA_CLASS, nData);
        gauge_read(i2c, CMD_CHECK_SUM, pCheckSum + 1, 1);

        if (pCheckSum[0] != pCheckSum[1]) return -2;

        pData += blockLen;
        nDataBlock++;
    } while (nRemainder > 0);

    return 0;
}

// ----------------------------------------------------------------
// Full FlashStream parser (Golden Image support) - now warning-free
// ----------------------------------------------------------------
char *gauge_execute_fs(I2C_Regs *i2c, char *pFS)
{
    int nLength = strlen(pFS);
    int nDataLength;
    char pBuf[16];
    uint8_t pData[32];
    int n, m;
    char *pEnd = NULL;
    char *pErr;
    bool bWriteCmd = false;
    unsigned char nRegister;

    m = 0;
    for (n = 0; n < nLength; n++)
        if (pFS[n] != ' ') pFS[m++] = pFS[n];

    pEnd = pFS + m;
    pEnd[0] = 0;

    do {
        switch (*pFS) {
            case ';': break;

            case 'W':
            case 'C':
                bWriteCmd = (*pFS == 'W');
                pFS++;
                if (*pFS != ':') return pFS;
                pFS++;
                n = 0;

                while ((pEnd - pFS > 2) && (n < sizeof(pData) + 2) && (*pFS != '\n')) {
                    pBuf[0] = *(pFS++);
                    pBuf[1] = *(pFS++);
                    pBuf[2] = 0;
                    m = strtoul(pBuf, &pErr, 16);
                    if (*pErr) return (pFS - 2);

                    if (n == 0) gauge_address(i2c, (uint8_t)m);
                    if (n == 1) nRegister = (uint8_t)m;
                    if (n > 1) pData[n - 2] = (uint8_t)m;
                    n++;
                }
                if (n < 3) return pFS;

                nDataLength = n - 2;

                if (bWriteCmd) {
                    gauge_write(i2c, nRegister, pData, (uint8_t)nDataLength);
                } else {
                    uint8_t pDataFromGauge[32];   // safe max size
                    gauge_read(i2c, nRegister, pDataFromGauge, (uint8_t)nDataLength);
                    if (memcmp(pData, pDataFromGauge, nDataLength)) return pFS;
                }
                break;

            case 'X':
                pFS++;
                if (*pFS != ':') return pFS;
                pFS++;
                n = 0;
                while ((pFS != pEnd) && (*pFS != '\n') && (n < sizeof(pBuf) - 1)) {
                    pBuf[n++] = *pFS;
                    pFS++;
                }
                pBuf[n] = 0;
                n = atoi(pBuf);
                usleep((uint32_t)n * 1000);
                break;

            default:
                return pFS;
        }

        while ((pFS != pEnd) && (*pFS != '\n')) pFS++;
        if (pFS != pEnd) pFS++;

    } while (pFS != pEnd);

    return pFS;
}