
#include "stm32f1xx_hal.h"
#include "crc.h"

static unsigned gCrc = 0;

void InitCRC(void)
{
    gCrc = 0;
}

void ProcessCRC(const uint8_t *data, int len)
{
    int i = 0, j = 0;
    for (j = len; j; j--, data++) {
        gCrc ^= (*data << 8);
        for(i = 8; i; i--) {
            if (gCrc & 0x8000)
                gCrc ^= (0x1070 << 3);
            gCrc <<= 1;
        }
    }
}

uint8_t GetCRC(void)
{
    return (uint8_t)(gCrc >> 8);
}
