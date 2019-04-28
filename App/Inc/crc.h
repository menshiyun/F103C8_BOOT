
#ifndef _CRC_H_
#define _CRC_H_

void InitCRC(void);
void ProcessCRC(const uint8_t *data, int len);
uint8_t GetCRC(void);

#endif
