
#ifndef _RTD266X_MAIN_H_
#define _RTD266X_MAIN_H_

#include "fatfs.h"
#include "crc.h"

typedef enum _ECommondCommandType {
    E_CC_NOOP             = 0,
    E_CC_WRITE            = 1,
    E_CC_READ             = 2,
    E_CC_WRITE_AFTER_WREN = 3,
    E_CC_WRITE_AFTER_EWSR = 4,
    E_CC_ERASE            = 5
} ECommondCommandType;

typedef struct _FlashDesc {
    const char* device_name;
    uint32_t    jedec_id;
    uint32_t    size_kb;
    uint32_t    page_size;
    uint32_t    block_size_kb;
} FlashDesc;

uint32_t SPICommonCommand(ECommondCommandType cmd_type, uint8_t cmd_code, uint8_t num_reads, uint8_t num_writes, uint32_t write_value);

void rtd266x_init(void);
int WriteReg(uint8_t a, uint8_t d);
uint32_t ReadReg(uint8_t a);
int ReadBytesFromAddr(uint8_t reg, uint8_t* dest, uint8_t len);
const FlashDesc* FindChip(uint32_t jedec_id);
uint8_t GetManufacturerId(uint32_t jedec_id);
void SetupChipCommands(uint32_t jedec_id);
int EraseFlash(void);
int ProgramFlash(FIL *f, uint32_t chip_size);
int WriteBytesToAddr(uint8_t reg, uint8_t* values, uint8_t len);
int VerifyFlash(FIL *f, uint32_t chip_size);
int ShouldProgramPage(uint8_t* buffer, uint32_t size);
int SaveFlash(FIL *f, uint32_t chip_size);

#endif
