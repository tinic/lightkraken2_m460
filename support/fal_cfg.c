
#include "fal.h"
#include "fal_cfg.h"

#include "NuMicro.h"

#include <stdint.h>
#include <memory.h>

static int init(void) {
    FMC_Open();
    return 1;
}

static int read(long offset, uint8_t *buf, size_t size) {
    uint32_t addr = nor_flash0.addr + offset;
    for (size_t i = 0; i < size; i++, addr++, buf++) {
        *buf = *(uint8_t *)addr;
    }
    return size;
}


static int write(long offset, const uint8_t *buf, size_t size) {
    uint32_t addr = nor_flash0.addr + offset;

    SYS_UnlockReg();

    FMC_ENABLE_AP_UPDATE();

    uint32_t write_data[2];
    for (size_t i = 0; i < size; i += 8, buf+=8, addr += 8) {
        memcpy(&write_data, buf, 8);
        FMC_Write8Bytes(addr, write_data[0], write_data[1]);
    }

    FMC_DISABLE_AP_UPDATE();

    SYS_LockReg();
    return size;
}


static int erase(long offset, size_t size) {
    if (offset % FLASH_DB_BLOCK_SIZE != 0) {
        return 0;
    }

    uint32_t addr = nor_flash0.addr + offset;

    size_t erase_pages = size / FLASH_DB_BLOCK_SIZE;
    if (size % FLASH_DB_BLOCK_SIZE != 0) {
        erase_pages++;
    }

    SYS_UnlockReg();

    FMC_ENABLE_AP_UPDATE();

    for (size_t i = 0; i < erase_pages; i++) {
        FMC_Erase(addr);
        addr += FLASH_DB_BLOCK_SIZE;
    }

    FMC_DISABLE_AP_UPDATE();

    SYS_LockReg();

    return size;
}

const struct fal_flash_dev nor_flash0 =
{
    .name       = "norflash0",
    .addr       = FLASH_DB_START_ADDRESS,
    .len        = FLASH_DB_LENGTH,
    .blk_size   = FLASH_DB_BLOCK_SIZE,
    .ops        = {init, read, write, erase},
    .write_gran = FLASH_DB_WRITE_GRAN
};
