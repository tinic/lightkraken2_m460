
/*
MIT License

Copyright (c) 2023 Tinic Uro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
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
