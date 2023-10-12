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
#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

//#define FAL_DEBUG 1
#define FAL_PART_HAS_TABLE_CFG
#define NOR_FLASH_DEV_NAME "norflash0"

#define FLASH_DB_LENGTH (128*1024)
#define FLASH_DB_START_ADDRESS (0x100000-FLASH_DB_LENGTH)
#define FLASH_DB_BLOCK_SIZE 4096
#define FLASH_DB_WRITE_GRAN 64

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev nor_flash0;

/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &nor_flash0,                                                     \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#define FAL_PART_TABLE                                                              \
{                                                                                   \
    {FAL_PART_MAGIC_WORD, "fdb_kvdb1",  NOR_FLASH_DEV_NAME, 0, FLASH_DB_LENGTH, 0}, \
}
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
