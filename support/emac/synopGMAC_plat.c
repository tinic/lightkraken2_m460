/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-24     Wayne        first version
 */

#include "synopGMAC_plat.h"

void plat_delay(u32 delay)
{
    volatile u32 loop = delay;
    while (loop--);
}

u32 synopGMACReadReg(u32 RegBase, u32 RegOffset)
{
    u32 addr = RegBase + RegOffset;
    u32 data = *((volatile u32 *)addr);

    return data;
}

void synopGMACWriteReg(u32 RegBase, u32 RegOffset, u32 RegData)
{
    u32 addr = RegBase + (u32)RegOffset;

    if (RegOffset == 0) // For gmacconfig, we need add a little delay time here.
        plat_delay(DEFAULT_LOOP_VARIABLE);

    *((volatile u32 *)addr) = RegData;

    return;
}

void synopGMACSetBits(u32 RegBase, u32 RegOffset, u32 BitPos)
{
    u32 data = synopGMACReadReg(RegBase, RegOffset) | BitPos;

    synopGMACWriteReg(RegBase, RegOffset, data);

    return;
}

void synopGMACClearBits(u32 RegBase, u32 RegOffset, u32 BitPos)
{
    u32 data = synopGMACReadReg(RegBase, RegOffset) & (~BitPos);

    synopGMACWriteReg(RegBase, RegOffset, data);

    return;
}

bool synopGMACCheckBits(u32 RegBase, u32 RegOffset, u32 BitPos)
{
    u32 data = synopGMACReadReg(RegBase, RegOffset) & BitPos;

    if (data)
        return true;
    else
        return false;
}
