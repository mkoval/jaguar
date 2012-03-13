//*****************************************************************************
//
// limit.h - Supports the operation of the limit switches.
//
// Copyright (c) 2008-2011 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 8264 of the RDK-BDC24 Firmware Package.
//
//*****************************************************************************

#ifndef __LIMIT_H__
#define __LIMIT_H__

//*****************************************************************************
//
// The bit positions of the flags in g_ulLimitFlags.
//
//*****************************************************************************
#define LIMIT_FLAG_FORWARD_OK   0
#define LIMIT_FLAG_REVERSE_OK   1

//*****************************************************************************
//
// This function determines if the state of the forward limit switch allows the
// motor to operate in the forward direction.
//
//*****************************************************************************
#define LimitForwardOK()                                                      \
        (HWREGBITW(&g_ulLimitFlags, LIMIT_FLAG_FORWARD_OK) == 1)

//*****************************************************************************
//
// This function determines if the state of the reverse limit switch allows the
// motor to operate in the reverse direction.
//
//*****************************************************************************
#define LimitReverseOK()                                                      \
        (HWREGBITW(&g_ulLimitFlags, LIMIT_FLAG_REVERSE_OK) == 1)

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern unsigned long g_ulLimitFlags;
extern void LimitInit(void);
extern void LimitPositionEnable(void);
extern void LimitPositionDisable(void);
extern unsigned long LimitPositionActive(void);
extern void LimitPositionForwardSet(long lPosition, unsigned long ulLessThan);
extern void LimitPositionForwardGet(long *plPosition,
                                    unsigned long *pulLessThan);
extern void LimitPositionReverseSet(long lPosition, unsigned long ulLessThan);
extern void LimitPositionReverseGet(long *plPosition,
                                    unsigned long *pulLessThan);
extern void LimitTick(void);

#endif // __LIMIT_H__
