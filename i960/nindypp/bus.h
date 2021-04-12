/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/******************************************************************/
/* 		Copyright (c) 1989, Intel Corporation

   Intel hereby grants you permission to copy, modify, and 
   distribute this software and its documentation.  Intel grants
   this permission provided that the above copyright notice 
   appears in all copies and that both the copyright notice and
   this permission notice appear in supporting documentation.  In
   addition, Intel grants this permission provided that you
   prominently mark as not part of the original any modifications
   made to this software or documentation, and that the name of 
   Intel Corporation not be used in advertising or publicity 
   pertaining to distribution of the software or the documentation 
   without specific, written prior permission.  

   Intel Corporation does not warrant, guarantee or make any 
   representations regarding the use of, or the results of the use
   of, the software and documentation in terms of correctness, 
   accuracy, reliability, currentness, or otherwise; and you rely
   on the software, documentation and results solely at your own 
   risk.							  */
/******************************************************************/
/***********************************************************************
****                                                                ****
****    bus.h   header file for 80960CA bus controller              ****
****                                                                ****
***********************************************************************/
#ifndef NINDY_BUS_H__
#define NINDY_BUS_H__

#define BURST_ENABLE    0x1
#define READY_ENABLE    0x2
#define PIPELINE_ENABLE 0x4

#define BUS_WIDTH_8     0x0
#define BUS_WIDTH_16    (0x1 << 19)
#define BUS_WIDTH_32    (0x2 << 19)

#define BIG_ENDIAN      (0x1 << 22)

#define NRAD(WS)    (WS << 3)   /* WS can be 0-31   */
#define NRDD(WS)    (WS << 8)   /* WS can be 0-3    */
#define NXDA(WS)    (WS << 10)  /* WS can be 0-3    */
#define NWAD(WS)    (WS << 12)  /* WS can be 0-31   */
#define NWDD(WS)    (WS << 17)  /* WS can be 0-3    */

/* 
Perform a bit-wise OR of the desired parameters to specify a region.

 
    bus_region_1_config =
        BURST_ENABLE | BUS_WIDTH_32 | NRAD(3) | NRDD(1) |
        NXDA(1) | NWAD(2) | NWDD(2);

*/
#endif // end NINDY_BUS_H__