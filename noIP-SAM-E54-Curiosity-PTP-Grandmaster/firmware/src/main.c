//DOM-IGNORE-BEGIN
/*
Copyright (C) 2025, Microchip Technology Inc., and its subsidiaries. All rights reserved.

The software and documentation is provided by microchip and its contributors
"as is" and any express, implied or statutory warranties, including, but not
limited to, the implied warranties of merchantability, fitness for a particular
purpose and non-infringement of third party intellectual property rights are
disclaimed to the fullest extent permitted by law. In no event shall microchip
or its contributors be liable for any direct, indirect, incidental, special,
exemplary, or consequential damages (including, but not limited to, procurement
of substitute goods or services; loss of use, data, or profits; or business
interruption) however caused and on any theory of liability, whether in contract,
strict liability, or tort (including negligence or otherwise) arising in any way
out of the use of the software and documentation, even if advised of the
possibility of such damage.

Except as expressly permitted hereunder and subject to the applicable license terms
for any third-party software incorporated in the software and any applicable open
source software license terms, no license or other rights, whether express or
implied, are granted under any patent or other intellectual property rights of
Microchip or any third party.
*/
//DOM-IGNORE-END
/*******************************************************************************
  main-loop for Grand Master Time synchronization using 10BASE T1S MAC-PHY

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    The main file

  Description:
    This files contains the main-loop driving the initialization and the cyclic
    tasks
*******************************************************************************/

/**************************************************************************************************/
/******                    SET YOUR SERIAL TERMINAL BAUDRATE TO 115200                    *********/
/**************************************************************************************************/

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <stdio.h>                      // printf
#include <string.h>                     // memset, memcpy
#include "definitions.h"                // SYS function prototypes
#include "tc6.h"
#include "tc6-noip.h"
#include "tc6-regs.h"
#include "ptp.h"

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*                          USER ADJUSTABLE                             */
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

#define FIRMWARE_VERSION            TC6_LIB_VER_STRING

#ifndef BOARD_INSTANCE
#define BOARD_INSTANCE              (0)
#endif
#define BOARD_INSTANCES_MAX         (4)
#define T1S_PLCA_ENABLE             (true)
#define T1S_PLCA_NODE_ID            (BOARD_INSTANCE)
#define T1S_PLCA_NODE_COUNT         (8)
#define T1S_PLCA_BURST_COUNT        (0)
#define T1S_PLCA_BURST_TIMER        (0x80)
#define MAC_PROMISCUOUS_MODE        (false)
#define MAC_TX_CUT_THROUGH          (true)
#define MAC_RX_CUT_THROUGH          (true)
#define DELAY_BEACON_CHECK          (2000)
#define DELAY_STAT_PRINT            (1000)
#define DELAY_LED                   (333)

#define UDP_PAYLOAD_OFFSET          (42)

#define ESC_CLEAR_TERMINAL          "\033[2J"
#define ESC_CURSOR_X1Y1             "\033[1;1H"
#define ESC_CURSOR_STAT             "\033[2;1H"
#define ESC_HIDE_CURSOR             "\033[?25l"
#define ESC_CLEAR_LINE              "\033[2K"
#define ESC_RESETCOLOR              "\033[0m"
#define ESC_GREEN                   "\033[0;32m"
#define ESC_RED                     "\033[0;31m"
#define ESC_YELLOW                  "\033[1;33m"
#define ESC_BLUE                    "\033[0;36m"

#define MAX_PRINT_LINES             (30)

#define PRINT(...)                  printf(__VA_ARGS__)

#ifdef DEBUG
#define ASSERT(x)                  __conditional_software_breakpoint(x)
#else
#define ASSERT(x)
#endif

#define DEBUG_PRINT 0
#if DEBUG_PRINT==1
#define DBG_PRINT(...)              PRINT(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*                      DEFINES AND LOCAL VARIABLES                     */
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

typedef struct
{
    uint32_t packetCntCurrent;
    uint32_t packetCntTotal;
    uint32_t byteCntCurrent;
    uint32_t byteCntTotal;
    uint32_t previousVal;
    uint32_t errors;
} MainStats_t;

typedef struct
{
    MainStats_t stats[BOARD_INSTANCES_MAX];
    uint32_t nextStat;
    uint32_t nextBeaconCheck;
    uint32_t nextLed;
    uint32_t iperfTx;
    int8_t idxNoIp;
    bool button1;
    bool button2;
    bool gotBeaconState;
    bool lastBeaconState;
    volatile bool txBusy;
    bool allowTxStress;
} MainLocal_t;

static MainLocal_t m;
extern SYSTICK_OBJECT systick; /* Instanced in plib_systick.c */

/* Frame (1512 bytes) */
static uint8_t iperf[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x80, /* ..)CI..P */
    0xC2, 0x00, 0x01, T1S_PLCA_NODE_ID, 0x08, 0x00, 0x45, 0x00, /* V.....E. */
    0x05, 0xda, 0xf5, 0x36, 0x00, 0x00, 0x80, 0x11, /* ...6.... */
    0xc4, 0x09, 0xc0, 0xa8, 0x7d, 0x01, 0xc0, 0xa8, /* ....}... */
    0x7d, 0xFF, 0xc6, 0x38, 0x13, 0x89, 0x05, 0xc6, /* }..8.... */
    0xc9, 0x7b, 0x00, 0x00, 0x00, 0x08, 0x60, 0xf8, /* .{....`. */
    0x7f, 0x17, 0x00, 0x08, 0xed, 0xe5, 0x00, 0x00, /* ........ */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, /* ........ */
    0x13, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
    0x00, 0x00, 0xff, 0xff, 0xfc, 0x18, 0x00, 0x00, /* ........ */
    0x27, 0x10, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* '.012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, /* 23456789 */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 01234567 */
    0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, /* 89012345 */
    0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, /* 67890123 */
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, /* 45678901 */
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39  /* 23456789 */
};

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*                      PRIVATE FUNCTION PROTOTYPES                     */
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

static char *MoveCursor(bool newLine);
static void PrintMenu(void);
static void CheckUartInput(void);
static void SendIperfPacket(void);
static void CheckButton(uint8_t instance, bool newLevel, bool *oldLevel);
static void OnSendIperf(void *pDummy, const uint8_t *pTx, uint16_t len, uint32_t idx, void *pDummy2);

static uint32_t invert_uint32(uint32_t in);
static uint16_t invert_uint16(uint16_t in);
static uint32_t init_PTP_master(void);
static void register_access_helper_function(bool success, uint32_t *state_var, uint32_t next_state);
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*                         PUBLIC FUNCTIONS                             */
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
uint8_t temp_buffer[256] = {0};
static uint32_t TC6_read_value = 0;

int main(void)
{
    //this is for demo purposes: It shows that even if the timestamp is delayed several ms, the PTP will still work
    uint32_t dly = 0;

    uint16_t seq_id = 0;
    uint32_t PTP_task_state = PTP_STATE_send_sync;
    uint32_t Timestamp_Register = 0;
    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
    const uint8_t temp_clk[8] = {0x40, 0x84, 0x32, 0xff, 0xfe, 0x7d, 0x07, 0xfa};
    uint32_t last_now = 0;
    uint32_t now = 0;
    uint32_t init_res = 0;

    /* Initialize all modules */
    SYS_Initialize(NULL);
    memset(&m, 0, sizeof(m));
    SYSTICK_TimerStart();
    PRINT(ESC_CLEAR_TERMINAL \
          ESC_CURSOR_X1Y1    \
          ESC_HIDE_CURSOR    \
          ESC_YELLOW         \
          "=== Time Sync 10BASE-T1S Demo - GM " FIRMWARE_VERSION " (" \
          __DATE__ " " __TIME__ ") ===%s" ESC_RESETCOLOR, MoveCursor(false));

    m.idxNoIp = TC6NoIP_Init(T1S_PLCA_ENABLE, T1S_PLCA_NODE_ID, T1S_PLCA_NODE_COUNT,
        T1S_PLCA_BURST_COUNT, T1S_PLCA_BURST_TIMER, MAC_PROMISCUOUS_MODE,
        MAC_TX_CUT_THROUGH, MAC_RX_CUT_THROUGH);

    if (m.idxNoIp < 0) {
        PRINT(ESC_RED "%sFailed to initialize TC6 noIP Driver" ESC_RESETCOLOR "\r\n", MoveCursor(true));
        goto ERROR;
    }

    m.nextStat = DELAY_STAT_PRINT;
    m.nextBeaconCheck = DELAY_BEACON_CHECK;
    m.allowTxStress = false;

    PrintMenu();
    while(true)
    {
        TC6NoIP_Service();
        if(true == TC6_GetInit_Done(m.idxNoIp)) break;
    }
    /*User code can be implemented after this point! */
    init_res = init_PTP_master();
    if (0 != init_res)
    {
        DBG_PRINT(ESC_RED "%sINIT FAILED!!!" ESC_RESETCOLOR "\r\n");
        DBG_PRINT("Init res: %i\r\n",init_res );
        goto ERROR;
    }
    while (true)
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();

        TC6NoIP_Service();
        now = systick.tickCounter;

        if((false == m.txBusy) && (true == m.allowTxStress) )
        {
            if((systick.tickCounter-last_now > SYN_MESSAGE_CLEAR_TIME_MS) && (systick.tickCounter-last_now < ((SYNC_MESSAGE_PERIOD_MS-SYN_MESSAGE_CLEAR_TIME_MS)+dly)))
            {
                SendIperfPacket();
                TC6NoIP_Service();
            }
        }

        switch(PTP_task_state)
        {
            case PTP_STATE_send_sync:
            {
                if(((now - last_now) > (SYNC_MESSAGE_PERIOD_MS+dly)) && (false == m.txBusy) )
                {
                    DBG_PRINT("SYSTICK: %i\r\n", now);
                    DBG_PRINT("sync:\r\n");
                    dly++;
                    if(dly>MAX_DELAY_MS) dly=0;
                    last_now = now;

                    syncMsg_t msg;
                    memset(&msg, 0, sizeof(syncMsg_t));

                    msg.header.tsmt = 0x10;
                    msg.header.version = 0x02;
                    msg.header.messageLength = invert_uint16((uint16_t) 0x2c);
                    msg.header.domainNumber = 0;
                    msg.header.flags[0] = 0x02;
                    msg.header.flags[1] = 0x08;
                    msg.header.correctionField = 0;

                    memcpy( &msg.header.sourcePortIdentity.clockIdentity, temp_clk, 8);
                    msg.header.sourcePortIdentity.portNumber = 1;
                    msg.header.sequenceID = invert_uint16(seq_id);
                    msg.header.controlField = 2;
                    msg.header.logMessageInterval = 0xfd;

                    memcpy(temp_buffer, buffer_header, BUFFER_HEADER_LEN);
                    memcpy( &temp_buffer[BUFFER_HEADER_LEN],&msg, sizeof(syncMsg_t));
                    m.txBusy = true;
                    if (TC6NoIP_SendEthernetPacket_TimestampA(m.idxNoIp, temp_buffer, sizeof(syncMsg_t)+BUFFER_HEADER_LEN, OnSendIperf))
                    {
                        PTP_task_state = PTP_STATE_get_tx_status;
                    }
                    else
                    {
                        m.txBusy = false;
                    }
                    TC6NoIP_Service();
                }
                break;
            } //case PTP_STATE_send_sync:
            
            case PTP_STATE_get_tx_status:
            {
                DBG_PRINT("gTX:\r\n");
                register_access_helper_function(TC6_get_TXStatus(m.idxNoIp), &PTP_task_state, PTP_STATE_get_oa_status0);
                break;
            } //case PTP_STATE_get_tx_status:
            
            case PTP_STATE_get_oa_status0:
            {
                DBG_PRINT("gSTS:\r\n");
                if(true == TC6_get_last_read_complete())
                {
                    TC6_read_value = TC6_get_last_read_value();
                    if(TC6_read_value & TXMCTL_TXPMDET) //Transmit Packet Match Detection (Sync Message))
                    {
                        register_access_helper_function(TC6_read_OA_STATUS0_reg(m.idxNoIp), &PTP_task_state, PTP_STATE_get_timestamp_reg_sec);
                    }
                    else
                    {
                        DBG_PRINT("ERRORSTS:\r\n");
                        PTP_task_state = PTP_STATE_send_sync;
                    }
                }
                break;
            } //case PTP_STATE_get_oa_status0:
            
            case PTP_STATE_get_timestamp_reg_sec:
            {
                DBG_PRINT("gsec:\r\n");
                if(true == TC6_get_last_read_complete())
                {
                    TC6_read_value = TC6_get_last_read_value();
                    Timestamp_Register = TC6_read_value;
                    if(Timestamp_Register & OA_STS0_TTSCAA)
                    {
                        register_access_helper_function(TC6_read_reg(m.idxNoIp, OA_TTSCAH), &PTP_task_state, PTP_STATE_get_timestamp_reg_nsec);
                    }
                    else if(Timestamp_Register & OA_STS0_TTSCAB)
                    {
                        register_access_helper_function(TC6_read_reg(m.idxNoIp, OA_TTSCBH), &PTP_task_state, PTP_STATE_get_timestamp_reg_nsec);
                    }
                    else if(Timestamp_Register & OA_STS0_TTSCAC)
                    {
                        register_access_helper_function(TC6_read_reg(m.idxNoIp, OA_TTSCCH), &PTP_task_state, PTP_STATE_get_timestamp_reg_nsec);
                    }
                    else
                    {
                        //ERROR!!!
                        DBG_PRINT("ERRORsec:\r\n");
                        PTP_task_state = PTP_STATE_send_sync;
                    }
                }
                break;
            } //case PTP_STATE_get_timestamp_reg_sec:
            
            case PTP_STATE_get_timestamp_reg_nsec:
            {
                DBG_PRINT("gnsec:\r\n");
                if(true == TC6_get_last_read_complete())
                {
                    TC6_read_value = TC6_get_last_read_value();
                    timestamp_sec = TC6_read_value;
                    if(Timestamp_Register & OA_STS0_TTSCAA)
                    {
                        register_access_helper_function(TC6_read_reg(m.idxNoIp, OA_TTSCAL), &PTP_task_state, PTP_STATE_clear_status_reg);
                    }
                    else if(Timestamp_Register & OA_STS0_TTSCAB)
                    {
                        register_access_helper_function(TC6_read_reg(m.idxNoIp, OA_TTSCBL), &PTP_task_state, PTP_STATE_clear_status_reg);
                    }
                    else if(Timestamp_Register & OA_STS0_TTSCAC)
                    {
                        register_access_helper_function(TC6_read_reg(m.idxNoIp, OA_TTSCCL), &PTP_task_state, PTP_STATE_clear_status_reg);
                    }
                    else
                    {
                        //ERROR!!!
                        DBG_PRINT("ERRORnsec:\r\n");
                        PTP_task_state = PTP_STATE_send_sync;
                    }
                }
                break;
            } //case PTP_STATE_get_timestamp_reg_nsec:
            
            case PTP_STATE_clear_status_reg:
            {
                DBG_PRINT("clrSTS:\r\n");
                if(true == TC6_get_last_read_complete())
                {
                    TC6_read_value = TC6_get_last_read_value();
                    timestamp_nsec = TC6_read_value;
                    register_access_helper_function(TC6_write_reg(m.idxNoIp, OA_STATUS0, invert_uint16(Timestamp_Register)), &PTP_task_state, PTP_STATE_send_followup);
                }
                break;
            } //case PTP_STATE_clear_status_reg:
            
            case PTP_STATE_send_followup:
            {
                DBG_PRINT("follow:\r\n");

                followUpMsg_t msg2;
                memset(&msg2, 0, sizeof(followUpMsg_t));

                msg2.preciseOriginTimestamp.secondsLsb = timestamp_sec;
                msg2.preciseOriginTimestamp.nanoseconds =  timestamp_nsec + STATIC_OFFSET;
                if(msg2.preciseOriginTimestamp.nanoseconds > MAX_MAC_TN_VAL)
                {
                    msg2.preciseOriginTimestamp.nanoseconds = msg2.preciseOriginTimestamp.nanoseconds - MAX_MAC_TN_VAL;
                    msg2.preciseOriginTimestamp.secondsLsb++;
                }
                msg2.preciseOriginTimestamp.secondsLsb = invert_uint32(msg2.preciseOriginTimestamp.secondsLsb);
                msg2.preciseOriginTimestamp.nanoseconds = invert_uint32(msg2.preciseOriginTimestamp.nanoseconds);

                msg2.header.tsmt = 0x18;
                msg2.header.version = 0x02;
                msg2.header.messageLength = invert_uint16((uint16_t)0x4c);
                msg2.header.domainNumber = 0;
                msg2.header.flags[0] = 0x00;
                msg2.header.flags[1] = 0x08;
                msg2.header.correctionField = 0;

                memcpy( &msg2.header.sourcePortIdentity.clockIdentity, temp_clk, 8);
                msg2.header.sourcePortIdentity.portNumber = 1;
                msg2.header.sequenceID = invert_uint16(seq_id);
                msg2.header.controlField = 2;
                msg2.header.logMessageInterval = 0xfd;

                msg2.tlv.tlvType = invert_uint16((uint16_t)0x03);
                msg2.tlv.lengthField = invert_uint16((uint16_t)28);
                msg2.tlv.organizationId[0] = 0x00;
                msg2.tlv.organizationId[1] = 0x80;
                msg2.tlv.organizationId[2] = 0xc2;
                msg2.tlv.organizationSubType[2] = 0x01;
                msg2.tlv.cumulativescaledRateOffset = 0;
                msg2.tlv.gmTimeBaseIndicator = 0;

                memcpy( temp_buffer, buffer_header, BUFFER_HEADER_LEN);
                memcpy( &temp_buffer[BUFFER_HEADER_LEN], &msg2, sizeof(followUpMsg_t));
                m.txBusy = true;
                if (TC6NoIP_SendEthernetPacket(m.idxNoIp, temp_buffer, sizeof(followUpMsg_t)+BUFFER_HEADER_LEN, OnSendIperf))
                {
                    DBG_PRINT("msgsent:\r\n");
                    PTP_task_state = PTP_STATE_send_sync;
                    seq_id++;
                }
                else
                {
                    m.txBusy = false;
                }
                TC6NoIP_Service();
                break;
            } //case PTP_STATE_send_followup:
            
            default:
                break;
        } //end switch(PTP_task_state)

        if (now > m.nextLed)
        {
            m.nextLed = now + DELAY_LED;
            GPIO_USER_LED_1_Toggle();
        }

        CheckUartInput();
        CheckButton(0, GPIO_USER_BUTTON_1_Get(), &m.button1);
        CheckButton(1, GPIO_USER_BUTTON_2_Get(), &m.button2);
    }
ERROR:
    while (true)
    {
        uint32_t now = systick.tickCounter;
        if (now > m.nextLed)
        {
            m.nextLed = now + 100;
            GPIO_USER_LED_1_Toggle();
        }
        SYS_Tasks();
    }
    return (EXIT_FAILURE);
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*                  PRIVATE  FUNCTION IMPLEMENTATIONS                   */
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
static void register_access_helper_function(bool success, uint32_t *state_var, uint32_t next_state)
{
    static uint32_t num_reg_access_failures = 0;
    if(true == success )
    {
        num_reg_access_failures = 0;
        *state_var = next_state;
    }
    else
    {
        num_reg_access_failures++;
        if(num_reg_access_failures > MAX_NUM_REG_RETRIES)
        {
            num_reg_access_failures = 0;
            *state_var = PTP_STATE_send_sync;
            DBG_PRINT("RegAccessFail!\r\n");
        }
    }
}


static uint32_t invert_uint32(const uint32_t in_var)
{
    uint32_t out_var = 0;
    out_var =   (uint32_t)( ((in_var & 0x000000ff) >>  0) << 24) |
                (uint32_t)( ((in_var & 0x0000ff00) >>  8) << 16) |
                (uint32_t)( ((in_var & 0x00ff0000) >> 16) <<  8) |
                (uint32_t)( ((in_var & 0xff000000) >> 24) <<  0) ;
    return out_var;
}

static uint16_t invert_uint16(const uint16_t in_var)
{
    uint16_t out_var = 0;
    out_var =   (uint16_t)( ((in_var & 0x00ff) >> 0) << 8) |
                (uint16_t)( ((in_var & 0xff00) >> 8) << 0);
    return out_var;
}

static uint32_t init_PTP_master(void)
{
    uint32_t res = TC6_ptp_master_init(m.idxNoIp);
    return res;
}
static char *MoveCursor(bool newLine)
{
    static uint8_t lineNr = 0;
    static char escseq[16];
    if (newLine) {
        lineNr = (lineNr + 1) % MAX_PRINT_LINES;
        snprintf(escseq, sizeof(escseq), "\033[%d;1H" ESC_CLEAR_LINE, (lineNr + 5 + BOARD_INSTANCES_MAX));
    } else {
        snprintf(escseq, sizeof(escseq), "\033[%d;1H", (lineNr + 5 + BOARD_INSTANCES_MAX));
    }
    return escseq;
}

static void PrintMenu(void)
{
    PRINT("%s=== Available Keys ===", MoveCursor(true));
    PRINT("%s m - print this menu", MoveCursor(true));
    PRINT("%s r - soft reset", MoveCursor(true));
    PRINT("%s c - clear screen", MoveCursor(true));
    PRINT("%s s - clear statisitcs", MoveCursor(true));
    PRINT("%s i - toggle stress tx test", MoveCursor(true));
    PRINT("%s======================\r\n", MoveCursor(true));
}

static void CheckUartInput(void)
{
    static uint8_t m_rx = 0;

    if (m_rx && SERCOM1_USART_ReadCountGet()) {
        switch(m_rx) {
            case 'M':
            case 'm':
                PrintMenu();
                break;
            case 'R':
            case 'r':
                __NVIC_SystemReset();
                while(true);
                break;
            case 'C':
            case 'c':
                PRINT(ESC_CLEAR_TERMINAL ESC_CURSOR_X1Y1 ESC_HIDE_CURSOR);
                break;
            case 'S':
            case 's':
                memset(m.stats, 0, sizeof(m.stats));
                break;
            case 'I':
            case 'i':
                m.allowTxStress = !m.allowTxStress;
                PRINT("%sStress is %s\r\n", MoveCursor(true), m.allowTxStress ? "enabled" : "disabled");
                break;
            default:
                PRINT("%sUnknown key='%c'(0x%X)\r\n", MoveCursor(true), m_rx, m_rx);
                break;
        }
    }
    if (!SERCOM1_USART_ReadIsBusy()) {
        SERCOM1_USART_Read(&m_rx, 1);
    }
}

static void OnSendIperf(void *pDummy, const uint8_t *pTx, uint16_t len, uint32_t idx, void *pDummy2)
{
    m.txBusy = false;
    DBG_PRINT("ip %i\r\n", m.txBusy);
}

static void SendIperfPacket(void)
{
    if (m.allowTxStress && !m.txBusy) {
        uint32_t len = sizeof(iperf);
        uint16_t i = UDP_PAYLOAD_OFFSET;
        iperf[i++] = BOARD_INSTANCE;
        iperf[i++] = (m.iperfTx >> 24) & 0xFF;
        iperf[i++] = (m.iperfTx >> 16) & 0xFF;
        iperf[i++] = (m.iperfTx >> 8) & 0xFF;
        iperf[i++] = (m.iperfTx) & 0xFF;
        m.txBusy = true;
        if (TC6NoIP_SendEthernetPacket(m.idxNoIp, iperf, len, OnSendIperf)) {
            m.iperfTx++;
            m.stats[BOARD_INSTANCE].packetCntCurrent++;
            m.stats[BOARD_INSTANCE].packetCntTotal++;
            m.stats[BOARD_INSTANCE].byteCntCurrent += len;
            m.stats[BOARD_INSTANCE].byteCntTotal += len;
        } else {
            m.txBusy = false;
        }
    }
}

static void CheckButton(uint8_t instance, bool newLevel, bool *oldLevel)
{
    if (newLevel != *oldLevel) {
        *oldLevel = newLevel;
        if (0 == instance && !newLevel) {
            /* Do something with the button */
        }
    }
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*                      Callback from NO IP component                   */
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void TC6NoIP_CB_OnEthernetReceive(int8_t idx, const uint8_t *pRx, uint16_t len)
{
    if (len >= (UDP_PAYLOAD_OFFSET + 5)) {
        uint16_t i = UDP_PAYLOAD_OFFSET;
        uint8_t idx = pRx[i++];
        if (idx < BOARD_INSTANCES_MAX) {
            uint32_t val = 0;
            uint32_t previous = m.stats[idx].previousVal;
            val |= pRx[i++] << 24;
            val |= pRx[i++] << 16;
            val |= pRx[i++] << 8;
            val |= pRx[i++];

            if (previous) {
                if ((previous + 1) != val) {
                    m.stats[idx].errors++;
                }
            }
            m.stats[idx].previousVal = val;

            m.stats[idx].byteCntCurrent += len;
            m.stats[idx].byteCntTotal += len;
            m.stats[idx].packetCntCurrent++;
            m.stats[idx].packetCntTotal++;
        }
    }
}