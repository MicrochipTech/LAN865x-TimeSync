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

#ifndef PTP_TASK_H
#define	PTP_TASK_H

#pragma once

#include <stdint.h>
#include <filters.h>

#include "cmsis_gcc.h"

#ifndef static_htons
#define static_htons(x) (((x & 0xff) << 8) | (x >> 8))
#endif

#ifndef htons
#define htons(x) ((uint16_t)__REV16(x))
#endif

#ifndef htonl
#define htonl(x) ((uint32_t)__REV(x))
#endif


#define SEC_IN_NS (1000000000llu)
#define SEC_IN_US (1000000u)
#define SEC_IN_MS (1000u)

#define PTP_ETHER_TYPE_H 0x88u
#define PTP_ETHER_TYPE_L 0xF7u

/* LAN8650 registers */
#define OA_IMASK1           (0x0000000Du)
#define OA_IMASK1_SEVM_Pos 28u
#define OA_IMASK1_SEVM_Msk (1U << OA_IMASK1_SEVM_Pos)

#define PADCTRL             (0x000A0088u)
#define MAC_TISUBN          (0x0001006Fu)
#define MAC_TSH             (0x00010070u)
#define MAC_TSL             (0x00010074u)
#define MAC_TN              (0x00010075u)
#define MAC_TA              (0x00010076u)
#define MAC_TI              (0x00010077u)
#define PACYC               (0x000A021Fu)
#define PACTRL              (0x000A0220u)
#define EG0STNS             (0x000A0221u)
#define EG0STSECL           (0x000A0222u)
#define EG0STSECH           (0x000A0223u)
#define EG0PW               (0x000A0224u)
#define EG0IT               (0x000A0225u)
#define EG0CTL              (0x000A0226u)
#define PPSCTL              (0x000A0239u)

#define SEVINTEN (0x000A023Au)
#define SEVINTEN_EG0DONE_Pos 16u
#define SEVINTEN_EG0DONE_Msk (1u << SEVINTEN_EG0DONE_Pos)
#define SEVINTEN_EG1DONE_Pos 17u
#define SEVINTEN_EG1DONE_Msk (1u << SEVINTEN_EG1DONE_Pos)
#define SEVINTEN_EG2DONE_Pos 18u
#define SEVINTEN_EG2DONE_Msk (1u << SEVINTEN_EG2DONE_Pos)
#define SEVINTEN_EG3DONE_Pos 19u
#define SEVINTEN_EG3DONE_Msk (1u << SEVINTEN_EG3DONE_Pos)
#define SEVINTEN_PPSDONE_Pos 30u
#define SEVINTEN_PPSDONE_Msk (1u << SEVINTEN_PPSDONE_Pos)

    
#define SEVINTDIS (0x000A023Bu)
#define SEVINTDIS_EG0DONE_Pos 16u
#define SEVINTDIS_EG0DONE_Msk (1u << SEVINTDIS_EG0DONE_Pos)
#define SEVINTDIS_EG1DONE_Pos 17u
#define SEVINTDIS_EG1DONE_Msk (1u << SEVINTDIS_EG1DONE_Pos)
#define SEVINTDIS_EG2DONE_Pos 18u
#define SEVINTDIS_EG2DONE_Msk (1u << SEVINTDIS_EG2DONE_Pos)
#define SEVINTDIS_EG3DONE_Pos 19u
#define SEVINTDIS_EG3DONE_Msk (1u << SEVINTDIS_EG3DONE_Pos)

#define SEVIM (0x000A023Cu)
#define SEVIM_EG0DONE_Pos 16u
#define SEVIM_EG0DONE_Msk (1u << SEVIM_EG0DONE_Pos)
#define SEVIM_EG1DONE_Pos 17u
#define SEVIM_EG1DONE_Msk (1u << SEVIM_EG1DONE_Pos)
#define SEVIM_EG2DONE_Pos 18u
#define SEVIM_EG2DONE_Msk (1u << SEVIM_EG2DONE_Pos)
#define SEVIM_EG3DONE_Pos 19u
#define SEVIM_EG3DONE_Msk (1u << SEVIM_EG3DONE_Pos)

#define SEVSTS (0x000A023Du)
#define SEVSTS_EG0DONE_Pos 16u
#define SEVSTS_EG0DONE_Msk (1u << SEVSTS_EG0DONE_Pos)
#define SEVSTS_EG1DONE_Pos 17u
#define SEVSTS_EG1DONE_Msk (1u << SEVSTS_EG1DONE_Pos)
#define SEVSTS_EG2DONE_Pos 18u
#define SEVSTS_EG2DONE_Msk (1u << SEVSTS_EG2DONE_Pos)
#define SEVSTS_EG3DONE_Pos 19u
#define SEVSTS_EG3DONE_Msk (1u << SEVSTS_EG3DONE_Pos)

#define EG0CTL_START        (1u << 0)
#define EG0CTL_STOP         (1u << 1)
#define EG0CTL_AH           (1u << 2)
#define EG0CTL_REP          (1u << 3)
#define EG0CTL_ISREL        (1u << 4)

#define UNINIT 0
#define MATCHFREQ 1
#define HARDSYNC 2
#define COARSE 3
#define FINE 4

/// Minimum 8ms, Maximum 16000ms
#define PTP_SYNC_INTERVAL 500u
#define PTP_SYNC_INTERVAL_LOG ((uint8_t)(-1))	// 2^-1 = 0.5sec

/// Minimum 8ms, Maximum 16000ms
#define PTP_PDELAYREQ_INTERVAL 1000u
#define PTP_PDELAYREQ_INTERVAL_LOG 0u

/// Minimum 125ms, Maximum 16000ms
#define PTP_ANNOUNCE_INTERVAL 1000u
#define PTP_ANNOUNCE_INTERVAL_LOG 0u

#define PROPAGATION_MAX_THRESHOLD 3500u

#define CLOCK_ID0	0xFFu
#define CLOCK_ID1	0xFEu
#define PORT_ID		0x0001u

#define PTP_802AS_DEST_MAC0 0x01u
#define PTP_802AS_DEST_MAC1 0x80u
#define PTP_802AS_DEST_MAC2 0xC2u
#define PTP_802AS_DEST_MAC3 0x00u
#define PTP_802AS_DEST_MAC4 0x00u
#define PTP_802AS_DEST_MAC5 0x0Eu

#define ETHERNET_MAC0     0x00
#define ETHERNET_MAC1     0x04
#define ETHERNET_MAC2     0x25
#define ETHERNET_MAC3     0x1c
#define ETHERNET_MAC4     0xa0
#define ETHERNET_MAC5     0x01

#define DEBUG8 0xAAu
#define DEBUG16 0x55AAu
#define DEBUG32 0xDEADBEEFu

#define PTP_VERSION1                  (0x01u)
#define PTP_VERSION2                  (0x02u)

#define PTP_TSP_ETHERNET_AVB          (0x01u)

#define PTP_FLAG_ALTERNATEMASTER      (0)
#define PTP_FLAG_TWOSTEPFLAG		      (1 << 1)
#define PTP_FLAG_UNICASTFLAG		      (1 << 2)
#define PTP_FLAG_TIMESCALE            (1 << 3)

#define MATCHFREQ_RESET_THRESHOLD       100000000
#define HARDSYNC_RESET_THRESHOLD        0x3FFFFFFFu
#define HARDSYNC_THRESHOLD              0xFFFFFF
#define HARDSYNC_COARSE_THRESHOLD       90
#define HARDSYNC_FINE_THRESHOLD         50

typedef enum
{
	PTP_DISABLED,
	PTP_MASTER,
	PTP_SLAVE
} ptpMode_t;

typedef enum
{
  MSG_SYNC              = 0x00,
  MSG_DELAY_REQ         = 0x01,
  MSG_PDELAY_REQ        = 0x02,
  MSG_PDELAY_RESP       = 0x03,
  MSG_FOLLOW_UP         = 0x08,
  MSG_DELAY_RESP        = 0x09,
  MSG_PDELAY_RESP_FUP   = 0x0A,
  MSG_ANNOUNCE          = 0x0B
} ptpMsgType_t;

typedef struct
{
  uint16_t              secondsMsb;		// Some embedded HW implementations only
  uint32_t              secondsLsb;		// support a 32 bit counter for seconds
  uint32_t              nanoseconds;
  uint64_t              correctionField;
} timeStamp_t;

#pragma pack(1)

typedef struct {
  uint8_t destMacAddr[6];
  uint8_t srcMacAddr[6];
  uint8_t ethType[2];
} ethHeader_t;

typedef uint8_t clockIdentity_t[8];

typedef struct
{
  uint16_t              secondsMsb;		// Some embedded HW implementations only
  uint32_t              secondsLsb;		// support a 32 bit counter for seconds
  uint32_t              nanoseconds;
} ptpTimeStamp_t;

typedef struct
{
  clockIdentity_t       clockIdentity;
  uint16_t              portNumber;
} portIdentity_t;

typedef struct 
{
  uint8_t               clockClass;
  uint8_t               clockAccuracy;
  uint16_t              offsetScaledLogVariance;
} clockQuality_t;

typedef struct 
{
  uint16_t              tlvType;
  uint16_t              lengthField;
  clockIdentity_t       pathSequence;
} tlv_t;

typedef struct  
{
  uint16_t              tlvType;
  uint16_t              lengthField;
  uint8_t               organizationId[3];
  uint8_t               organizationSubType[3];
  uint32_t              cumulativescaledRateOffset;
  uint16_t              gmTimeBaseIndicator;
  uint8_t               lastGmPhaseChange[12];
  uint32_t              scaledLastGmFreqChange;
} tlv_followUp_t;

typedef struct
{
  uint8_t               tsmt;
  uint8_t               version;
  uint16_t              messageLength;
  uint8_t               domainNumber;
  uint8_t               reserved2;
  uint8_t               flags[2];
  int64_t               correctionField;
  uint8_t               reserved3[4];
  portIdentity_t        sourcePortIdentity;
  uint16_t              sequenceID;
  uint8_t               controlField;
  uint8_t               logMessageInterval;
} ptpHeader_t;

typedef struct
{
  ptpHeader_t           header;
  ptpTimeStamp_t        originTimestamp;
  uint16_t              currentUtcOffset;
  uint8_t               reserved1;
  uint8_t               grandmasterPriority1;
  clockQuality_t        grandmasterClockQuality;
  uint8_t               grandmasterPriority2;
  clockIdentity_t       grandmasterIdentity;
  uint16_t              stepsRemoved;
  uint8_t               timeSource;
  tlv_t                 tlv;	
} announceMsg_t;

typedef struct _syncMsg
{
  ptpHeader_t           header;
  ptpTimeStamp_t        originTimestamp;
} syncMsg_t;

typedef struct
{
  ptpHeader_t           header;
  ptpTimeStamp_t        preciseOriginTimestamp;
  tlv_followUp_t        tlv;
} followUpMsg_t;

typedef struct 
{
  ptpHeader_t           header;
  ptpTimeStamp_t        originTimestamp;
  uint8_t               reserved[10];
} pdelayReqMsg_t;

typedef struct
{
  ptpHeader_t           header;
  ptpTimeStamp_t        receiveReceiptTimestamp;
  portIdentity_t        requestingPortIdentity;
} pdelayRespMsg_t;

typedef struct
{
  ptpHeader_t           header;
  ptpTimeStamp_t        responseOriginTimestamp;
  portIdentity_t        requestingPortIdentity;
} pdelayRespFollowUpMsg_t;
#pragma pack()

typedef struct
{
  timeStamp_t origin;
  timeStamp_t origin_prev;
  timeStamp_t receipt;
  timeStamp_t receipt_prev;
} ptpSync_ct;

typedef struct
{
  timeStamp_t req_origin;
  timeStamp_t req_origin_prev;
  timeStamp_t req_receipt;
  timeStamp_t req_receipt_prev;
  timeStamp_t resp_origin;
  timeStamp_t resp_receipt;
} ptpDelayReq_ct;

announceMsg_t* preparePtpAnnounceMsg(uint8_t* msgBuffer);
syncMsg_t* preparePtpSyncMsg(uint8_t* msgBuffer);
followUpMsg_t* preparePtpFollowUp(uint8_t* msgBuffer);
pdelayReqMsg_t* preparePtpPathDelayRequest(uint8_t* msgBuffer);
pdelayRespMsg_t* preparePtpPathDelayResponse(uint8_t* msgBuffer);
pdelayRespFollowUpMsg_t* preparePtpPathDelayResponseFollowUp(uint8_t* msgBuffer);

void ptpTask(void);
void resetSync();
uint64_t tsToInternal(const timeStamp_t* ts);

void handlePtp(uint8_t* pData, uint32_t size, uint32_t sec, uint32_t nsec);



#endif	/* PTP_TASK_H */

