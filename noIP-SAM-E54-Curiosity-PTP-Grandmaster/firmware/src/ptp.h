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

#ifndef PTP_H
#define	PTP_H

#define STATIC_OFFSET               7650
#define MAX_MAC_TN_VAL              0x3B9ACA00
#define SYNC_MESSAGE_PERIOD_MS      125
#define SYN_MESSAGE_CLEAR_TIME_MS   5
#define MAX_NUM_REG_RETRIES         5

#define MAX_DELAY_MS                50

#define BUFFER_HEADER_LEN           14
const uint8_t buffer_header[BUFFER_HEADER_LEN] = {0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e, 0x40, 0x84, 0x32, 0x7d, 0x07, 0xfa, 0x88, 0xf7};

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

typedef enum
{
    PTP_STATE_send_sync = 0,
    PTP_STATE_get_tx_status,
    PTP_STATE_get_oa_status0,
    PTP_STATE_get_timestamp_reg_sec,
	PTP_STATE_get_timestamp_reg_nsec,
    PTP_STATE_clear_status_reg,
    PTP_STATE_send_followup
}enum_PTP_task_state;

#endif	/* PTP_H */

