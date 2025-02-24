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
//DOM-IGNORE-END                                                                      */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <tc6.h>
#include <tc6-regs.h>
#include <math.h>
#include <stdlib.h>
#include "ptp_task.h"
#include "tc6.h"
#include "tc6-noip.h"
#include "cmsis_gcc.h"
#define PTP_LOG printf
#include <filters.h>

ptpSync_ct      TS_SYNC;
extern TC6_t* macPhy;
static ptpMode_t ptpMode = PTP_DISABLED;

static int32_t ptp_sync_sequenceId = -1;
static uint8_t syncReceived = 0;
static bool wallClockSet = false;

volatile uint8_t sendPtpSyncFlag = 0u;
volatile uint8_t sendPtpFollowUpFlag = 0u;

volatile double rateRatio = 1.0;
volatile double rateRatioIIR = 1.0;
volatile double rateRatioFIR = 1.0;

volatile double offsetIIR = 0;
volatile double offsetFIR = 0;
static uint8_t ptpSynced = 0;

volatile bool prr = false;
volatile int hardResync = 0;
static uint8_t syncStatus = UNINIT;
static uint32_t runs = 0;
static uint64_t diffLocal = 0;
static uint64_t diffRemote = 0;

volatile uint32_t g_correction = 0;

volatile int64_t offset = 0;
volatile uint64_t offset_abs = 0;
volatile uint8_t sendPdelayRespFup = 0;

static double rateRatioValue[FIR_FILER_SIZE] = {0};
static lpfStateF rateRatiolpfState;

static int32_t offsetValue[FIR_FILER_SIZE_FINE] = {0};
static lpfState offsetState;

static int32_t offsetCoarseValue[FIR_FILER_SIZE_FINE] = {0};
static lpfState offsetCoarseState;
long double continiousratio = 1.0;

static int32_t diff = 0;
static int32_t filteredDiff = 0;
long double corrNs = 0.0;
long double corrNsFlt = 0.0;

void processSync(syncMsg_t* ptpPkt);
void processFollowUp(followUpMsg_t* ptpPkt);
void regCallBack(TC6_t *pInst, bool success, uint32_t addr, uint32_t value, void *pTag, void *pGlobalTag);

void resetSlaveNode() {
    
    printf("GM_RESET -> Slave node reset initiated due to sequence ID mismatch \n");
    ptp_sync_sequenceId = -1;
    syncReceived = 0;
    wallClockSet = false;
    ptpSynced = 0;
    syncStatus = UNINIT;
    runs = 0;
    
    memset(&TS_SYNC, 0, sizeof(ptpSync_ct));
    
    for(uint32_t x = 0; x < FIR_FILER_SIZE_FINE; x++) {
        firLowPassFilter(0, &offsetCoarseState);
        firLowPassFilter(0, &offsetState);
    }
    for(uint32_t x = 0; x < FIR_FILER_SIZE; x++) {
        firLowPassFilterF(1.0, &rateRatiolpfState);
    }
    
    ptpTask();
    
}

static uint64_t BSWAP64(uint64_t rawValue)
{
  uint32_t high = (uint32_t)((rawValue >> 32u) & 0xFFFFFFFFu);
  uint32_t low  = (uint32_t)((rawValue >>  0u) & 0xFFFFFFFFu);

  return ( ((uint64_t)__REV(low)) << 32u ) | ( (uint64_t)__REV(high) );
}

static int64_t getCorrectionField(ptpHeader_t* hdr)
{
  return BSWAP64(hdr->correctionField) >> 16;
}

uint64_t tsToInternal(const timeStamp_t* ts)
{
  uint64_t seconds = ((uint64_t)ts->secondsMsb << 32u | ts->secondsLsb);
  return (seconds * SEC_IN_NS) + ts->nanoseconds;
}

void processSync(syncMsg_t* ptpPkt)
{
  uint16_t seqId = htons(ptpPkt->header.sequenceID);
  
  if(ptp_sync_sequenceId < 0)
  {
    ptp_sync_sequenceId = seqId;
    syncReceived = 0;
  }
  else
  {
        int sequenceDifference = abs(seqId - ptp_sync_sequenceId);
        if (sequenceDifference > 10) {  // Adjust threshold based on acceptable range
            PTP_LOG("Large sequence mismatch detected: %hu - %ld. Resetting sync...\r\n", seqId, ptp_sync_sequenceId);
            resetSlaveNode();
        } else if (ptp_sync_sequenceId == seqId) {
            syncReceived = 1;
        } else {
            syncReceived = 0;
            PTP_LOG("Sync sequenceId mismatch. Is: %hu - %ld\r\n", seqId, ptp_sync_sequenceId);
            ptp_sync_sequenceId = -1;
        }
  }
}

void processFollowUp(followUpMsg_t* ptpPkt)
{
    macPhy = get_macPhy_inst();
  uint16_t seqId = htons(ptpPkt->header.sequenceID);
  if(ptp_sync_sequenceId >= 0 && syncReceived)
  {
    if(ptp_sync_sequenceId == seqId)
    {
      ptp_sync_sequenceId = (ptp_sync_sequenceId + 1u) % UINT16_MAX;
      syncReceived = 0;
    }
    else
    {
      PTP_LOG("FollowUp sequenceId missmatch. Is: %hu - %ld\r\n", seqId, ptp_sync_sequenceId);
      ptp_sync_sequenceId = -1;
      memset(&TS_SYNC.receipt, 0, sizeof(ptpTimeStamp_t));
      memset(&TS_SYNC.receipt_prev, 0, sizeof(ptpTimeStamp_t));
      return;
    }
  }
  else
  {
    ptp_sync_sequenceId = (seqId + 1u) % UINT16_MAX;
    PTP_LOG("FollowUp sequenceId out of sync. Is: %hu - %ld\r\n", seqId, ptp_sync_sequenceId);
    return;
  }
  
  /* Get t1 from PTP frame */
  TS_SYNC.origin.secondsMsb  = htons( ptpPkt->preciseOriginTimestamp.secondsMsb  );
  TS_SYNC.origin.secondsLsb  = htonl( ptpPkt->preciseOriginTimestamp.secondsLsb  );
  TS_SYNC.origin.nanoseconds = htonl( ptpPkt->preciseOriginTimestamp.nanoseconds );
  TS_SYNC.origin.correctionField = getCorrectionField( &ptpPkt->header ) >> 16;
    
  if(hardResync)
  {
    TC6_WriteRegister(macPhy, MAC_TSL, TS_SYNC.origin.secondsLsb, true, 0, 0);
    TC6_WriteRegister(macPhy, MAC_TN, TS_SYNC.origin.nanoseconds, true, 0, 0);
    PTP_LOG("Large offset, doing hard sync\r\n");
    hardResync = 0;
  }
  
  if(ptpSynced && !wallClockSet )
  {
      // Enable PPS after PTP synced 
      while (!TC6_WriteRegister(macPhy, PPSCTL, 0x000007Du, true, 0, 0)) 
      {
        TC6_Service(macPhy, true);
      }

      wallClockSet = true;
  }
  
  /* Convert to internal time format */
  uint64_t t1 = tsToInternal(&TS_SYNC.origin);
  uint64_t t2 = tsToInternal(&TS_SYNC.receipt);
  
  if(TS_SYNC.receipt_prev.secondsLsb != 0)
  {
    uint64_t curr = t2;
    uint64_t prev = tsToInternal(&TS_SYNC.receipt_prev);
    diffLocal = curr - prev;
  }
  
  if(TS_SYNC.origin_prev.secondsLsb != 0)
  {
    uint64_t curr = t1;
    uint64_t prev = tsToInternal(&TS_SYNC.origin_prev);
    diffRemote = curr - prev;
  }

  TS_SYNC.receipt_prev = TS_SYNC.receipt;
  TS_SYNC.origin_prev = TS_SYNC.origin;
  
  /* Calculate rateRatio */
  if(diffLocal && diffRemote)
  {
    if(syncStatus == UNINIT || syncStatus > HARDSYNC) 
    {
    rateRatio = (double)diffRemote / (double)diffLocal; //lowPassExponential( (double)diffRemote / (double)diffLocal, rateRatio, 0.8f);
      if((rateRatio > 0.998 && rateRatio < 1.002)) 
      {
        rateRatioIIR = lowPassExponential( (double)diffRemote / (double)diffLocal, rateRatio, 0.5f);
        rateRatioFIR = firLowPassFilterF( (double)diffRemote / (double)diffLocal, &rateRatiolpfState );
      }
      else {
        PTP_LOG("Filtered rateRatio outlier\r\n");
      }        
    }    
    runs++;
  }
  else
  {
    printf("!");
    return;
  }
  
  offset = t2 - t1;
  uint8_t neg = 1;
  if(offset < 0) neg = 0;
  offset_abs = llabs(offset);
  
  if(syncStatus == UNINIT)
  {
    if(runs >= (FIR_FILER_SIZE*1))
    {
      double calcInc = CLOCK_CYCLE_NS * rateRatioFIR;
      
      uint8_t mac_ti = (uint8_t)calcInc; 
      double calcSubInc = calcInc - (double)mac_ti;
      calcSubInc *= 16777216.0;
      uint32_t calcSubInc_uint = (uint32_t)calcSubInc;
      calcSubInc_uint = ((calcSubInc_uint >> 8) & 0xFFFF) | ((calcSubInc_uint & 0xFF) << 24);
      
      TC6_WriteRegister(macPhy, MAC_TISUBN, calcSubInc_uint, true, 0, 0);
      TC6_Service(macPhy, true);
      TC6_WriteRegister(macPhy, MAC_TI, (uint32_t)mac_ti, true, 0, 0);
      TC6_Service(macPhy, true);
      if(prr) PTP_LOG("MAC_TI %li\r\n",(uint32_t)mac_ti );
      if(prr) PTP_LOG("MAC_TISUBN %li\r\n",(uint32_t)calcSubInc_uint );
      
      if(syncStatus == UNINIT) syncStatus = MATCHFREQ;
      ptpSynced = 1;
      runs = 0;
    }      
  }
  else if(syncStatus == MATCHFREQ)
  {
    if(offset_abs > MATCHFREQ_RESET_THRESHOLD) {
      hardResync = 1;
    }
    else {
      syncStatus = HARDSYNC;
    }
  }
  else if(syncStatus >= HARDSYNC)
  {
    if(offset_abs > HARDSYNC_RESET_THRESHOLD)
    {
        syncStatus == UNINIT;
        for(uint32_t x=0; x<FIR_FILER_SIZE_FINE ; x++)
        {
            (void) firLowPassFilter(0, &offsetCoarseState);
            (void) firLowPassFilter(0, &offsetState);
        }
        for(uint32_t x=0; x<FIR_FILER_SIZE ; x++)
        {
            (void) firLowPassFilterF( 1.0 , &rateRatiolpfState );
        }
        runs=0;
    }
    else if(offset_abs > HARDSYNC_THRESHOLD) 
    {
      offset_abs = HARDSYNC_THRESHOLD;
      TC6_WriteRegister(macPhy, MAC_TA, ((neg & 1) << 31) | offset_abs, true, 0, 0);
      TC6_Service(macPhy, true);
      syncStatus = HARDSYNC;
    }
    else if(offset_abs > HARDSYNC_COARSE_THRESHOLD)
    {
      for(uint32_t x=0; x<FIR_FILER_SIZE_FINE ; x++)
      {
            (void) firLowPassFilter(0, &offsetCoarseState);
            (void) firLowPassFilter(0, &offsetState);
            offsetCoarseState.filled = 0;
            offsetState.filled = 0;
      }
      TC6_WriteRegister(macPhy, MAC_TA, ((neg & 1) << 31) | offset_abs, true, 0, 0);
      TC6_Service(macPhy, true);
      syncStatus = HARDSYNC;
    }
    else if(offset_abs > HARDSYNC_FINE_THRESHOLD)
    {
      for(uint32_t x=0; x<FIR_FILER_SIZE_FINE ; x++)
      {
            (void) firLowPassFilter(0, &offsetState);
            offsetState.filled = 0;
      }
      offsetFIR = firLowPassFilter(offset, &offsetCoarseState);
      neg = (offsetFIR < 0) ? 0 : 1;
      int32_t write_val = 0;
      write_val = (int32_t) offsetFIR;
      if(!neg) write_val = write_val*(-1);

      TC6_WriteRegister(macPhy, MAC_TA, (((neg & 1) << 31) | ((uint32_t)write_val)), true, 0, 0);
      TC6_Service(macPhy, true);
      syncStatus = COARSE;
      if(prr) PTP_LOG("Offset:%lld,  Pos: %i, Offset Coarse: %li\r\n", offset, neg, ((uint32_t)write_val));
    }
    else
    {
      offsetFIR = firLowPassFilter(offset, &offsetState);
      neg = (offsetFIR < 0) ? 0 : 1;
      int32_t write_val = 0;
      write_val = (int32_t) offsetFIR;
      if(!neg) write_val = write_val*(-1);

      TC6_WriteRegister(macPhy, MAC_TA, (((neg & 1) << 31) | ((uint32_t)write_val)), true, 0, 0);
      TC6_Service(macPhy, true);
      
      syncStatus = FINE;
      if(prr) PTP_LOG("Offset:%lld,  Pos: %i, Offset Fine: %li\r\n", offset, neg, ((uint32_t)write_val));
    }
  }
}


void handlePtp(uint8_t* pData, uint32_t size, uint32_t sec, uint32_t nsec)
{
  (void) size;
  ptpHeader_t* ptpPkt = 0;
  ptpPkt = (ptpHeader_t*)(pData + sizeof(ethHeader_t));
  
  uint8_t messageType = ptpPkt->tsmt & 0xFu;
  
  if(messageType == MSG_FOLLOW_UP)
  {
    processFollowUp((followUpMsg_t*)ptpPkt);
  }    
  else if(messageType == MSG_SYNC)
  {
      processSync((syncMsg_t*)ptpPkt);
      if(syncReceived)
      {
          TS_SYNC.receipt_prev = TS_SYNC.receipt;
          TS_SYNC.receipt.secondsLsb = sec;
          TS_SYNC.receipt.nanoseconds = nsec;
      }
  }
}


void ptpTask(void)
{
    macPhy = get_macPhy_inst();
    
    while (!TC6_WriteRegister(macPhy, PPSCTL, 0x00000002u, true, 0, 0)) {
      TC6_Service(macPhy, true);
    }
    
    while (!TC6_WriteRegister(macPhy, SEVINTEN, SEVINTEN_PPSDONE_Msk, true, 0, 0)) {
      TC6_Service(macPhy, true);
    }    
    
    memset(&TS_SYNC, 0, sizeof(TS_SYNC)); 
    ptpMode = PTP_SLAVE;
    
    rateRatiolpfState.buffer = &rateRatioValue[0];
    rateRatiolpfState.filterSize = sizeof(rateRatioValue) / sizeof(rateRatioValue[0]);
    
    offsetState.buffer = &offsetValue[0];
    offsetState.filterSize = sizeof(offsetValue) / sizeof(offsetValue[0]);
    offsetCoarseState.buffer = &offsetCoarseValue[0];
    offsetCoarseState.filterSize = sizeof(offsetCoarseValue) / sizeof(offsetCoarseValue[0]);
}