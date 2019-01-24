/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  AIC-6915 (10/100 Ethernet) registers           File: aic6915.h
    *  
    *********************************************************************  
    *
    *  Copyright 2002
    *  Broadcom Corporation. All rights reserved.
    * 
    *  This software is furnished under license and may be used and 
    *  copied only in accordance with the following terms and conditions.  
    *  Subject to these conditions, you may download, copy, install, 
    *  use, modify and distribute modified or unmodified copies of 
    *  this software in source and/or binary form.  No title or ownership 
    *  is transferred hereby.
    * 
    *  1) Any source code used, modified or distributed must reproduce 
    *     and retain this copyright notice and list of conditions as 
    *     they appear in the source file.
    * 
    *  2) No right is granted to use any trade name, trademark, or 
    *     logo of Broadcom Corporation.  The "Broadcom Corporation" 
    *     name may not be used to endorse or promote products 
    *     derived from this software without the prior written 
    *     permission of Broadcom Corporation.
    * 
    *  3) THIS SOFTWARE IS PROVIDED "AS-IS" AND ANY EXPRESS OR IMPLIED
    *     WARRANTIES, INCLUDING BUT NOT LIMITED TO, ANY IMPLIED 
    *     WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
    *     PURPOSE, OR NON-INFRINGEMENT ARE DISCLAIMED. IN NO EVENT 
    *     SHALL BROADCOM BE LIABLE FOR ANY DAMAGES WHATSOEVER, AND 
    *     IN PARTICULAR, BROADCOM SHALL NOT BE LIABLE FOR DIRECT, 
    *     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
    *     DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    *     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
    *     PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
    *     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
    *     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE), 
    *     EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ********************************************************************* */

#ifndef _AIC6915_H_
#define _AIC6915_H_

/*
 * Register and bit definitions for the Adaptec AIC-6915 10/100
 * Ethernet controller ("Starfire").
 *
 * Reference:
 *   AIC-6915 Ethernet LAN Controller Programmer's Manual
 *   Stock No. 512130-00, Rev. A
 *   Adaptec, Inc., Milpitas CA 95035, September 1998
 */

#define K_PCI_VENDOR_ADAPTEC   0x9004
#define K_PCI_ID_AIC6915       0x6915

#define _DD_MAKEMASK1(n) (1 << (n))
#define _DD_MAKEMASK(v,n) ((((1)<<(v))-1) << (n))
#define _DD_MAKEVALUE(v,n) ((v) << (n))
#define _DD_GETVALUE(v,n,m) (((v) & (m)) >> (n))


/* Offsets in the comments below are for accesses via PCI memory
   space, but register identifiers are relative to 0x50000 in that
   space.  The subspaces at 0x50000-0x500FF (Adaptec's
   "Internal_Functional_Registers") also appear at 0x0000 - 0x00FF in
   i/o and config spaces, with indirect I/O for other registers. */

#define K_AIC_REG_OFFSET                0x00050000


/* Registers 0x00000 - 0x3ffff are EPROM expansion space (not used) */

/* Registers 0x40000 - 0x4ffff are external registers (not supported) */

/* Registers 0x50000 - 0x50040 are PCI configuration registers (shadow) */

/* Registers 0x50040 - 0x5006f are additional PCI registers */

#define R_PCIDeviceConfig               0x0040
#define R_BacControl                    0x0044
#define R_PCIMonitor1                   0x0048
#define R_PCIMonitor2                   0x004C

/* Power Management Capability */
#define R_PMC                           0x0050
#define R_PMCSR                         0x0054
#define R_PMEEvent                      0x0058

#define R_EEPROMControlStatus           0x0060
#define R_PCIComplianceTesting          0x0064
#define R_IndirectioAddress             0x0068
#define R_IndirectioDataPort            0x006C

/* Registers 0x50070 - 0x500ff are Ethernet functional registers */

/* General Ethernet Functional Registers */
#define R_GeneralEthernetCtrl           0x0070
#define R_TimersCtrl                    0x0074
#define R_CurrentTime                   0x0078
#define R_InterruptStatus               0x0080
#define R_ShadowInterruptStatus         0x0084
#define R_InterruptEn                   0x0088
#define R_GPIO                          0x008C

/* Transmit Registers */
#define R_TxDescQueueCtrl               0x0090
#define R_HiPrTxDescQueueBaseAddr       0x0094
#define R_LoPrTxDescQueueBaseAddr       0x0098
#define R_TxDescQueueHighAddr           0x009C
#define R_TxDescQueueProducerIndex      0x00A0
#define R_TxDescQueueConsumerIndex      0x00A4
#define R_TxDmaStatus1                  0x00A8
#define R_TxDmaStatus2                  0x00AC
#define R_TxFrameCtrl                   0x00B0

/* Completion Queue Registers */
#define R_CompletionQueueHighAddr       0x00B4
#define R_TxCompletionQueueCtrl         0x00B8
#define R_RxCompletionQueue1Ctrl        0x00BC
#define R_RxCompletionQueue2Ctrl        0x00C0
#define R_CompletionQueueConsumerIndex  0x00C4
#define R_CompletionQueueProducerIndex  0x00C8
#define R_RxHiPrCompletionPtrs          0x00CC

/* Receive Registers */
#define R_RxDmaCtrl                     0x00D0
#define R_RxDescQueue1Ctrl              0x00D4
#define R_RxDescQueue2Ctrl              0x00D8
#define R_RxDescQueueHighAddress        0x00DC
#define R_RxDescQueue1LowAddress        0x00E0
#define R_RxDescQueue2LowAddress        0x00E4
#define R_RxDescQueue1Ptrs              0x00E8
#define R_RxDescQueue2Ptrs              0x00EC
#define R_RxDmaStatus                   0x00F0
#define R_RxAddressFilteringCtrl        0x00F4

/* Registers 0x50100 - 0x50fff are PCI extra registers */

/* PCI Diagnostic Registers */
#define R_PCITargetStatus               0x0100
#define R_PCIMasterStatus1              0x0104
#define R_PCIMasterStatus2              0x0108
#define R_PCIDmaLowHostAddr             0x010C
#define R_BacDmaDiagnostic0             0x0110
#define R_BacDmaDiagnostic1             0x0114
#define R_BacDmaDiagnostic2             0x0118
#define R_BacDmaDiagnostic3             0x011C

#define R_MacAddr1                      0x0120
#define R_MacAddr2                      0x0124

/* PCI CardBus Registers */
#define R_FunctionEvent                 0x0130
#define R_FunctionEventMask             0x0134
#define R_FunctionPresentState          0x0138
#define R_ForceFunction                 0x013C

/* Registers 0x51000 - 0x51fff are serial EEPROM */

/* Registers 0x52000 - 0x53fff are MII registers */

#define R_MIIRegistersAccessPort        0x2000

#define PHY_REGISTERS                   32

#define M_MiiDataValid                  (1 << 31)
#define M_MiiBusy                       (1 << 30)
#define S_MiiRegDataPort                0
#define M_MiiRegDataPort                (0xFFFF << S_MiiRegDataPort)

/* Registers 0x54000 - 0x54fff are Ethernet extra registers */

#define R_TestMode                      0x4000
#define R_RxFrameProcessorCtrl          0x4004
#define R_TxFrameProcessorCtrl          0x4008

/* Registers 0x55000 - 0x55fff are MAC registers */

#define R_MacConfig1                    0x5000
#define R_MacConfig2                    0x5004
#define R_BkToBkIPG                     0x5008
#define R_NonBkToBkIPG                  0x500C
#define R_ColRetry                      0x5010
#define R_MaxLength                     0x5014
#define R_TxNibbleCnt                   0x5018
#define R_TxByteCnt                     0x501C
#define R_ReTxCnt                       0x5020
#define R_RandomNumGen                  0x5024
#define R_MskRandomNum                  0x5028
#define R_TotalTxCnt                    0x5034
#define R_RxByteCnt                     0x5040
#define R_TxPauseTimer                  0x5060
#define R_VLANType                      0x5064
#define R_MIIStatus                     0x5070

/* Registers 0x56000 - 0x56fff are address filtering */

#define R_PerfectAddressBase            0x6000

#define PERFECT_ADDRESS_ENTRIES         16
#define PERFECT_ADDRESS_STRIDE          0x10

/* Hash and VLAN Tables are interleaved (Table 7-108) */
#define R_HashAddressBase               0x6100

#define HASH_TABLE_STRIDE               0x10
#define HASH_TABLE_ROWS                 32
#define HASH_BIT_OFFSET                 0x0
#define HASH_PRIORITY_OFFSET            0x4
#define HASH_VLAN_OFFSET                0x8

/* Registers 0x57000 - 0x57fff are the statistics register file */

#define R_StatisticsBase                0x7000

#define R_TransmitOKFrames              0x7000
#define R_SingleCollisionFrames         0x7004
#define R_MultipleCollisionFrames       0x7008
#define R_TransmitCRCErrors             0x700C
#define R_TransmitOKOctets              0x7010
#define R_TransmitDeferredFrames        0x7014
#define R_TransmitLateCollision         0x7018
#define R_TransmitPauseFrames           0x701C
#define R_TransmitControlFrames         0x7020
#define R_TransmitExcessiveCollisions   0x7024
#define R_TransmitExcessiveDeferral     0x7028
#define R_MulticastTransmitOKFrames     0x702C
#define R_BroadcastTransmitOKFrames     0x7030
#define R_FramesLostInternalTransmit    0x7034

#define R_ReceiveOKFrames               0x7038
#define R_ReceiveCRCErrors              0x703C
#define R_AlignmentErrors               0x7040
#define R_ReceiveOKOctets               0x7044
#define R_PauseFramesReceivedOK         0x7048
#define R_ControlFramesReceivedOK       0x704C
#define R_UnknownControlFramesReceived  0x7050
#define R_ReceiveFramesTooLong          0x7054
#define R_ReceiveFramesTooShort         0x7058
#define R_ReceiveFramesJabberError      0x705C
#define R_ReceiveFramesFragments        0x7060
#define R_ReceivePackets_64             0x7064
#define R_ReceivePackets_65_127         0x7068
#define R_ReceivePackets_128_255        0x706C
#define R_ReceivePackets_256_511        0x7070
#define R_ReceivePackets_512_1023       0x7074
#define R_ReceivePackets_1024_1518      0x7078
#define R_FramesLostInternalReceive     0x707C

#define R_TransmitFIFOUnderflow         0x7080

#define STATISTICS_COUNT                33

/* Registers 0x58000 - 0x59fff are tx frame processor instruction memory */

#define R_TxGfpMem                      0x8000

/* Registers 0x5a000 - 0x5bfff are rx frame processor instruction memory */

#define R_RxGfpMem                      0xA000

/* Registers 0x5c000 - 0x5dfff are Ethernet FIFO access */

#define R_EthernetFIFO                  0xC000

/* Registers 0x5e000 - 0x5ffff are reserved */


/* 0x0040 PCI Device Config Register */

#define M_EnDpeInt                      (1 << 31)
#define M_EnSseInt                      (1 << 30)
#define M_EnRmaInt                      (1 << 29)
#define M_EnRtaInt                      (1 << 28)
#define M_EnStaInt                      (1 << 27)
#define M_EnDprInt                      (1 << 24)
#define M_IntEnable                     (1 << 23)
#define S_ExternalRegCsWidth            20
#define M_ExternalRegCsWidth            (0x7 << S_ExternalRegCsWidth)
#define M_StopMWrOnCacheLineDis         (1 << 19)
#define S_EpromCsWidth                  16
#define M_EpromCsWidth                  (0x7 << S_EpromCsWidth)
#define M_EnBeLogic                     (1 << 15)
#define M_LatencyStopOnCacheLine        (1 << 14)
#define M_PCIMstDmaEn                   (1 << 13)
#define M_StopOnCachelineEn             (1 << 12)
#define S_FifoThreshold                 8
#define M_FifoThreshold                 (0xF << S_FifoThreshold)
#define M_MemRdCmdEn                    (1 << 7)
#define M_StopOnPerr                    (1 << 6)
#define M_AbortOnAddrParityErr          (1 << 5)
#define M_EnIncrement                   (1 << 4)
#define M_System64                      (1 << 2)
#define M_Force64                       (1 << 1)
#define M_SoftReset                     (1 << 0)

/* 0x0044 BAC Control Register */

#define S_DescSwapMode                  6
#define M_DescSwapMode                  (0x3 << S_DescSwapMode)
#define S_DataSwapMode                  4
#define M_DataSwapMode                  (0x3 << S_DataSwapMode)
#define M_SingleDmaMode                 (1 << 3)
#define M_PreferTxDmaReq                (1 << 2)
#define M_PreferRxDmaReq                (1 << 1)
#define M_BacDmaEn                      (1 << 0)

#define V_DataSwapMode_LE               (0x0 << S_DataSwapMode)
#define V_DataSwapMode_BE               (0x1 << S_DataSwapMode)

/* 0x0048 PCI Monitor1 Register */

#define S_PCIBusMaxLatency              24
#define M_PCIBusMaxLatency              (0xFF << S_PCIBusMaxLatency)
#define S_PCIIntMaxLatency              16
#define M_PCIIntMaxLatency              (0xFF << S_PCIIntMaxLatency)
#define S_PCISlaveBusUtilization        0
#define M_PCISlaveBusUtilization        (0xFFFF << S_PCISlaveBusUtilization)

/* 0x004C PCI Monitor2 Register */

#define S_PCIMasterBusUtilization       16
#define M_PCIMasterBusUtilization       (0xFFFF << S_PCIMasterBusUtilization)
#define S_ActiveTransferCount           0
#define M_ActiveTransferCount           (0xFFFF << S_ActiveTransferCount)


/* 0x0070 General Ethernet Control Register */

#define M_SetSoftInt                    (1 << 8)
#define M_TxGfpEn                       (1 << 5)
#define M_RxGfpEn                       (1 << 4)
#define M_TxDmaEn                       (1 << 3)
#define M_RxDmaEn                       (1 << 2)
#define M_TransmitEn                    (1 << 1)
#define M_ReceiveEn                     (1 << 0)

/* 0x0074 Timers Control Register */

#define M_EarlyRxQ1IntDelayDiasable     (1 << 31)
#define M_RxQ1DoneIntDelayDisable       (1 << 30)
#define M_EarlyRxQ2IntDelayDisable      (1 << 29)
#define M_RxQ2DoneIntDelayDisable       (1 << 28)
#define M_TimeStampResolution           (1 << 26)
#define M_GeneralTimerResolution        (1 << 25)
#define M_OneShotMode                   (1 << 24)
#define S_GeneralTimerInterval          16
#define M_GeneralTimerInterval          (0xFF << S_GeneralTimerInterval)
#define M_TxFrameCompleteIntDelayDisable (1 << 15)
#define M_TxQueueDoneIntDelayDisable    (1 << 14)
#define M_TxDmaDoneIntDelayDisable      (1 << 13)
#define M_RxHiPrBypass                  (1 << 12)
#define M_Timer10X                      (1 << 11)
#define S_SmallRxFrame                  9
#define M_SmallRxFrame                  (0x3 << S_SmallRxFrame)
#define M_SmallFrameBypass              (1 << 8)
#define S_IntMaskMode                   5
#define M_IntMaskMode                   (0x3 << S_IntMaskMode)
#define S_IntMaskPeriod                 0
#define M_IntMaskPeriod                 (0x1F << S_IntMaskPeriod)

/* 0x0078 Current Time Register */

#define M_CurrentTime                   0xFFFFFFFF

/* 0x0080 Interrupt Status Register                        */
/* 0x0084 Shadow Interrupt Status Register                 */
/* 0x0088 Interrupt En Register  (data sheet appends "En") */

#define M_GPIOInt_3                     (1 << 31)
#define M_GPIOInt_2                     (1 << 30)
#define M_GPIOInt_1                     (1 << 29)
#define M_GPIOInt_0                     (1 << 28)
#define M_GPIOInt                       (M_GPIOInt_3 | M_GPIOInt_2 | \
                                         M_GPIOInt_1 | M_GPIOInt_0)
#define M_StatisticWrapInt              (1 << 27)
#define M_AbnormalInterrupt             (1 << 25)
#define M_GeneralTimerInt               (1 << 24)
#define M_SoftInt                       (1 << 23)
#define M_RxCompletionQueue1Int         (1 << 22)
#define M_TxCompletionQueueInt          (1 << 21)
#define M_PCIInt                        (1 << 20)
#define M_DmaErrInt                     (1 << 19)
#define M_TxDataLowInt                  (1 << 18)
#define M_RxCompletionQueue2Int         (1 << 17)
#define M_RxQ1LowBuffersInt             (1 << 16)
#define M_NormalInterrupt               (1 << 15)
#define M_TxFrameCompleteInt            (1 << 14)
#define M_TxDMADoneInt                  (1 << 13)
#define M_TxQueueDoneInt                (1 << 12)
#define M_EarlyRxQ2Int                  (1 << 11)
#define M_EarlyRxQ1Int                  (1 << 10)
#define M_RxQ2DoneInt                   (1 << 9)
#define M_RxQ1DoneInt                   (1 << 8)
#define M_RxGfpNoResponseInt            (1 << 7)
#define M_RxQ2LowBuffersInt             (1 << 6)
#define M_NoTxChecksumInt               (1 << 5)
#define M_TxLowPrMismatchInt            (1 << 4)
#define M_TxHiPrMismatchInt             (1 << 3)
#define M_GfpRxInt                      (1 << 2)
#define M_GfpTxInt                      (1 << 1)
#define M_PCIPadInt                     (1 << 0)

/* 0x008C GPIO Register */

#define M_GPIOCtrl_3                    (1 << 27)
#define M_GPIOCtrl_2                    (1 << 26)
#define M_GPIOCtrl_1                    (1 << 25)
#define M_GPIOCtrl_0                    (1 << 24)
#define M_GPIOCtrl                      (M_GPIOCtrl_3 | M_GPIOCtrl_2 | \
                                         M_GPIOCtrl_1 | M_GPIOCtrl_0)
#define M_GPIOOutMode_3                 (1 << 19)
#define M_GPIOOutMode_2                 (1 << 18)
#define M_GPIOOutMode_1                 (1 << 17)
#define M_GPIOOutMode_0                 (1 << 16)
#define M_GPIOOutMode                   (M_GPIOOutMode_3 | M_GPIOOutMode_2 | \
                                         M_GPIOOutMode_1 | M_GPIOOutMode_0)
#define M_GPIOInpMode_3                 (3 << 14)
#define M_GPIOInpMode_2                 (3 << 12)
#define M_GPIOInpMode_1                 (3 << 10)
#define M_GPIOInpMode_0                 (3 << 8)
#define M_GPIOInpMode                   (M_GPIOInpMode_3 | M_GPIOInpMode_2 | \
                                         M_GPIOInpMode_1 | M_GPIOInpMode_0)
#define V_GPIOInpMode(x,n)              ((x) << (8+2*(n)))
#define G_GPIOInpMode(x,n)              (((x) >> (8+2*(n))) & 0x3)
#define K_GPIOInpMode_Normal            0
#define K_GPIOInpMode_IntHi             1
#define K_GPIOInpMode_IntLo             2
#define K_GPIOInpMode_Change            3
#define M_GPIOData_3                    (1 << 27)
#define M_GPIOData_2                    (1 << 26)
#define M_GPIOData_1                    (1 << 25)
#define M_GPIOData_0                    (1 << 24)
#define M_GPIOData                      (M_GPIOData_3 | M_GPIOData_2 | \
                                         M_GPIOData_1 | M_GPIOData_0)

/* 0x0090 Tx DescQueue Ctrl Register (see 7-46) */

#define S_TxHighPriorityFifoThreshold   24
#define M_TxHighPriorityFifoThreshold   (0xFF << S_TxHighPriorityFifoThreshold)
#define S_SkipLength                    16
#define M_SkipLength                    (0x1F << S_SkipLength)
#define S_TxDmaBurstSize                8
#define M_TxDmaBurstSize                (0x3F << S_TxDmaBurstSize)
#define M_TxDescQueue64bitAddr          (1 << 7)
#define S_MinFrameDescSpacing           4
#define M_MinFrameDescSpacing           (0x7 << S_MinFrameDescSpacing)
#define M_DisableTxDmaCompletion        (1 << 3)
#define S_TxDescType                    0
#define M_TxDescType                    (0x7 << S_TxDescType)

/* 0x0094 HiPr Tx DescQueue Base Addr Register */
/* 0x0098 LoPr Tx DescQueue Base Addr Register */

#define S_TxDescQueueBaseAddress        0            /* bit-aligned */
#define M_TxDescQueueBaseAddress       0xFFFFFF00

/* 0x00A0 Tx DescQueue Producer Index Register */

#define S_HiPrTxProducerIndex           16
#define M_HiPrTxProducerIndex           (0x7FF << S_HiPrTxProducerIndex)
#define S_LoPrTxProducerIndex           0
#define M_LoPrTxProducerIndex           (0x7FF << S_LoPrTxProducerIndex)


/* 0x00A4 Tx DescQueue Consumer Index Register */

#define S_HiPrTxConsumerIndex           16
#define M_HiPrTxConsumerIndex           (0x7FF << S_HiPrTxConsumerIndex)
#define S_LoPrTxConsumerIndex           0
#define M_LoPrTxConsumerIndex           (0x7FF << S_LoPrTxConsumerIndex)

/* 0x00B0 Tx Frame Ctrl/Status Register */

#define S_TxFrameStates                 16
#define M_TxFrameStates                 (0x1FF << S_TxFrameStates)
#define S_TxDebugConfigBits             9
#define M_TxDebugConfigBits             (0x7F << S_TxDebugConfigBits)
#define M_DmaCompletionAfterTransmitComplete (1 << 8)
#define S_TransmitThreshold             0
#define M_TransmitThreshold             (0xFF << S_TransmitThreshold)

/* 0x00B8 Tx Completion Queue Ctrl Register */

#define S_TxCompletionBaseAddress       0            /* bit-aligned */
#define M_TxCompletionBaseAddress       0xFFFFFF00
#define M_TxCompletion64bitAddress      (1 << 7)
#define M_TxCompletionProducerWe        (1 << 6)
#define M_TxCompletionSize              (1 << 5)
#define M_CommonQueueMode               (1 << 4)
#define S_TxCompletionQueueThreshold    0
#define M_TxCompletionQueueThreshold    (0xF << S_TxCompletionQueueThreshold)


/* 0x00BC Rx Completion Queue 1 Ctrl Register */
/* 0x00C0 Rx Completion Queue 2 Ctrl Register */

#define S_RxCompletionBaseAddress       0            /* bit-aligned */
#define M_RxCompletionBaseAddress       0xFFFFFF00
#define M_RxCompletion64bitAddress      (1 << 7)
#define M_RxCompletionProducerWe        (1 << 6)
#define S_RxCompletionType              4
#define M_RxCompletionType              (0x3 << S_RxCompletionType)
#define S_RxCompletionThreshold         0
#define M_RxCompletionThreshold         (0xF << S_RxCompletionQueueThreshold)

/* 0x00C4 Completion Queue Consumer Index Register */

#define M_TxCompletionThresholdMode     (1 << 31)
#define S_TxCompletionConsumerIndex     16
#define M_TxCompletionConsumerIndex     (0x3FF << S_TxCompletionConsumerIndex)
#define V_TxCompletionConsumerIndex(x)  ((x) << S_TxCompletionConsumerIndex)
#define G_TxCompletionConsumerIndex(x)  (((x) & M_TxCompletionConsumerIndex) \
                                           >> S_TxCompletionConsumerIndex)
#define M_RxCompletionQ1ThresholdMode   (1 << 15)
#define S_RxCompletionQ1ConsumerIndex   0
#define M_RxCompletionQ1ConsumerIndex   (0x3FF << S_RxCompletionQ1ConsumerIndex)
#define V_RxCompletionQ1ConsumerIndex(x) ((x) << S_RxCompletionQ1ConsumerIndex)
#define G_RxCompletionQ1ConsumerIndex(x) (((x) & M_RxCompletionQ1ConsumerIndex)\
                                           >> S_RxCompletionQ1ConsumerIndex)

/* 0x00C8 Completion Queue Producer Index Register */

#define S_TxCompletionProducerIndex     16
#define M_TxCompletionProducerIndex     (0x3FF << S_TxCompletionProducerIndex)
#define V_TxCompletionProducerIndex(x)  ((x) << S_TxCompletionProducerIndex)
#define G_TxCompletionProducerIndex(x)  (((x) & M_TxCompletionProducerIndex) \
                                           >> S_TxCompletionProducerIndex)
#define S_RxCompletionQ1ProducerIndex   0
#define M_RxCompletionQ1ProducerIndex   (0x3FF << S_RxCompletionQ1ProducerIndex)
#define V_RxCompletionQ1ProducerIndex(x) ((x) << S_RxCompletionQ1ProducerIndex)
#define G_RxCompletionQ1ProducerIndex(x) (((x) & M_RxCompletionQ1ProducerIndex)\
                                           >> S_RxCompletionQ1ProducerIndex)

/* 0x00CC Rx Hi Pr Completion Ptrs Register */

#define S_RxCompletionQ2ProducerIndex   16
#define M_RxCompletionQ2ProducerIndex   (0x3FF << S_RxCompletionQ2ProducerIndex)
#define M_RxCompletionQ2ThresholdMode   (1 << 15)
#define S_RxCompletionQ2ConsumerIndex   0
#define M_RxCompletionQ2ConsumerIndex   (0x3FF << S_RxCompletionQ2ConsumerIndex)

/* 0x00D0 Rx DMA Ctrl Register */

#define M_RxReportBadFrames             (1 << 31)
#define M_RxDmaShortFrames              (1 << 30)
#define M_RxDmaBadFrames                (1 << 29)
#define M_RxDmaCrcErrorFrames           (1 << 28)
#define M_RxDmaControlFrame             (1 << 27)
#define M_RxDmaPauseFrame               (1 << 26)

#define S_RxChecksumMode                24
#define M_RxChecksumMode                (0x3 << S_RxChecksumMode)
#define K_RxChecksumMode_None           0
#define K_RxChecksumMode_TCP            1
#define K_RxChecksumMode_TCP_UDP        2

#define M_RxCompletionQ2Enable          (1 << 23)

#define S_RxDmaQueueMode                20
#define M_RxDmaQueueMode                (0x7 << S_RxDmaQueueMode)
#define K_RxDmaQueueMode_Q1Only         0
#define K_RxDmaQueueMode_Q2FP           1
#define K_RxDmaQueueMode_Q2Short        2
#define K_RxDmaQueueMode_Q2Priority     3
#define K_RxDmaQueueMode_SplitIP        4

#define M_RxUseBackupQueue              (1 << 19)
#define M_RxDmaCrc                      (1 << 18)
#define S_RxEarlyIntThreshold           12
#define M_RxEarlyIntThreshold           (0x1F << S_RxEarlyIntThreshold)
#define S_RxHighPriorityThreshold       8
#define M_RxHighPriorityThreshold       (0xF << S_RxHighPriorityThreshold)
#define M_RxFpTestMode                  (1 << 7)
#define S_RxBurstSize                   0
#define M_RxBurstSize                   (0x7F << S_RxBurstSize)

/* 0x00D4 Rx DescQueue1 Ctrl Register */
/* 0x00D8 Rx DescQueue2 Ctrl Register */

#define S_RxBufferLength                16
#define M_RxBufferLength                (0xFFFF << S_RxBufferLength)
#define M_RxDescEntries                 (1 << 14)
#define M_RxConsumerWe                  (1 << 7)
#define S_RxMinDescriptorsThreshold     0
#define M_RxMinDescriptorsThreshold     (0x7F << S_RxMinDescriptorsThreshold)

/* 0x00D4 Rx DescQueue1 Ctrl Register (only) */

#define M_RxPrefetchDescriptorsMode     (1 << 15)
#define M_RxVariableSizeQueues          (1 << 13)
#define M_Rx64BitBufferAddresses        (1 << 12)
#define M_Rx64BitDescQueueAddress       (1 << 11)
#define S_RxDescSpacing                 8
#define M_RxDescSpacing                 (0x7 << S_RxDescSpacing)

/* 0x00E0 Rx DescQueue1 Low Address Register */
/* 0x00E4 Rx DescQueue2 Low Address Register */

#define S_RxDescQueueLowAddress         0            /* bit-aligned */
#define M_RxDescQueueLowAddress         0xFFFFFF00

/* 0x00E8 Rx DescQueue1 Ptrs */
/* 0x00EC Rx DescQueue2 Ptrs */

#define S_RxDescConsumer                16
#define M_RxDescConsumer                (0x7FF << S_RxDescConsumer)
#define S_RxDescProducer                0
#define M_RxDescProducer                (0x7FF << S_RxDescProducer)

/* 0x00F4 Rx Address Filtering Ctrl Register */

#define S_PerfectAddressPriority        16
#define M_PerfectAddressPriority        (0xFFFF << S_PerfectAddressPriority)
#define S_MinVlanPriority               13
#define M_MinVlanPriority               (0x7 << S_MinVlanPriority)
#define M_PassMulticastExceptBroadcast  (1 << 12)
#define S_WakeupMode                    10
#define M_WakeupMode                    (3 << S_WakeupMode)
#define S_VlanMode                      8
#define M_VlanMode                      (3 << S_VlanMode)

#define S_PerfectFilteringMode          6
#define M_PerfectFilteringMode          (3 << S_PerfectFilteringMode)
#define K_PerfectFiltering_Off          0
#define K_PerfectFiltering_16           1
#define K_PerfectFiltering_16Inv        2
#define K_PerfectFiltering_Vlan         3

#define S_HashFilteringMode             4
#define M_HashFilteringMode             (3 << S_HashFilteringMode)
#define K_HashFiltering_Off             0
#define K_HashFiltering_VlanMcast       1
#define K_HashFiltering_AllMcast        2
#define K_HashFiltering_All             3

#define M_HashPriorityEnable            (1 << 3)
#define M_PassBroadcast                 (1 << 2)
#define M_PassMulticast                 (1 << 1)
#define M_PromiscuousMode               (1 << 0)


/* 0x5000 Mac Config1 Register */

#define M_SoftRst                       (1 << 15)
#define M_MIILoopBack                   (1 << 14)
#define S_TestMode                      12
#define M_TestMode                      (0x3 << S_TestMode)
#define M_TxFlowEn                      (1 << 11)
#define M_RxFlowEn                      (1 << 10)
#define M_PreambleDetectCount           (1 << 9)
#define M_PassAllRxPackets              (1 << 8)
#define M_PurePreamble                  (1 << 7)
#define M_LengthCheck                   (1 << 6)
#define M_NoBackoff                     (1 << 5)
#define M_DelayCRC                      (1 << 4)
#define M_TxHalfDuplexJam               (1 << 3)
#define M_PadEn                         (1 << 2)
#define M_FullDuplex                    (1 << 1)
#define M_HugeFrame                     (1 << 0)

/* 0x5004 MacConfig2 Register */

#define M_TxCRCerr                      (1 << 15)
#define M_TxIslCRCerr                   (1 << 14)
#define M_RxCRCerr                      (1 << 13)
#define M_RxIslCRCerr                   (1 << 12)
#define M_TXCF                          (1 << 11)
#define M_CtlSoftRst                    (1 << 10)
#define M_RxSoftRst                     (1 << 9)
#define M_TxSoftRst                     (1 << 8)
#define M_RxISLEn                       (1 << 7)
#define M_BackPressureNoBackOff         (1 << 6)
#define M_AutoVlanPad                   (1 << 5)
#define M_MandatoryVLANPad              (1 << 4)
#define M_TxISLAppen                    (1 << 3)
#define M_TxISLCrcEn                    M_TxISLAppen
#define M_TxISLEn                       (1 << 2)

/* 0x5008 BkToBkIPG Register */

#define S_IPGT                          0
#define M_IPGT                          (0x7F << S_IPGT)
#define K_IPGT_FDX                      0x15
#define K_IPGT_HDX                      0x11

/* 0x500C NonBkToBkIPG Register */

#define S_IPGR1                         8
#define M_IPGR1                         (0x7F << S_IPGR1)
#define S_IPGR2                         0
#define M_IPGR2                         (0x7F << M_IPGR2)

/* 0x5010 ColRetry Register */

#define S_LateColWin                    8
#define M_LateColWin                    (0x3F << S_LateColWin)
#define S_MaxRetry                      0
#define M_MaxRetry                      (0xF << S_MaxRetry)

/* 0x5014 MaxLength Register */

#define S_MaxPacketLength               0
#define M_MaxPacketLength               (0xFFFF << S_MaxPacketLength)

/* 0x5024 RandomNumGen Register */

#define S_RandomNumGen                  0
#define M_RandomNumGen                  (0x3FF << S_RandomNumGen)

/* 0x5028 MakRandomNum Register */

#define S_MskRandomNum                  0
#define M_MskRandomNum                  (0x3FF << S_MskRandomNum)

/* 0x5060 TxPauseTimer Register */

#define S_TxPauseTimer                  0
#define M_TxPauseTimer                  (0xFFFF << S_TxPauseTimer)

/* 0x5064 VLAN Type Register */

#define S_VLANType                      0
#define M_VLANType                      (0xFFFF << S_VLANType)

/* 0x5070 MIIStatus Register */

#define M_MIIStatus_MIILinkFail         (1 << 4)
#define M_MIIStatus_NotValid            (1 << 3)
#define M_MIIStatus_Scan                (1 << 2)
#define M_MIIStatus_MiiDataValid        (1 << 1)
#define M_MIIStatus_MiiBusy             (1 << 0)

#endif /* _AIC6915_H_ */
