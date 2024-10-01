#include "ethernet.h"

#include <cmsis_gcc.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <strings.h>

#include "stm32h743xx.h"
#include "stm32h7xx.h"

#include "tools.h"

eth_txdesc_t eth_txdesc_global[ETH_TX_RING_LENGTH] ETH_DMA_DATA_ATTRS;
eth_txdesc_t * const eth_txdesc_global_end = &eth_txdesc_global[ETH_TX_RING_LENGTH];

eth_rxdesc_t eth_rxdesc_global[ETH_RX_RING_LENGTH] ETH_DMA_DATA_ATTRS;

char eth_tx_bufs[ETH_TX_BUF_LENGTH][ETH_TX_RING_LENGTH] ETH_DMA_DATA_ATTRS;
char eth_tx_buf_extra[ETH_TX_BUF_LENGTH] ETH_DMA_DATA_ATTRS;

eth_dma_state_t eth_dma_state_global;

// DMA Tx descriptor ring processing scheme (RD = read, WB = write-back)
//   head = ETH_DMACT(X)DLAR = desc list address register
//   curr = ETH_DMACCATXBR = current application tx descriptor register
//   tail = ETH_DMACT(X)DTPR = desc tail pointer register
//   start = eth_dma_state_global.tx_start
//   end = eth_dma_state_global.tx_end
// Invariants:
// - end equals to the index one past the latest enqueued descriptor.
// - start equals to the index of the earliest not freed yet descriptor.
//   (to free a descriptor means to read the status of its write-back form and
//   increment the start index)
//
// Regular operation:
// head -> [#0, WB, OWN = 0] - processed <- start
//         [#1, WB, OWN = 0] - processed
// curr -> [#2, RD, OWN = 1] - processing
//         [#3, RD, OWN = 1] - pending
// tail -> [#4, RD, OWN = 0] - free <- end
//         [#5, RD, OWN = 0] - free
//
// Two write-back descriptors being freed by the app:
// head -> [#0, RD, OWN = 0] - free
//         [#1, RD, OWN = 0] - free
// curr -> [#2, RD, OWN = 1] - processing <- start
//         [#3, RD, OWN = 1] - pending
// tail -> [#4, RD, OWN = 0] - free <- end
//         [#5, RD, OWN = 0] - free
//
// A single new packet being enqueued at the end:
// head -> [#0, RD, OWN = 0] - free
//         [#1, RD, OWN = 0] - free
// curr -> [#2, RD, OWN = 1] - processing <- start
//         [#3, RD, OWN = 1] - pending
//         [#4, RD, OWN = 0] - pending
// tail -> [#5, RD, OWN = 0] - free <- end
//

// TODO: add a safety mechanism
char *eth_get_first_buffer(void)
{
    return eth_tx_bufs[0];
}

// Minimal to no safety checks
// TODO: write a list of conditions when it's safe to call this
int eth_send(char *buf, uint16_t len, char **next_buf)
{
    eth_txdesc_t *free_desc, *new_end;

    if (len > ETH_TX_BUF_LENGTH) {
        return -1;
    }

    if (eth_dma_state_global.tx_end == eth_txdesc_global_end ||
        eth_dma_state_global.tx_end == eth_txdesc_global)
    {
        free_desc = eth_txdesc_global;
        new_end = &eth_txdesc_global[1];
    } else {
        free_desc = eth_dma_state_global.tx_end;
        new_end = eth_dma_state_global.tx_end + 1;
    }

    if (eth_dma_state_global.tx_end != eth_txdesc_global) {
        while (eth_dma_state_global.tx_start == free_desc ||
               ETH->DMACCATDR == (uint32_t)new_end);
    }

    *free_desc = (eth_txdesc_t) { .raw = {0, 0, 0, 0} };
    free_desc->read.BUF1AP = (uint32_t)buf;
    free_desc->read.IOC = 1;
    free_desc->read.HL_B1L = len;
    free_desc->read.OWN = 1;
    free_desc->read.FD = 1;
    free_desc->read.LD = 1;
    // Inlcude or insert source address (use MAC address register 0)
    free_desc->read.SAIC = 0b001;

    size_t next_buf_idx;
    if (eth_dma_state_global.tx_end != eth_txdesc_global_end) {
        // No wrapping around
        ETH->DMACTDTPR = (uint32_t)new_end;
        next_buf_idx = new_end - eth_txdesc_global;
    } else {
        next_buf_idx = 0;
    }
    eth_dma_state_global.tx_end = new_end;

    // Allocate a buffer for the next eth_send() call
    if (&eth_txdesc_global[next_buf_idx] != eth_dma_state_global.tx_start) {
        *next_buf = eth_tx_bufs[next_buf_idx];
    } else {
        *next_buf = eth_tx_buf_extra;
        eth_dma_state_global.extra_buffer_desc_idx = next_buf_idx;
    }
    return 0;
}

void ETH_IRQHandler(void)
{
    if (ETH->DMAISR & ETH_DMAISR_DMACIS) {
        if (ETH->DMACSR & ETH_DMACSR_AIS) {
            __BKPT(0);
        }
        if (ETH->DMACSR & ETH_DMACSR_NIS) {
            // The packet is transmitted and the corresponding descriptor is
            // updated with its write-back form
            if (ETH->DMACSR & ETH_DMACSR_TI) {
                // while (eth_dma_state_global.tx_start->writeback.OWN == 0)
                if (eth_dma_state_global.tx_start != eth_txdesc_global_end) {
                    eth_dma_state_global.tx_start += 1;
                } else {
                    eth_dma_state_global.tx_start = eth_txdesc_global_end;
                }
                ETH->DMACSR |= ETH_DMACSR_TI;
            }
            ETH->DMACSR |= ETH_DMACSR_NIS;
        }
    }
    return;
}

void setup_eth_dma(void)
{
    eth_dma_state_global.tx_start = eth_txdesc_global;
    eth_dma_state_global.tx_end = eth_txdesc_global;
    memset(eth_txdesc_global, 0, sizeof eth_txdesc_global);
    memset(eth_rxdesc_global, 0, sizeof eth_rxdesc_global);

    for (size_t i = 0; i < ETH_RX_RING_LENGTH; i++) {
        eth_rxdesc_global[i].read.OWN = 1;
    }

    // Descriptor ring length (actually, the index of the last descriptor in
    // the ring, i.e. length - 1)
    ETH->DMACTDRLR = ETH_TX_RING_LENGTH - 1;
    ETH->DMACRDRLR = ETH_RX_RING_LENGTH - 1;

    // Descriptor list address (base)
    ETH->DMACTDLAR = (uint32_t)eth_txdesc_global;
    ETH->DMACRDLAR = (uint32_t)eth_rxdesc_global;

    // Descriptor tail pointer
    ETH->DMACTDTPR = (uint32_t)eth_txdesc_global;
    ETH->DMACRDTPR = (uint32_t)&eth_rxdesc_global[ETH_RX_RING_LENGTH - 1];

    // Tx DMA transfers in bursts of 32 beats (beat = bus width = 4 bytes)
    MODIFY_REG(ETH->DMACTCR, ETH_DMACTCR_TPBL, ETH_DMACTCR_TPBL_32PBL);
    // Receive buffer size - 1514 (max size of DIX Ethernet II packets minus 4
    // stripped CRC32 bytes) rounded up to a multiple of 4
    MODIFY_REG(ETH->DMACRCR, ETH_DMACRCR_RBSZ, 1516);
    // Rx DMA transfers in bursts of 32 beats
    MODIFY_REG(ETH->DMACRCR, ETH_DMACRCR_RPBL, ETH_DMACRCR_RPBL_32PBL);

    // Enable DMA interrupts
    ETH->DMACIER |= ETH_DMACIER_NIE | ETH_DMACIER_AIE |
        // Abnormal, ETH_DMACIER_RSE disabled
        ETH_DMACIER_CDEE | ETH_DMACIER_FBEE | ETH_DMACIER_ETIE |
        ETH_DMACIER_RWTE | ETH_DMACIER_RBUE |
        ETH_DMACIER_TXSE |
        // Normal
        ETH_DMACIER_TBUE | ETH_DMACIER_TIE;

    NVIC_EnableIRQ(ETH_IRQn);
    ETH->DMACTCR |= ETH_DMACTCR_ST;
    // ETH->DMACRCR |= ETH_DMACRCR_SR;
};

void setup_eth_gpio(void)
{
    // RMII pin configuration:
    // I: REF_CLK (PA1), CRS_DV (PA7), RXD0 (PC4), RXD1 (PC5)
    // O: MDC (PC1), TX_EN (PB11), TXD0 (PB12), TXD1 (PB13)
    // I/O: MDIO (PA2)
    // All pins are using AF11.
    // MDC and MDIO are working on ~2.5 MHz (OSPEED = 0b00), TXD0 and TXD1
    // are working on 50 MHz (OSPEED = 0b10) and TX_EN is not high-speed
    SET_RCC_xxxxEN(RCC->AHB4ENR,
        RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN);

    // MODER = AF, AFR = AF11 for REF_CLK, MDIO and CRS_DV
    MODIFY_REG(GPIOA->MODER,
        GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE7,
        (0b10 << GPIO_MODER_MODE1_Pos) | (0b10 << GPIO_MODER_MODE2_Pos) |
        (0b10 << GPIO_MODER_MODE7_Pos));
    MODIFY_REG(GPIOA->AFR[0],
        GPIO_AFRL_AFSEL1 | GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL7,
        (0b1011 << GPIO_AFRL_AFSEL1_Pos) | (0b1011 << GPIO_AFRL_AFSEL2_Pos) |
        (0b1011 << GPIO_AFRL_AFSEL7_Pos));

    // MODER = AF, AFR = AF11 for TX_EN, TXD0 and TXD1
    MODIFY_REG(GPIOB->MODER,
        GPIO_MODER_MODE11 | GPIO_MODER_MODE12 | GPIO_MODER_MODE13,
        (0b10 << GPIO_MODER_MODE11_Pos) | (0b10 << GPIO_MODER_MODE12_Pos) |
        (0b10 << GPIO_MODER_MODE13_Pos));
    MODIFY_REG(GPIOB->AFR[1],
        GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12 | GPIO_AFRH_AFSEL13,
        (0b1011 << GPIO_AFRH_AFSEL11_Pos) | (0b1011 << GPIO_AFRH_AFSEL12_Pos) |
        (0b1011 << GPIO_AFRH_AFSEL13_Pos));
    // OSPEED = 0b10 for TXD0 and TXD1
    MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13,
        (0b10 << GPIO_OSPEEDR_OSPEED12_Pos) | (0b10 << GPIO_OSPEEDR_OSPEED13_Pos));

    // MODER = AF, AFR = AF11 for MDC, RXD0 and RXD1
    MODIFY_REG(GPIOC->MODER,
        GPIO_MODER_MODE1 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5,
        (0b10 << GPIO_MODER_MODE1_Pos) | (0b10 << GPIO_MODER_MODE4_Pos) |
        (0b10 << GPIO_MODER_MODE5_Pos));
    MODIFY_REG(GPIOC->AFR[0],
        GPIO_AFRL_AFSEL1 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5,
        (0b1011 << GPIO_AFRL_AFSEL1_Pos) | (0b1011 << GPIO_AFRL_AFSEL4_Pos) |
        (0b1011 << GPIO_AFRL_AFSEL5_Pos));
}

void setup_ethernet(void)
{
    setup_eth_gpio();

    // Enable SYSCFG block
    SET_RCC_xxxxEN(RCC->APB4ENR, RCC_APB4ENR_SYSCFGEN);
    // Set the Ethernet PHY type to RMII
    MODIFY_REG(SYSCFG->PMCR, SYSCFG_PMCR_EPIS_SEL, 0b100 << SYSCFG_PMCR_EPIS_SEL_Pos);

    SET_RCC_xxxxEN(RCC->AHB1ENR, RCC_AHB1ENR_ETH1MACEN | RCC_AHB1ENR_ETH1RXEN
        | RCC_AHB1ENR_ETH1TXEN);
    // Reset the DMA and MAC blocks and wait until thery're ready
    ETH->DMAMR |= ETH_DMAMR_SWR;
    while (ETH->DMAMR & ETH_DMAMR_SWR);
    // Use reset values for DMA AHB settings
    ETH->DMASBMR = 0;

    // Start reading on a full packet retrival (Store and Forward mode). That
    // means we aren't able to receive packets larger than the queue size.
    // Rx queue size is read-only and is equal to 2048 bytes.
    ETH->MTLRQOMR |= ETH_MTLRQOMR_RSF;
    // Start transmission on a full packet retrival (Store and Forward mode).
    // Tx queue size is (0b111 + 1) * 256 = 2048 bytes, and we can't transmit
    // packets larger than that because of TSF bit being set, but nor we have
    // to worry about MTL underflows. (analogous to the Rx queue situation
    // described above). Enable the Tx queue (write 0b10 to TXQEN).
    ETH->MTLTQOMR |= ETH_MTLTQOMR_TSF | (0b111 << 16) | (0b10 << 2);

    // MDIO clock is eth_hclk (AHB1 clock) / 102.
    // The recommended AHB1 clock range for this configuration is 150-250 MHz.
    ETH->MACMDIOAR = (0b0100 << ETH_MACMDIOAR_CR_Pos);
    // Our MAC address for DA matching
    ETH->MACA0HR = 0xF0001223;
    ETH->MACA0LR = 0x456789ab;
    // MAC packet filtering: drop all broadcast and non-matching to our DA
    // unicast packets
    ETH->MACPFR = ETH_MACPFR_DBF;
    // MAC settings: full duplex, sense carrier before transmitting, strip CRC
    // from all packets
    // TODO: deal with SARC
    ETH->MACCR |= ETH_MACCR_ECRSFD | ETH_MACCR_FES | ETH_MACCR_DM | ETH_MACCR_ACS | ETH_MACCR_CST;
    ETH->MACIER |= ETH_MACIER_PHYIE | ETH_MACIER_PMTIE | ETH_MACIER_LPIIE | ETH_MACIER_TSIE | ETH_MACIER_TXSTSIE | ETH_MACIER_RXSTSIE;

    setup_eth_dma();
    // Enable MAC Tx and Rx
    ETH->MACCR |= ETH_MACCR_TE;  // | ETH_MACCR_RE;
}

