#include "ethernet.h"

#include <cmsis_gcc.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <strings.h>

#include "stm32h743xx.h"
#include "stm32h7xx.h"

#include "tools.h"

ETH_DMA_DATA eth_txdesc_t eth_tx_ring[ETH_TX_RING_SZ];
ETH_DMA_DATA eth_rxdesc_t eth_rx_ring[ETH_RX_RING_SZ];

ETH_DMA_DATA char eth_tx_bufs[ETH_TX_RING_SZ][ETH_TX_BUF_SZ];
ETH_DMA_DATA char eth_tx_buf_extra[ETH_TX_BUF_SZ];
ETH_DMA_DATA char eth_rx_bufs[ETH_RX_RING_SZ][ETH_RX_BUF_SZ];
ETH_DMA_DATA char eth_rx_buf_extra[ETH_RX_BUF_SZ];
char *eth_rx_buf_table[ETH_RX_RING_SZ];

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
char *eth_next_tx_buf(void)
{
    return eth_dma_state_global.tx_tail->writeback.OWN == 0
        ? eth_tx_bufs[eth_dma_state_global.tx_tail - eth_tx_ring]
        : eth_tx_buf_extra;
}

// Minimal to no safety checks
// TODO: write a list of conditions when it's safe to call this
int eth_send(uint16_t len, char **next_buf)
{
    eth_txdesc_t *new_desc, *new_tail;

    if (len > ETH_TX_BUF_SZ) {
        return -1;
    }

    new_desc = eth_dma_state_global.tx_tail;
    while (new_desc->writeback.OWN == 1);

    new_tail = eth_dma_state_global.tx_tail < &eth_tx_ring[ETH_TX_RING_SZ - 1]
        ? eth_dma_state_global.tx_tail + 1
        : eth_tx_ring;
    eth_dma_state_global.tx_tail = new_tail;

    *new_desc = (eth_txdesc_t) { .raw = {0, 0, 0, 0} };
    new_desc->read.BUF1AP = (uint32_t)eth_dma_state_global.tx_curr_buf;
    new_desc->read.IOC = 1;
    new_desc->read.HL_B1L = len;
    new_desc->read.OWN = 1;
    new_desc->read.FD = 1;
    new_desc->read.LD = 1;

    __DSB();
    ETH->DMACTDTPR = (uint32_t)new_tail;

    // Get a buffer for the next eth_send() call
    eth_dma_state_global.tx_curr_buf = *next_buf = eth_next_tx_buf();
    return 0;
}

// sz (for dma) + 1 (for user) max buffers in use:
// - free from user's last call (if it was)
// - alloc a new one for dma
// - transfer from dma (last one used in this pos) to user
// before first call sz buffers are used by dma, one is free for user
// algo:
//   - last_out_buf = extra
//   eth_read:
//   - give to user from the dma_buf_table[i], set it to last_out_buf at the end
//   - free the one in last_out_buf
//   - set it for dma and in the dma_buf_table[i]
//
int eth_recv(char **out_buf) {
    eth_rxdesc_t *head = eth_dma_state_global.rx_head;
    while (head->writeback.OWN == 1);

    // Error summary is not zero
    if (head->writeback.ES == 1) {
        return -1;
    }
    int len = head->writeback.PL;

    char **buf_table_entry = &eth_rx_buf_table[head - eth_rx_ring];
    char *filled = *buf_table_entry;
    *buf_table_entry = eth_dma_state_global.rx_last_out_buf;

    head->read.BUF1AP = (uint32_t)eth_dma_state_global.rx_last_out_buf;
    head->read.BUF1V = 1;
    head->read.OWN = 1;

    eth_dma_state_global.rx_head = head < &eth_rx_ring[ETH_RX_RING_SZ - 1]
        ? head + 1
        : eth_rx_ring;
    *out_buf = eth_dma_state_global.rx_last_out_buf = filled;

    __DSB();
    ETH->DMACRDTPR = (uint32_t)&eth_rx_ring[ETH_RX_RING_SZ];
    return len;
}

void ETH_IRQHandler(void)
{
    if (ETH->DMAISR & ETH_DMAISR_DMACIS) {
        if (ETH->DMACSR & ETH_DMACSR_AIS) {
            __BKPT(0);
        }
    }
    return;
}

void setup_eth_dma(char **first_buf)
{
    memset(eth_tx_ring, 0, sizeof eth_tx_ring);
    memset(eth_rx_ring, 0, sizeof eth_rx_ring);
    eth_dma_state_global.tx_tail = eth_tx_ring;
    eth_dma_state_global.tx_curr_buf = *first_buf = eth_next_tx_buf();
    eth_dma_state_global.rx_head = eth_rx_ring;
    eth_dma_state_global.rx_last_out_buf = eth_rx_buf_extra;

    for (size_t i = 0; i < ETH_RX_RING_SZ; i++) {
        eth_rx_ring[i].read.BUF1AP = (uint32_t)eth_rx_bufs[i];
        eth_rx_ring[i].read.BUF1V = 1;
        eth_rx_ring[i].read.OWN = 1;
        eth_rx_buf_table[i] = eth_rx_bufs[i];
    }

    // Descriptor ring length (actually, the index of the last descriptor in
    // the ring, i.e. length - 1)
    ETH->DMACTDRLR = ETH_TX_RING_SZ - 1;
    ETH->DMACRDRLR = ETH_RX_RING_SZ - 1;

    // Descriptor list address (base)
    ETH->DMACTDLAR = (uint32_t)eth_tx_ring;
    ETH->DMACRDLAR = (uint32_t)eth_rx_ring;

    // Descriptor tail pointer
    ETH->DMACTDTPR = (uint32_t)eth_tx_ring;
    ETH->DMACRDTPR = (uint32_t)&eth_rx_ring[ETH_RX_RING_SZ];

    // Tx DMA transfers in bursts of 32 beats (beat = bus width = 4 bytes)
    MODIFY_REG(ETH->DMACTCR, ETH_DMACTCR_TPBL, ETH_DMACTCR_TPBL_32PBL);
    // Receive buffer size - 1514 (max size of DIX Ethernet II packets minus 4
    // stripped CRC32 bytes) rounded up to a multiple of 4
    MODIFY_REG(ETH->DMACRCR, ETH_DMACRCR_RBSZ, 1516);
    // Rx DMA transfers in bursts of 32 beats
    MODIFY_REG(ETH->DMACRCR, ETH_DMACRCR_RPBL, ETH_DMACRCR_RPBL_32PBL);

    // Enable some abnormal DMA interrupts (to crash the code explicitly in
    // case of incorrect behaviour)
    ETH->DMACIER = ETH_DMACIER_AIE |
        // Abnormal: Context descriptor error and Fatal bus error
        // What does Transmit stopped mean? (hasn't been seen firing yet)
        ETH_DMACIER_CDEE | ETH_DMACIER_FBEE | ETH_DMACIER_TXSE;

    NVIC_EnableIRQ(ETH_IRQn);
    ETH->DMACTCR |= ETH_DMACTCR_ST;
    ETH->DMACRCR |= ETH_DMACRCR_SR;
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

void eth_setup(char **first_buf)
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
    // Our MAC address for DA matching and insertion: e2:18:e1:2c:f9:79
    // (randomly generated, locally administered, unicast).
    ETH->MACA0HR = 0xF00079F9;
    ETH->MACA0LR = 0x2CE118E2;
    // MAC packet filtering: drop all broadcast and non-matching to our DA
    // unicast packets
    ETH->MACPFR = ETH_MACPFR_DBF;
    // MAC settings: full duplex, sense carrier before transmitting, strip CRC
    // from all packets, insert MAC address 0 as a SA in all packets
    ETH->MACCR |= ETH_MACCR_ECRSFD | ETH_MACCR_FES | ETH_MACCR_DM |
        ETH_MACCR_ACS | ETH_MACCR_CST | ETH_MACCR_SARC_INSADDR0;
    ETH->MACIER |= ETH_MACIER_PHYIE | ETH_MACIER_PMTIE | ETH_MACIER_LPIIE | ETH_MACIER_TSIE | ETH_MACIER_TXSTSIE | ETH_MACIER_RXSTSIE;

    setup_eth_dma(first_buf);

    // Enable MAC Tx and Rx
    ETH->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;
}

