#include "ethernet.h"

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "stm32h743xx.h"
#include "stm32h7xx.h"

#include "tools.h"


void setup_eth_dma(void)
{
    memset(eth_txdesc_global, 0, sizeof eth_txdesc_global);
    memset(eth_rxdesc_global, 0, sizeof eth_rxdesc_global);

    for (size_t i = 0; i < ETH_RX_RING_LENGTH; i++) {
        eth_rxdesc_global[i].read.OWN = 1;
    }

    ETH->DMACTDRLR = ETH_TX_RING_LENGTH;
    ETH->DMACRDRLR = ETH_RX_RING_LENGTH;

    ETH->DMACTDLAR = (uint32_t)eth_txdesc_global;
    ETH->DMACRDLAR = (uint32_t)eth_rxdesc_global;

    ETH->DMACTDTPR = (uint32_t)&eth_txdesc_global[ETH_TX_RING_LENGTH - 1];
    ETH->DMACRDTPR = (uint32_t)&eth_rxdesc_global[ETH_RX_RING_LENGTH - 1];

    // Tx DMA transfers in bursts of 32 beats
    MODIFY_REG(ETH->DMACTCR, ETH_DMACTCR_TPBL, ETH_DMACTCR_TPBL_32PBL);
    // Receive buffer size
    // Round up 1518 (max size of DIX Ethernet II packets) to a multiple of 4
    MODIFY_REG(ETH->DMACRCR, ETH_DMACRCR_RBSZ, 1520);
    // Rx DMA transfers in bursts of 32 beats
    MODIFY_REG(ETH->DMACRCR, ETH_DMACRCR_RPBL, ETH_DMACRCR_RPBL_32PBL);

    // ETH->DMACTCR |= ETH_DMACTCR_ST;
    // ETH->DMACRCR |= ETH_DMACRCR_SR;
};

void setup_eth_gpio(void)
{
    // RMII pin configuration
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);

    // Mode = 0b10 (Alternate Function)
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE1, 0b10 << GPIO_MODER_MODE1_Pos);
    // AF11
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL1, 0b1011 << GPIO_AFRL_AFSEL1_Pos);
    // Output speed = 0b11 (Very High)
    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED1, 0b11 << GPIO_OSPEEDR_OSPEED1_Pos);
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

    // MDIO clock is eth_hclk (AHB1 clock) / 102.
    // The recommended AHB1 clock range for this configuration is 150-250 MHz.
    ETH->MACMDIOAR = (0b0100 << ETH_MACMDIOAR_CR_Pos);

    // Start transmission on a full packet retrival (Store and Forward mode)
    // Tx queue size is (0b111 + 1) * 256 = 2048 bytes
    ETH->MTLTQOMR |= ETH_MTLTQOMR_TSF | (0b111 << 16);
    // Enable the Tx queue (write 0b10 to TXQEN)
    MODIFY_REG(ETH->MTLTQOMR, 0b11 << 2, 0b10);
    // Start reading on a full packet retrival (Store and Forward mode)
    // Rx queue size is (0b111 + 1) * 256 = 2048 bytes
    ETH->MTLRQOMR |= ETH_MTLRQOMR_RSF | (0b111 << 20);

    setup_eth_dma();

    // Configure our MAC address
    ETH->MACA0HR = 0xF0000123;
    ETH->MACA0LR = 0x456789ab;
    // Configure MAC settings
    ETH->MACCR |= ETH_MACCR_DM | ETH_MACCR_CST;
    // Enable MAC Tx and Rx
    // ETH->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;
}

eth_txdesc_t eth_txdesc_global[ETH_TX_RING_LENGTH];
eth_rxdesc_t eth_rxdesc_global[ETH_RX_RING_LENGTH];

