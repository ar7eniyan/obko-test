#ifndef INCLUDE_ETHERNET_H
#define INCLUDE_ETHERNET_H

#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#include <cmsis_gcc.h>

#define ETH_DMA_DATA __attribute__((aligned (4), section (".eth_dma_data")))

// 6 bytes of DA, 2 bytes of EtherType, and 1500 bytes of payload maximum
#define ETH_TX_BUF_SZ 1508
#define ETH_TX_RING_SZ 4
// 2048 bytes of watchdog limit - 2 stripped bytes of FCS = 2046 bytes max.
// Rounded up to a multiple of 4: 2048 bytes.
#define ETH_RX_BUF_SZ 2048
#define ETH_RX_RING_SZ 4

typedef union {
    struct {

        uint32_t BUF1AP;

        uint32_t BUF2AP;

        uint32_t HL_B1L : 14;
        uint32_t VTIR : 2;
        uint32_t B2L : 14;
        uint32_t TTSE : 1;
        uint32_t IOC : 1;

        uint32_t RSVD_TPL : 16;
        uint32_t CIC_TPL : 2;
        uint32_t TSE : 1;
        uint32_t THL : 4;
        uint32_t SAIC : 3;
        uint32_t CPC : 2;
        uint32_t LD : 1;
        uint32_t FD : 1;
        uint32_t CTXT : 1;
        uint32_t OWN : 1;
    } read;

    struct {

        uint32_t TTSL;

        uint32_t TTSH;

        uint32_t RSVD_1;

        uint32_t IHE : 1;
        uint32_t DB : 1;
        uint32_t UF : 1;
        uint32_t ED : 1;
        uint32_t CC : 4;
        uint32_t EC : 1;
        uint32_t LC : 1;
        uint32_t NC : 1;
        uint32_t LoC : 1;
        uint32_t PCE : 1;
        uint32_t FF : 1;
        uint32_t JT : 1;
        uint32_t ES : 1;
        uint32_t RSVD_2 : 1;
        uint32_t TTSS : 1;
        uint32_t RSVD_3 : 10;
        uint32_t LD : 1;
        uint32_t FD : 1;
        uint32_t CTXT : 1;
        uint32_t OWN : 1;
    } writeback;

    struct {

        uint32_t TTSL;

        uint32_t TTSH;

        uint32_t MSS : 14;
        uint32_t RSVD_1 : 2;
        uint32_t IVT : 16;

        uint32_t VT : 16;
        uint32_t VLTV : 1;
        uint32_t IVLTV : 1;
        uint32_t IVTIR : 2;
        uint32_t RSVD_2 : 3;
        uint32_t CDE : 1;
        uint32_t RSVD_3 : 2;
        uint32_t TCMSSV : 1;
        uint32_t OSTC : 1;
        uint32_t RSVD_4 : 2;
        uint32_t CTXT : 1;
        uint32_t OWN : 1;
    } context;

    struct {
        uint32_t TDES0;
        uint32_t TDES1;
        uint32_t TDES2;
        uint32_t TDES3;
    } raw;
} eth_txdesc_t;

typedef union {
    struct {

        uint32_t BUF1AP;

        uint32_t RSVD_1;

        uint32_t BUF2AP;

        uint32_t RSVD_2 : 24;
        uint32_t BUF1V : 1;
        uint32_t BUF2V : 1;
        uint32_t RSVD_3 : 4;
        uint32_t IOC : 1;
        uint32_t OWN : 1;
    } read;

    struct {

        uint32_t OVT : 16;
        uint32_t IVT : 16;

        uint32_t PT : 3;
        uint32_t IPHE : 1;
        uint32_t IPV4 : 1;
        uint32_t IPV6 : 1;
        uint32_t IPCB : 1;
        uint32_t IPCE : 1;
        uint32_t PMT : 4;
        uint32_t PFT : 1;
        uint32_t PV : 1;
        uint32_t TSA : 1;
        uint32_t TD : 1;
        uint32_t OPC : 16;

        uint32_t RSVD_1 : 10;
        uint32_t ARPNR : 1;
        uint32_t RSVD_2 : 4;
        uint32_t VF : 1;
        uint32_t SAF : 1;
        uint32_t DAF : 1;
        uint32_t HF : 1;
        uint32_t MADRM : 8;
        uint32_t L3FM : 1;
        uint32_t L4FM : 1;
        uint32_t L3L4FM : 3;

        uint32_t PL : 15;
        uint32_t ES : 1;
        uint32_t LT : 3;
        uint32_t DE : 1;
        uint32_t RE : 1;
        uint32_t OE : 1;
        uint32_t RWT : 1;
        uint32_t GP : 1;
        uint32_t CE : 1;
        uint32_t RS0V : 1;
        uint32_t RS1V : 1;
        uint32_t RS2V : 1;
        uint32_t LD : 1;
        uint32_t FD : 1;
        uint32_t CTXT : 1;
        uint32_t OWN : 1;
    } writeback;

    struct {

        uint32_t RTSL;

        uint32_t RTSH;

        uint32_t RSVD_1;

        uint32_t RSVD_2 : 30;
        uint32_t CTXT : 1;
        uint32_t OWN : 1;
    } context;

    struct {
        uint32_t RDES0;
        uint32_t RDES1;
        uint32_t RDES2;
        uint32_t RDES3;
    } raw;
} eth_rxdesc_t;

static_assert(sizeof(eth_txdesc_t) == 16, "eth_txdesc size is not 16 bytes");
static_assert(sizeof(eth_rxdesc_t) == 16, "eth_rxdesc size is not 16 bytes");

// API usage workflow:
// 1. Initialize the ETH peripheral and get the first buffer to fill by calling
//    setup_ethernet(&buf), with `buf` being a char pointer.
// 2. to send a packet:
//      a. fill `buf` with your data (no more than ETH_TX_BUF_SZ bytes)
//      b. call eth_send(len, &buf) to enqueue a new frame of `len` bytes long
//         for transmission and to get the next buffer in `buf`
void eth_setup(char **first_tx_buf);
int eth_send(uint16_t len, char **next_buf);
int eth_recv(char **out_buf);

typedef struct {
    // Points to the descriptor one past the last one owned by DMA
    // (wrapped around to zero if goes out of the bounds)
    eth_txdesc_t *tx_tail;
    // Stores the most recently allocated buffer from eth_tx_bufs or
    // eth_tx_buf_extra to be sent to the DMA in the next eth_send() call.
    char *tx_curr_buf;
    eth_rxdesc_t *rx_head;
    char *rx_last_out_buf;
} eth_dma_state_t;

#endif  // #ifndef INCLUDE_ETHERNET_H

