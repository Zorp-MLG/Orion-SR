// Bare-metal USB CDC-ACM (virtual COM port) for the STM32G0B1 "USB" full-speed device peripheral.
// Interrupt driven, no vendor code. Clock: HSI48 trimmed by CRS from USB SOF (no crystal needed).
//
// Endpoints:  EP0 control | EP1 bulk IN/OUT (data) | EP2 interrupt IN (CDC notification, unused)
#include "orion.h"

#define REG32(a) (*(volatile uint32_t *)(a))

#define RCC_CRRCR   REG32(0x40021014u)
#define RCC_APBENR1 REG32(0x4002103Cu)
#define CRS_CR      REG32(0x40002000u)
#define NVIC_ISER   REG32(0xE000E100u)

#define USB_BASE   0x40005C00u
#define USB_EPR(n) REG32(USB_BASE + 4u * (n))
#define USB_CNTR   REG32(USB_BASE + 0x40u)
#define USB_ISTR   REG32(USB_BASE + 0x44u)
#define USB_DADDR  REG32(USB_BASE + 0x4Cu)
#define USB_BTABLE REG32(USB_BASE + 0x50u)
#define USB_BCDR   REG32(USB_BASE + 0x58u)

#define PMA ((volatile uint16_t *)0x40009800u)
#define BT_ADDR_TX(n)  PMA[(n) * 4 + 0]
#define BT_COUNT_TX(n) PMA[(n) * 4 + 1]
#define BT_ADDR_RX(n)  PMA[(n) * 4 + 2]
#define BT_COUNT_RX(n) PMA[(n) * 4 + 3]

#define EP0_TX 0x40
#define EP0_RX 0x80
#define EP1_TX 0xC0
#define EP1_RX 0x100
#define EP2_TX 0x140
#define RXCNT_64 0x8400u       // BL_SIZE=1, NUM_BLOCK=1 -> 64 bytes

#define ST_DIS 0
#define ST_STALL 1
#define ST_NAK 2
#define ST_VALID 3
#define T_BULK 0x0000u
#define T_CTRL 0x0200u
#define T_INT  0x0600u

#define IRQ_OFF() __asm volatile("cpsid i" ::: "memory")
#define IRQ_ON()  __asm volatile("cpsie i" ::: "memory")

// ---- descriptors ----
static const uint8_t dev_desc[18] = {
  18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, 64,
  0x09, 0x12, 0x01, 0x00,           // VID 0x1209 / PID 0x0001 (pid.codes test PID - hobby use only)
  0x00, 0x01, 1, 2, 0, 1
};

static const uint8_t cfg_desc[75] = {
  9, 2, 75, 0, 2, 1, 0, 0x80, 50,                    // configuration
  8, 0x0B, 0, 2, 0x02, 0x02, 0x00, 0,                // interface association
  9, 4, 0, 0, 1, 0x02, 0x02, 0x00, 0,                // comm interface (ACM)
  5, 0x24, 0x00, 0x10, 0x01,                         // header
  5, 0x24, 0x01, 0x00, 0x01,                         // call management
  4, 0x24, 0x02, 0x02,                               // ACM
  5, 0x24, 0x06, 0x00, 0x01,                         // union
  7, 5, 0x82, 0x03, 8, 0, 255,                       // EP2 IN interrupt
  9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,                // data interface
  7, 5, 0x01, 0x02, 64, 0, 0,                        // EP1 OUT bulk
  7, 5, 0x81, 0x02, 64, 0, 0                         // EP1 IN bulk
};

static uint8_t line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };   // 115200 8N1
static uint8_t sbuf[40];

static const uint8_t *string_desc(uint8_t idx, uint16_t *len) {
  if (idx == 0) { sbuf[0] = 4; sbuf[1] = 3; sbuf[2] = 0x09; sbuf[3] = 0x04; *len = 4; return sbuf; }
  const char *s = (idx == 1) ? "Orion" : (idx == 2) ? "Orion USB Serial" : 0;
  if (!s) return 0;
  uint8_t n = 2;
  while (*s && n < 38) { sbuf[n++] = (uint8_t)*s++; sbuf[n++] = 0; }
  sbuf[0] = n; sbuf[1] = 3;
  *len = n;
  return sbuf;
}

// ---- PMA helpers ----
static void pma_write(uint16_t off, const uint8_t *src, uint16_t len) {
  volatile uint16_t *p = PMA + (off >> 1);
  for (uint16_t i = 0; i < len; i += 2) {
    uint16_t w = src[i];
    if (i + 1 < len) w |= (uint16_t)src[i + 1] << 8;
    *p++ = w;
  }
}
static void pma_read(uint16_t off, uint8_t *dst, uint16_t len) {
  volatile uint16_t *p = PMA + (off >> 1);
  for (uint16_t i = 0; i < len; i += 2) {
    uint16_t w = *p++;
    dst[i] = (uint8_t)w;
    if (i + 1 < len) dst[i + 1] = (uint8_t)(w >> 8);
  }
}

// ---- endpoint register helpers (STAT/DTOG bits toggle on write-1) ----
static void ep_set(uint8_t n, int rx, int tx) {
  uint32_t cur = USB_EPR(n) & 0xFFFFu;
  uint32_t v = (cur & 0x0F0Fu) | 0x8080u;
  if (rx >= 0) v |= (cur ^ ((uint32_t)rx << 12)) & 0x3000u;
  if (tx >= 0) v |= (cur ^ ((uint32_t)tx << 4)) & 0x0030u;
  USB_EPR(n) = v;
}
static void ep_init(uint8_t n, uint32_t type, int rx, int tx) {
  uint32_t cur = USB_EPR(n) & 0xFFFFu;
  uint32_t v = type | n | 0x8080u;
  v |= (cur ^ ((uint32_t)rx << 12)) & 0x3000u;
  v |= (cur ^ ((uint32_t)tx << 4)) & 0x0030u;
  v |= cur & 0x4040u;                       // reset data toggles
  USB_EPR(n) = v;
}
static inline void ep_clr_rx(uint8_t n) { USB_EPR(n) = (USB_EPR(n) & 0x0F0Fu) | 0x0080u; }
static inline void ep_clr_tx(uint8_t n) { USB_EPR(n) = (USB_EPR(n) & 0x0F0Fu) | 0x8000u; }

// ---- state ----
static volatile uint8_t configured, dtr, tx_busy, rx_blocked, dfu_req, started;
static volatile uint8_t addr_pending, new_addr;

static uint8_t rx_buf[256], tx_buf[256];
static volatile uint16_t rx_head, rx_tail, tx_head, tx_tail;

enum { C_IDLE, C_IN, C_OUT_DATA };
static uint8_t c_state;
static const uint8_t *c_ptr;
static uint16_t c_rem;
static uint8_t c_sending, c_zlp, c_need_zlp;

static inline uint16_t rx_used() { return (rx_head - rx_tail) & 255u; }
static inline uint16_t rx_free() { return 255u - rx_used(); }

// ---- control endpoint ----
static void ep0_send(void) {
  uint16_t n = c_rem > 64 ? 64 : c_rem;
  pma_write(EP0_TX, c_ptr, n);
  BT_COUNT_TX(0) = n;
  c_ptr += n;
  c_rem -= n;
  c_zlp = (n == 64 && c_rem == 0 && c_need_zlp);
  ep_set(0, -1, ST_VALID);
}

static void ctrl_in(const uint8_t *p, uint16_t len, uint16_t wlen) {
  if (len > wlen) len = wlen;
  c_ptr = p;
  c_rem = len;
  c_need_zlp = (len < wlen) && (len % 64 == 0);
  c_state = C_IN;
  c_sending = 1;
  ep0_send();
  ep_set(0, ST_VALID, -1);                  // accept the status OUT
}

static void ctrl_status_in(void) {
  c_ptr = 0; c_rem = 0; c_need_zlp = 0;
  c_sending = 1;
  ep0_send();
}

static void handle_setup(const uint8_t *b) {
  uint8_t bm = b[0], req = b[1];
  uint16_t wv = b[2] | (b[3] << 8);
  uint16_t wl = b[6] | (b[7] << 8);
  static const uint8_t zero2[2] = { 0, 0 };
  static uint8_t cfgval;

  ep_set(0, ST_NAK, ST_NAK);                // also clears any earlier stall
  c_sending = 0;
  c_state = C_IDLE;

  if (bm & 0x80) {                          // device -> host
    if (bm == 0x80 && req == 6) {           // GET_DESCRIPTOR
      uint16_t len;
      const uint8_t *p = 0;
      switch (wv >> 8) {
        case 1: p = dev_desc; len = sizeof dev_desc; break;
        case 2: p = cfg_desc; len = sizeof cfg_desc; break;
        case 3: p = string_desc(wv & 0xFF, &len); break;
      }
      if (p) { ctrl_in(p, len, wl); return; }
    } else if (bm == 0x80 && req == 0) {    // GET_STATUS
      ctrl_in(zero2, 2, wl); return;
    } else if (bm == 0x80 && req == 8) {    // GET_CONFIGURATION
      cfgval = configured; ctrl_in(&cfgval, 1, wl); return;
    } else if (bm == 0xA1 && req == 0x21) { // GET_LINE_CODING
      ctrl_in(line_coding, 7, wl); return;
    }
  } else {                                  // host -> device
    if (bm == 0x00 && req == 5) {           // SET_ADDRESS (applied after status stage)
      new_addr = wv & 0x7F; addr_pending = 1; ctrl_status_in(); ep_set(0, ST_VALID, -1); return;
    } else if (bm == 0x00 && req == 9) {    // SET_CONFIGURATION
      configured = wv & 0xFF;
      if (configured) {
        ep_init(1, T_BULK, ST_VALID, ST_NAK);
        ep_init(2, T_INT, ST_DIS, ST_NAK);
        tx_busy = 0; rx_blocked = 0;
      } else {
        ep_init(1, T_BULK, ST_DIS, ST_DIS);
        ep_init(2, T_INT, ST_DIS, ST_DIS);
      }
      ctrl_status_in(); ep_set(0, ST_VALID, -1); return;
    } else if ((bm == 0x00 && (req == 1 || req == 3)) || (bm == 0x01 && req == 0x0B) ||
               (bm == 0x02 && (req == 1 || req == 3))) {
      ctrl_status_in(); ep_set(0, ST_VALID, -1); return;   // CLEAR/SET_FEATURE, SET_INTERFACE: accept
    } else if (bm == 0x21 && req == 0x20 && wl == 7) {     // SET_LINE_CODING (data follows)
      c_state = C_OUT_DATA; ep_set(0, ST_VALID, -1); return;
    } else if (bm == 0x21 && req == 0x22) {                // SET_CONTROL_LINE_STATE
      dtr = wv & 1; ctrl_status_in(); ep_set(0, ST_VALID, -1); return;
    }
  }
  ep_set(0, ST_VALID, ST_STALL);            // unsupported request
}

static void ep0_rx(bool setup) {
  if (setup) {
    uint8_t b[8];
    pma_read(EP0_RX, b, 8);
    handle_setup(b);
    return;
  }
  if (c_state == C_OUT_DATA) {
    uint16_t cnt = BT_COUNT_RX(0) & 0x3FFu;
    if (cnt >= 7) {
      pma_read(EP0_RX, line_coding, 7);
      uint32_t baud = line_coding[0] | (line_coding[1] << 8) | ((uint32_t)line_coding[2] << 16) | ((uint32_t)line_coding[3] << 24);
      if (baud == 1200) dfu_req = 1;
    }
    c_state = C_IDLE;
    ctrl_status_in();
  }
  ep_set(0, ST_VALID, -1);
}

static void ep0_tx(void) {
  if (!c_sending) return;
  if (c_rem || c_zlp) { ep0_send(); return; }
  c_sending = 0;
  if (addr_pending) { USB_DADDR = 0x80u | new_addr; addr_pending = 0; }
}

// ---- bulk data ----
static void tx_kick(void) {                 // call with IRQs disabled (or from the ISR)
  if (tx_busy || !configured) return;
  uint16_t used = (tx_head - tx_tail) & 255u;
  if (!used) return;
  uint16_t n = used > 64 ? 64 : used;
  uint8_t tmp[64];
  for (uint16_t i = 0; i < n; i++) { tmp[i] = tx_buf[tx_tail]; tx_tail = (tx_tail + 1) & 255u; }
  pma_write(EP1_TX, tmp, n);
  BT_COUNT_TX(1) = n;
  ep_set(1, -1, ST_VALID);
  tx_busy = 1;
}

static void ep1_rx(void) {
  uint16_t cnt = BT_COUNT_RX(1) & 0x3FFu;
  if (cnt > 64) cnt = 64;
  uint8_t tmp[64];
  pma_read(EP1_RX, tmp, cnt);
  for (uint16_t i = 0; i < cnt; i++) { rx_buf[rx_head] = tmp[i]; rx_head = (rx_head + 1) & 255u; }
  if (rx_free() >= 64) ep_set(1, ST_VALID, -1); else rx_blocked = 1;
}

static void usb_reset(void) {
  USB_BTABLE = 0;
  BT_ADDR_TX(0) = EP0_TX; BT_COUNT_TX(0) = 0; BT_ADDR_RX(0) = EP0_RX; BT_COUNT_RX(0) = RXCNT_64;
  BT_ADDR_TX(1) = EP1_TX; BT_COUNT_TX(1) = 0; BT_ADDR_RX(1) = EP1_RX; BT_COUNT_RX(1) = RXCNT_64;
  BT_ADDR_TX(2) = EP2_TX; BT_COUNT_TX(2) = 0; BT_ADDR_RX(2) = 0;      BT_COUNT_RX(2) = 0;
  ep_init(0, T_CTRL, ST_VALID, ST_NAK);
  USB_DADDR = 0x80u;
  configured = 0; dtr = 0; tx_busy = 0; rx_blocked = 0;
  c_state = C_IDLE; c_sending = 0; addr_pending = 0;
}

extern "C" void USB_IRQHandler(void) {
  uint32_t istr;
  while (((istr = USB_ISTR) & (0x8000u | 0x0400u)) != 0) {
    if (istr & 0x0400u) { USB_ISTR = (uint16_t)~0x0400u; usb_reset(); continue; }
    uint8_t n = istr & 0xFu;
    uint32_t epr = USB_EPR(n);
    if (epr & 0x8000u) {
      bool setup = (epr & 0x0800u) != 0;
      ep_clr_rx(n);
      if (n == 0) ep0_rx(setup);
      else if (n == 1) ep1_rx();
    }
    if (epr & 0x0080u) {
      ep_clr_tx(n);
      if (n == 0) ep0_tx();
      else if (n == 1) { tx_busy = 0; tx_kick(); }
    }
  }
}

// ---- public API ----
void usb_cdc_begin(void) {
  if (started) return;
  started = 1;
  RCC_CRRCR |= 1u;                                  // HSI48 on
  while (!(RCC_CRRCR & 2u)) { }
  RCC_APBENR1 |= (1u << 13) | (1u << 16);           // USB + CRS clocks
  CRS_CR = (1u << 6) | (1u << 5) | (0x20u << 8);    // auto-trim HSI48 from USB SOF
  USB_CNTR = 1u;                                    // FRES set, power-down cleared
  for (volatile int i = 0; i < 2000; i++) { }       // tSTARTUP
  USB_CNTR = 0;
  USB_ISTR = 0;
  USB_BCDR = 1u << 15;                              // DP pull-up: connect to host
  USB_CNTR = 0x8000u | 0x0400u;                     // CTRM | RESETM
  NVIC_ISER = 1u << 8;                              // USB_UCPD1_2 IRQ
}

int usb_cdc_available(void) { return rx_used(); }

int usb_cdc_peek(void) { return rx_used() ? rx_buf[rx_tail] : -1; }

int usb_cdc_read(void) {
  if (!rx_used()) return -1;
  int c = rx_buf[rx_tail];
  rx_tail = (rx_tail + 1) & 255u;
  if (rx_blocked && rx_free() >= 64) {
    IRQ_OFF();
    rx_blocked = 0;
    ep_set(1, ST_VALID, -1);
    IRQ_ON();
  }
  return c;
}

size_t usb_cdc_write(const uint8_t *b, size_t len) {
  if (!configured) return len;                      // no host: drop
  for (size_t i = 0; i < len; i++) {
    uint32_t t0 = millis();
    while (((tx_head + 1) & 255u) == tx_tail) {     // ring full: let the ISR drain it
      IRQ_OFF(); tx_kick(); IRQ_ON();
      if (!configured || (millis() - t0) > 20) return i;
    }
    tx_buf[tx_head] = b[i];
    tx_head = (tx_head + 1) & 255u;
  }
  IRQ_OFF(); tx_kick(); IRQ_ON();
  return len;
}

void usb_cdc_flush(void) {
  uint32_t t0 = millis();
  while ((tx_head != tx_tail || tx_busy) && configured && (millis() - t0) < 50) { }
}

void usb_cdc_disconnect(void) {                     // drop the pull-up and power the peripheral down
  if (started) { USB_BCDR = 0; USB_CNTR = 3u; }
}

bool usb_cdc_connected(void) { return configured && dtr; }
bool usb_cdc_dfu_requested(void) { return dfu_req; }
