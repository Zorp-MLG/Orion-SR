// Orion core: clock, SysTick timing, GPIO, Serial printing, main().
#include "orion.h"

#define REG32(a) (*(volatile uint32_t *)(a))

// RCC / FLASH / SYSCFG
#define RCC_CR      REG32(0x40021000u)
#define RCC_CFGR    REG32(0x40021008u)
#define RCC_PLLCFGR REG32(0x4002100Cu)
#define RCC_IOPENR  REG32(0x40021034u)
#define RCC_APBENR2 REG32(0x40021040u)
#define FLASH_ACR   REG32(0x40022000u)
#define SYSCFG_CFGR1 REG32(0x40010000u)

// SysTick / NVIC
#define SYST_CSR REG32(0xE000E010u)
#define SYST_RVR REG32(0xE000E014u)
#define SYST_CVR REG32(0xE000E018u)
#define NVIC_ICER REG32(0xE000E180u)
#define NVIC_ICPR REG32(0xE000E280u)

static uint32_t core_hz = 16000000u;      // HSI16 until the PLL is up
static volatile uint32_t ms_ticks;
static uint32_t reload;

extern "C" void SysTick_Handler(void) { ms_ticks++; }

// ---- tiny libc replacements (compiler may emit these) ----
extern "C" {
__attribute__((optimize("no-tree-loop-distribute-patterns")))
void *memcpy(void *d, const void *s, size_t n) {
  uint8_t *dd = (uint8_t *)d; const uint8_t *ss = (const uint8_t *)s;
  while (n--) *dd++ = *ss++;
  return d;
}
__attribute__((optimize("no-tree-loop-distribute-patterns")))
void *memset(void *d, int c, size_t n) {
  uint8_t *dd = (uint8_t *)d;
  while (n--) *dd++ = (uint8_t)c;
  return d;
}
}

// ---- clock: 64 MHz from HSI16 via PLL; falls back to 16 MHz if the PLL fails ----
static void clock_init(void) {
  FLASH_ACR = (FLASH_ACR & ~7u) | 2u | (1u << 8) | (1u << 9);   // 2 wait states, prefetch, icache
  while ((FLASH_ACR & 7u) != 2u) { }

  // HSI16 source, M=1, N=8 (VCO 128 MHz), R=2 -> 64 MHz
  RCC_PLLCFGR = 2u | (8u << 8) | (1u << 28) | (1u << 29);
  RCC_CR |= (1u << 24);                                         // PLLON
  uint32_t t = 100000;
  while (!(RCC_CR & (1u << 25)) && --t) { }
  if (!t) return;                                               // stay on HSI16

  RCC_CFGR = (RCC_CFGR & ~7u) | 2u;                             // SYSCLK = PLLRCLK
  t = 100000;
  while (((RCC_CFGR >> 3) & 7u) != 2u && --t) { }
  if (!t) { RCC_CFGR &= ~7u; return; }
  core_hz = 64000000u;
}

static void systick_init(void) {
  reload = core_hz / 1000u - 1u;
  SYST_RVR = reload;
  SYST_CVR = 0;
  SYST_CSR = 7u;                         // core clock, interrupt, enable
}

uint32_t millis(void) { return ms_ticks; }

uint32_t micros(void) {
  uint32_t ms, val;
  do { ms = ms_ticks; val = SYST_CVR; } while (ms != ms_ticks);
  return ms * 1000u + ((reload - val) / (core_hz / 1000000u));
}

void delay(uint32_t ms) {
  uint32_t start = ms_ticks;
  while ((ms_ticks - start) < ms) { }
}

void delayMicroseconds(uint32_t us) {
  uint32_t start = micros();
  while ((micros() - start) < us) { }
}

// ---- GPIO ----
static inline uint32_t gpio_base(uint8_t pin) { return 0x50000000u + ((uint32_t)(pin >> 4) * 0x400u); }

void pinMode(uint8_t pin, uint8_t mode) {
  uint32_t b = gpio_base(pin);
  uint32_t n = pin & 15u;
  RCC_IOPENR |= 1u << (pin >> 4);
  (void)RCC_IOPENR;
  uint32_t pupd = (mode == INPUT_PULLUP) ? 1u : (mode == INPUT_PULLDOWN ? 2u : 0u);
  REG32(b + 0x0C) = (REG32(b + 0x0C) & ~(3u << (n * 2))) | (pupd << (n * 2));
  REG32(b + 0x00) = (REG32(b + 0x00) & ~(3u << (n * 2))) | ((mode == OUTPUT ? 1u : 0u) << (n * 2));
}

void digitalWrite(uint8_t pin, uint8_t val) {
  uint32_t n = pin & 15u;
  REG32(gpio_base(pin) + 0x18) = val ? (1u << n) : (1u << (n + 16));
}

int digitalRead(uint8_t pin) {
  return (REG32(gpio_base(pin) + 0x10) >> (pin & 15u)) & 1u;
}

// ---- Serial printing helpers ----
USBSerial Serial;

size_t USBSerial::write(const char *s) {
  size_t n = 0;
  while (s[n]) n++;
  return write((const uint8_t *)s, n);
}

size_t USBSerial::printNum(unsigned long v, int base, bool neg) {
  char buf[34];
  int i = 0;
  if (base < 2 || base > 16) base = 10;
  if (v == 0) buf[i++] = '0';
  while (v) { unsigned d = v % (unsigned)base; buf[i++] = d < 10 ? '0' + d : 'A' + d - 10; v /= (unsigned)base; }
  if (neg) buf[i++] = '-';
  char out[34];
  for (int k = 0; k < i; k++) out[k] = buf[i - 1 - k];
  return write((const uint8_t *)out, (size_t)i);
}

size_t USBSerial::print(int v, int base) { return print((long)v, base); }
size_t USBSerial::print(unsigned v, int base) { return printNum(v, base, false); }
size_t USBSerial::print(unsigned long v, int base) { return printNum(v, base, false); }
size_t USBSerial::print(long v, int base) {
  if (v < 0 && base == DEC) return printNum((unsigned long)(-v), base, true);
  return printNum((unsigned long)v, base, false);
}

size_t USBSerial::print(double v, int digits) {
  size_t n = 0;
  if (v != v) return write("nan");
  if (v < 0) { n += write((uint8_t)'-'); v = -v; }
  if (v > 4294967040.0) return n + write("ovf");
  double r = 0.5;
  for (int i = 0; i < digits; i++) r /= 10.0;
  v += r;
  unsigned long ip = (unsigned long)v;
  double rem = v - (double)ip;
  n += printNum(ip, 10, false);
  if (digits > 0) {
    n += write((uint8_t)'.');
    while (digits-- > 0) {
      rem *= 10.0;
      int d = (int)rem;
      n += write((uint8_t)('0' + d));
      rem -= d;
    }
  }
  return n;
}

// ---- jump to ROM bootloader (USB DFU) ----
void rebootToBootloader(void) {
  __asm volatile("cpsid i");
  usb_cdc_disconnect();
  SYST_CSR = 0;
  NVIC_ICER = 0xFFFFFFFFu;
  NVIC_ICPR = 0xFFFFFFFFu;
  RCC_CFGR &= ~7u;                                   // back to HSI16
  while (((RCC_CFGR >> 3) & 7u) != 0u) { }
  RCC_CR &= ~(1u << 24);                             // PLL off
  RCC_APBENR2 |= 1u;                                 // SYSCFG clock
  SYSCFG_CFGR1 = (SYSCFG_CFGR1 & ~3u) | 1u;          // map system memory at 0x0
  uint32_t sp = *(volatile uint32_t *)0x1FFF0000u;
  uint32_t pc = *(volatile uint32_t *)0x1FFF0004u;
  __asm volatile("msr msp, %0\n\tbx %1" : : "r"(sp), "r"(pc));
  for (;;) { }
}

int main(void) {
  clock_init();
  systick_init();
  setup();
  for (;;) {
    loop();
    if (usb_cdc_dfu_requested()) { delay(50); rebootToBootloader(); }
  }
}
