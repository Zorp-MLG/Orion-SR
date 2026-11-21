// Orion mini-Arduino API for STM32G0B1 (bare metal, no vendor libraries).
// Provides: setup()/loop(), pinMode/digitalWrite/digitalRead, millis/micros/delay,
// and Serial over native USB (CDC virtual COM port).
#pragma once
#include <stdint.h>
#include <stddef.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

typedef uint8_t byte;

// Pin names: (port << 4) | pin.  PA11/PA12 are the USB pins - don't use them.
enum : uint8_t {
  PA0 = 0x00, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
  PB0 = 0x10, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
  PC0 = 0x20, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
  PD0 = 0x30, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
  PF0 = 0x50, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
};

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);

uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

// Jump to the STM32 ROM USB bootloader (DFU) without touching any buttons.
// Also triggered automatically when the host opens the port at 1200 baud.
void rebootToBootloader(void);

void setup(void);
void loop(void);

template <class T> static inline T min(T a, T b) { return a < b ? a : b; }
template <class T> static inline T max(T a, T b) { return a > b ? a : b; }
template <class T> static inline T constrain(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ---- USB CDC (implemented in orion_usb.cpp) ----
void usb_cdc_begin(void);
int usb_cdc_available(void);
int usb_cdc_read(void);
int usb_cdc_peek(void);
size_t usb_cdc_write(const uint8_t *buf, size_t len);
void usb_cdc_flush(void);
bool usb_cdc_connected(void);   // host has opened the port (DTR set)
bool usb_cdc_dfu_requested(void);
void usb_cdc_disconnect(void);

class USBSerial {
public:
  void begin(unsigned long baud = 115200) { (void)baud; usb_cdc_begin(); }
  void end() {}
  int available() { return usb_cdc_available(); }
  int read() { return usb_cdc_read(); }
  int peek() { return usb_cdc_peek(); }
  void flush() { usb_cdc_flush(); }
  explicit operator bool() { return usb_cdc_connected(); }

  size_t write(uint8_t c) { return usb_cdc_write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) { return usb_cdc_write(b, n); }
  size_t write(const char *s);

  size_t print(const char *s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(int v, int base = DEC);
  size_t print(unsigned v, int base = DEC);
  size_t print(long v, int base = DEC);
  size_t print(unsigned long v, int base = DEC);
  size_t print(double v, int digits = 2);

  size_t println() { return write((const uint8_t *)"\r\n", 2); }
  template <class T> size_t println(T v) { size_t n = print(v); return n + println(); }
  template <class T> size_t println(T v, int x) { size_t n = print(v, x); return n + println(); }

private:
  size_t printNum(unsigned long v, int base, bool neg);
};

extern USBSerial Serial;
