/* Vector table + reset handler for STM32G0B1 (Cortex-M0+). */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);

void Reset_Handler(void) {
  uint32_t *src = &_sidata, *dst = &_sdata;
  while (dst < &_edata) *dst++ = *src++;
  for (dst = &_sbss; dst < &_ebss;) *dst++ = 0;
  main();
  for (;;) { }
}

void Default_Handler(void) { for (;;) { } }

void SysTick_Handler(void);
void USB_IRQHandler(void);

__attribute__((section(".isr_vector"), used))
void (*const vector_table[])(void) = {
  (void (*)(void))&_estack,   /* 0  initial SP */
  Reset_Handler,              /* 1  */
  Default_Handler,            /* 2  NMI */
  Default_Handler,            /* 3  HardFault */
  0, 0, 0, 0, 0, 0, 0,        /* 4-10 reserved */
  Default_Handler,            /* 11 SVC */
  0, 0,                       /* 12-13 reserved */
  Default_Handler,            /* 14 PendSV */
  SysTick_Handler,            /* 15 SysTick */
  Default_Handler,            /* 16 WWDG */
  Default_Handler,            /* 17 PVD */
  Default_Handler,            /* 18 RTC/TAMP */
  Default_Handler,            /* 19 FLASH */
  Default_Handler,            /* 20 RCC/CRS */
  Default_Handler,            /* 21 EXTI0_1 */
  Default_Handler,            /* 22 EXTI2_3 */
  Default_Handler,            /* 23 EXTI4_15 */
  USB_IRQHandler,             /* 24 USB / UCPD1 / UCPD2 (IRQ 8) */
};
