/*****************************************************************************
 * Copyright (c) 2018 Rowley Associates Limited.                             *
 *                                                                           *
 * This file may be distributed under the terms of the License Agreement     *
 * provided with this software.                                              *
 *                                                                           *
 * THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND, INCLUDING THE   *
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. *
 ****************************************************************************/
/*****************************************************************************
 *                           Preprocessor Definitions
 *                           ------------------------
 *
 * STARTUP_FROM_RESET
 *
 *   If defined, the program will startup from power-on/reset. If not defined
 *   the program will just loop endlessly from power-on/reset.
 *
 *   This definition is not defined by default on this target because the
 *   debugger is unable to reset this target and maintain control of it over the
 *   JTAG interface. The advantage of doing this is that it allows the debugger
 *   to reset the CPU and run programs from a known reset CPU state on each run.
 *   It also acts as a safety net if you accidently download a program in FLASH
 *   that crashes and prevents the debugger from taking control over JTAG
 *   rendering the target unusable over JTAG. The obvious disadvantage of doing
 *   this is that your application will not startup without the debugger.
 *
 *   We advise that on this target you keep STARTUP_FROM_RESET undefined whilst
 *   you are developing and only define STARTUP_FROM_RESET when development is
 *   complete.
 *
 * __NO_SYSTEM_INIT
 *
 *   If defined, the SystemInit() function will NOT be called. By default SystemInit()
 *   is called after reset to enable the clocks and memories to be initialised 
 *   prior to any C startup initialisation.
 *
 * VECTORS_IN_RAM
 *
 *   If defined then the exception vectors are copied from Flash to RAM
 *
 * __NO_FPU
 *
 *   If defined do NOT turn on the FPU
 *
 * __NO_RUNFAST_MODE
 *
 *   If defined do NOT turn on flush-to-zero and default NaN modes
 *
 *****************************************************************************/

.macro ISR_HANDLER name=
  .section .vectors, "ax"
  .word \name
  .section .init, "ax"
  .thumb_func
  .weak \name
\name:
1: b 1b // endless loop 
.endm

.macro ISR_HANDLER2 name=
  .section .vectors, "ax"
  .word \name\()_IRQHandler
  .section .init, "ax"
  .thumb_func
  .weak \name\()_IRQHandler
\name\()_IRQHandler:
1: b \name\()_DriverIRQHandler
  .thumb_func
  .weak \name\()_DriverIRQHandler
\name\()_DriverIRQHandler:
1: b 1b
.endm

.macro ISR_RESERVED
  .section .vectors, "ax"
  .word 0
.endm

  .syntax unified
  .global reset_handler

  .section .vectors, "ax"
  .code 16 
  .global _vectors
_vectors:
   .long   __stack_end__
#ifdef STARTUP_FROM_RESET
  .word reset_handler
#else
  .word reset_wait
#endif // STARTUP_FROM_RESET 
ISR_HANDLER NMI_Handler
ISR_HANDLER HardFault_Handler
ISR_HANDLER MemManage_Handler
ISR_HANDLER BusFault_Handler
ISR_HANDLER UsageFault_Handler
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER SVC_Handler
ISR_HANDLER DebugMon_Handler
ISR_RESERVED
ISR_HANDLER PendSV_Handler
ISR_HANDLER SysTick_Handler
  // External Interrupts
ISR_HANDLER2 CTI0                                 // Cross Trigger Interface for CM4
ISR_HANDLER2 DMA0_0_4                             // DMA Channel 0, 4 Transfer Complete
ISR_HANDLER2 DMA0_1_5                             // DMA Channel 1, 5 Transfer Complete
ISR_HANDLER2 DMA0_2_6                             // DMA Channel 2, 6 Transfer Complete
ISR_HANDLER2 DMA0_3_7                             // DMA Channel 3, 7 Transfer Complete
ISR_HANDLER2 DMA0_8_12                            // DMA Channel 8, 12 Transfer Complete
ISR_HANDLER2 DMA0_9_13                            // DMA Channel 9, 13 Transfer Complete
ISR_HANDLER2 DMA0_10_14                           // DMA Channel 10, 14 Transfer Complete
ISR_HANDLER2 DMA0_11_15                           // DMA Channel 11, 15 Transfer Complete
ISR_HANDLER2 DMA0_16_20                           // DMA Channel 16, 20 Transfer Complete
ISR_HANDLER2 DMA0_17_21                           // DMA Channel 17, 21 Transfer Complete
ISR_HANDLER2 DMA0_18_22                           // DMA Channel 18, 22 Transfer Complete
ISR_HANDLER2 DMA0_19_23                           // DMA Channel 19, 23 Transfer Complete
ISR_HANDLER2 DMA0_24_28                           // DMA Channel 24, 28 Transfer Complete
ISR_HANDLER2 DMA0_25_29                           // DMA Channel 25, 29 Transfer Complete
ISR_HANDLER2 DMA0_26_30                           // DMA Channel 26, 30 Transfer Complete
ISR_HANDLER2 DMA0_27_31                           // DMA Channel 27, 31 Transfer Complete
ISR_HANDLER2 DMA0_Error                           // DMA Error Interrupt - All Channels
ISR_HANDLER2 MCM0                                 // MCM Interrupt
ISR_HANDLER2 EWM                                  // External Watchdog Monitor Interrupt
ISR_HANDLER2 LLWU0                                // Low Leakage Wake Up
ISR_HANDLER2 SIM                                  // System Integation Module
ISR_HANDLER2 MU_A                                 // Messaging Unit - Side A
ISR_HANDLER2 Reserved39                           // Secured JTAG Controller
ISR_HANDLER2 Software1                            // Software Interrupt
ISR_HANDLER2 Software2                            // Software Interrupt
ISR_HANDLER2 WDOG0                                // Watchdog Interrupt
ISR_HANDLER2 SCG0                                 // System Clock Generator for M4 domain
ISR_HANDLER2 QSPI                                 // Quad Serial Peripheral Interface
ISR_HANDLER2 LTC                                  // Low Power Trusted Cryptography
ISR_HANDLER2 XRDC                                 // Extended Domain Resource Controller
ISR_HANDLER2 SNVS                                 // Secure Non-Volatile Storage Consolidated Interrupt
ISR_HANDLER2 TRNG0                                // Random Number Generator
ISR_HANDLER2 LPIT0                                // Low Power Periodic Interrupt Timer
ISR_HANDLER2 PMC0                                 // Power Management  Control interrupts for M4 domain
ISR_HANDLER2 CMC0                                 // Core Mode Controller interrupts for M4 domain
ISR_HANDLER2 LPTMR0                               // Low Power Timer
ISR_HANDLER2 LPTMR1                               // Low Power Timer
ISR_HANDLER2 TPM0                                 // Timer PWM module
ISR_HANDLER2 TPM1                                 // Timer PWM module
ISR_HANDLER2 TPM2                                 // Timer PWM module
ISR_HANDLER2 TPM3                                 // Timer PWM module
ISR_HANDLER2 FLEXIO0                              // Flexible IO
ISR_HANDLER2 LPI2C0                               // Inter-integrated circuit 0
ISR_HANDLER2 LPI2C1                               // Inter-integrated circuit 1
ISR_HANDLER2 LPI2C2                               // Inter-integrated circuit 2
ISR_HANDLER2 LPI2C3                               // Inter-integrated circuit 3
ISR_HANDLER2 I2S0                                 // Serial Audio Interface 0
ISR_HANDLER2 I2S1                                 // Serial Audio Interface 1
ISR_HANDLER2 LPSPI0                               // Low Power Serial Peripheral Interface
ISR_HANDLER2 LPSPI1                               // Low Power Serial Peripheral Interface
ISR_HANDLER2 LPUART0                              // Low Power UART
ISR_HANDLER2 LPUART1                              // Low Power UART
ISR_HANDLER2 LPUART2                              // Low Power UART
ISR_HANDLER2 LPUART3                              // Low Power UART
ISR_HANDLER2 DPM                                  // Dynamic Process Monitor
ISR_HANDLER2 PCTLA                                // Port A pin interrupt
ISR_HANDLER2 PCTLB                                // Port B pin interrupt
ISR_HANDLER2 ADC0                                 // Analog to Digital Convertor
ISR_HANDLER2 ADC1                                 // Analog to Digital Convertor
ISR_HANDLER2 CMP0                                 // Comparator
ISR_HANDLER2 CMP1                                 // Comparator
ISR_HANDLER2 DAC0                                 // Digital to Analog Convertor
ISR_HANDLER2 DAC1                                 // Digital to Analog Convertor
ISR_HANDLER2 WDOG1                                // Watchdog Interrupt from A7 subsystem
ISR_HANDLER2 USB0                                 // USB 0 Interrupt from A7 subsystem
ISR_HANDLER2 USB1                                 // USB 1 Interrupt from A7 subsystem
ISR_RESERVED
ISR_HANDLER2 WDOG2                                // Watchdog Interrupt from A7 subsystem
ISR_HANDLER2 USBPHY                               // USB PHY (used in conjunction with USBOTG1)
ISR_HANDLER2 CMC1                                 // A7 resets
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER2 GPU3D                                // Graphics Processing Unit 3D
ISR_HANDLER2 GPU2D                                // Graphics Processing Unit 2D
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
.long 0xFFFFFFFF   //  Reserved for user TRIM value

  .section .vectors, "ax"
_vectors_end:

#ifdef VECTORS_IN_RAM
  .section .vectors_ram, "ax"
_vectors_ram:
  .space _vectors_end-_vectors, 0
#endif

  .section .init, "ax"
  .thumb_func

reset_handler:

#ifndef __NO_SYSTEM_INIT
  ldr r0, =__stack_end__
  mov sp, r0
  bl SystemInit
#endif

#ifdef VECTORS_IN_RAM
  ldr r0, =__vectors_load_start__
  ldr r1, =__vectors_load_end__
  ldr r2, =_vectors_ram
l0:
  cmp r0, r1
  beq l1
  ldr r3, [r0]
  str r3, [r2]
  adds r0, r0, #4
  adds r2, r2, #4
  b l0
l1:
#endif

#if !defined(__NO_FPU) && !defined(__SOFTFP__)
  // Enable CP11 and CP10 with CPACR |= (0xf<<20)
  movw r0, 0xED88
  movt r0, 0xE000
  ldr r1, [r0]
  orrs r1, r1, #(0xf << 20)
  str r1, [r0]
#ifndef __NO_RUNFAST_MODE
  isb
  dsb
  vmrs r0, fpscr
  orrs r0, r0, #(0x3 << 24) // FZ and DN
  vmsr fpscr, r0
  // clear the CONTROL.FPCA bit
  mov r0, #0
  msr control, r0 
  // FPDSCR similarly
  movw r1, 0xEF3C
  movt r1, 0xE000
  ldr r0, [r1]
  orrs r0, r0, #(0x3 << 24) // FZ and DN
  str r0, [r1]
#endif
#endif

  // Configure vector table offset register 
  ldr r0, =0xE000ED08
#ifdef VECTORS_IN_RAM
  ldr r1, =_vectors_ram
#else
  ldr r1, =_vectors
#endif
  str r1, [r0]

  b _start

#ifndef __NO_SYSTEM_INIT
  .thumb_func
  .weak SystemInit
SystemInit:
  bx lr
#endif

#ifndef STARTUP_FROM_RESET
  .thumb_func
reset_wait:
1: b 1b // endless loop 
#endif // STARTUP_FROM_RESET 
