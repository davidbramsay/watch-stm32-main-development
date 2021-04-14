/**
  ******************************************************************************
  * @file    stm32wb55xx.h
  * @author  MCD Application Team
  * @brief   CMSIS Cortex Device Peripheral Access Layer Header File.
  *          This file contains all the peripheral register's definitions, bits
  *          definitions and memory mapping for stm32wb55xx devices.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral's registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32wb55xx
  * @{
  */

#ifndef __STM32WB55xx_H
#define __STM32WB55xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */
/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
  */
#define __CM4_REV                 1U /*!< Core Revision r0p1                            */
#define __MPU_PRESENT             1U /*!< M4 provides an MPU                            */
#define __VTOR_PRESENT            1U /*!< Vector Table Register supported               */
#define __NVIC_PRIO_BITS          4U /*!< STM32WBxx uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1U /*!< FPU present                                   */
/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief stm32wb55xx Interrupt Number Definition, according to the selected device 
 *        in @ref Library_configuration_section
 */
/*!< Interrupt Number Definition for M4 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn                 = -14,    /*!< Non Maskable Interrupt                                            */
  HardFault_IRQn                      = -13,    /*!< Cortex-M4 Hard Fault Interrupt                                    */
  MemoryManagement_IRQn               = -12,    /*!< Cortex-M4 Memory Management Interrupt                             */
  BusFault_IRQn                       = -11,    /*!< Cortex-M4 Bus Fault Interrupt                                     */
  UsageFault_IRQn                     = -10,    /*!< Cortex-M4 Usage Fault Interrupt                                   */
  SVCall_IRQn                         = -5,     /*!< Cortex-M4 SV Call Interrupt                                       */
  DebugMonitor_IRQn                   = -4,     /*!< Cortex-M4 Debug Monitor Interrupt                                 */
  PendSV_IRQn                         = -2,     /*!< Cortex-M4 Pend SV Interrupt                                       */
  SysTick_IRQn                        = -1,     /*!< Cortex-M4 System Tick Interrupt                                   */

/*************  STM32WBxx specific Interrupt Numbers on M4 core ************************************************/
  WWDG_IRQn                           = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_PVM_IRQn                        = 1,      /*!< PVD and PVM detector                                              */
  TAMP_STAMP_LSECSS_IRQn              = 2,      /*!< RTC Tamper and TimeStamp Interrupts and LSECSS Interrupts         */
  RTC_WKUP_IRQn                       = 3,      /*!< RTC Wakeup Interrupt                                              */
  FLASH_IRQn                          = 4,      /*!< FLASH (CFI)  global Interrupt                                     */
  RCC_IRQn                            = 5,      /*!< RCC Interrupt                                                     */
  EXTI0_IRQn                          = 6,      /*!< EXTI Line 0 Interrupt                                             */
  EXTI1_IRQn                          = 7,      /*!< EXTI Line 1 Interrupt                                             */
  EXTI2_IRQn                          = 8,      /*!< EXTI Line 2 Interrupt                                             */
  EXTI3_IRQn                          = 9,      /*!< EXTI Line 3 Interrupt                                             */
  EXTI4_IRQn                          = 10,     /*!< EXTI Line 4 Interrupt                                             */
  DMA1_Channel1_IRQn                  = 11,     /*!< DMA1 Channel 1 Interrupt                                          */
  DMA1_Channel2_IRQn                  = 12,     /*!< DMA1 Channel 2 Interrupt                                          */
  DMA1_Channel3_IRQn                  = 13,     /*!< DMA1 Channel 3 Interrupt                                          */
  DMA1_Channel4_IRQn                  = 14,     /*!< DMA1 Channel 4 Interrupt                                          */
  DMA1_Channel5_IRQn                  = 15,     /*!< DMA1 Channel 5 Interrupt                                          */
  DMA1_Channel6_IRQn                  = 16,     /*!< DMA1 Channel 6 Interrupt                                          */
  DMA1_Channel7_IRQn                  = 17,     /*!< DMA1 Channel 7 Interrupt                                          */
  ADC1_IRQn                           = 18,     /*!< ADC1 Interrupt                                                    */
  USB_HP_IRQn                         = 19,     /*!< USB High Priority Interrupt                                       */
  USB_LP_IRQn                         = 20,     /*!< USB Low Priority Interrupt (including USB wakeup)                 */
  C2SEV_PWR_C2H_IRQn                  = 21,     /*!< CPU2 SEV Interrupt                                                */
  COMP_IRQn                           = 22,     /*!< COMP1 and COMP2 Interrupts                                        */
  EXTI9_5_IRQn                        = 23,     /*!< EXTI Lines [9:5] Interrupt                                        */
  TIM1_BRK_IRQn                       = 24,     /*!< TIM1 Break Interrupt                                              */
  TIM1_UP_TIM16_IRQn                  = 25,     /*!< TIM1 Update and TIM16 global Interrupts                           */
  TIM1_TRG_COM_TIM17_IRQn             = 26,     /*!< TIM1 Trigger and Communication and TIM17 global Interrupts        */
  TIM1_CC_IRQn                        = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                           = 28,     /*!< TIM2 Global Interrupt                                             */
  PKA_IRQn                            = 29,     /*!< PKA Interrupt                                                     */
  I2C1_EV_IRQn                        = 30,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                        = 31,     /*!< I2C1 Error Interrupt                                              */
  I2C3_EV_IRQn                        = 32,     /*!< I2C3 Event Interrupt                                              */
  I2C3_ER_IRQn                        = 33,     /*!< I2C3 Error Interrupt                                              */
  SPI1_IRQn                           = 34,     /*!< SPI1 Interrupt                                                    */
  SPI2_IRQn                           = 35,     /*!< SPI2 Interrupt                                                    */
  USART1_IRQn                         = 36,     /*!< USART1 Interrupt                                                  */
  LPUART1_IRQn                        = 37,     /*!< LPUART1 Interrupt                                                 */
  SAI1_IRQn                           = 38,     /*!< SAI1 A and B global interrupt                                     */
  TSC_IRQn                            = 39,     /*!< TSC Interrupt                                                     */
  EXTI15_10_IRQn                      = 40,     /*!< EXTI Lines1[15:10 ]Interrupts                                     */
  RTC_Alarm_IRQn                      = 41,     /*!< RTC Alarms (A and B) Interrupt                                    */
  CRS_IRQn                            = 42,     /*!< CRS interrupt                                                     */
  PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQn = 43,     /*!< PWR switching on the fly interrupt
                                                     PWR end of BLE activity interrupt
                                                     PWR end of 802.15.4 (Zigbee) activity interrupt
                                                     PWR end of critical radio phase interrupt                         */
  IPCC_C1_RX_IRQn                     = 44,     /*!< IPCC RX Occupied Interrupt                                        */
  IPCC_C1_TX_IRQn                     = 45,     /*!< IPCC TX Free Interrupt                                            */
  HSEM_IRQn                           = 46,     /*!< HSEM Interrupt                                                    */
  LPTIM1_IRQn                         = 47,     /*!< LPTIM1 Interrupt                                                  */
  LPTIM2_IRQn                         = 48,     /*!< LPTIM2 Interrupt                                                  */
  LCD_IRQn                            = 49,     /*!< LCD Interrupt                                                     */
  QUADSPI_IRQn                        = 50,     /*!< QUADSPI Interrupt                                                 */
  AES1_IRQn                           = 51,     /*!< AES1 Interrupt                                                    */
  AES2_IRQn                           = 52,     /*!< AES2 Interrupt                                                    */
  RNG_IRQn                            = 53,     /*!< RNG Interrupt                                                     */
  FPU_IRQn                            = 54,     /*!< FPU Interrupt                                                     */
  DMA2_Channel1_IRQn                  = 55,     /*!< DMA2 Channel 1 Interrupt                                          */
  DMA2_Channel2_IRQn                  = 56,     /*!< DMA2 Channel 2 Interrupt                                          */
  DMA2_Channel3_IRQn                  = 57,     /*!< DMA2 Channel 3 Interrupt                                          */
  DMA2_Channel4_IRQn                  = 58,     /*!< DMA2 Channel 4 Interrupt                                          */
  DMA2_Channel5_IRQn                  = 59,     /*!< DMA2 Channel 5 Interrupt                                          */
  DMA2_Channel6_IRQn                  = 60,     /*!< DMA2 Channel 6 Interrupt                                          */
  DMA2_Channel7_IRQn                  = 61,     /*!< DMA2 Channel 7 Interrupt                                          */
  DMAMUX1_OVR_IRQn                    = 62      /*!< DMAMUX1 overrun Interrupt                                         */
} IRQn_Type;
/**
  * @}
  */

#include "core_cm4.h"                /* Cortex-M4 processor and core peripherals */
#include "system_stm32wbxx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */
typedef struct
{
  __IO uint32_t ISR;          /*!< ADC interrupt and status register,             Address offset: 0x00 */
  __IO uint32_t IER;          /*!< ADC interrupt enable register,                 Address offset: 0x04 */
  __IO uint32_t CR;           /*!< ADC control register,                          Address offset: 0x08 */
  __IO uint32_t CFGR;         /*!< ADC configuration register 1,                  Address offset: 0x0C */
  __IO uint32_t CFGR2;        /*!< ADC configuration register 2,                  Address offset: 0x10 */
  __IO uint32_t SMPR1;        /*!< ADC sampling time register 1,                  Address offset: 0x14 */
  __IO uint32_t SMPR2;        /*!< ADC sampling time register 2,                  Address offset: 0x18 */
       uint32_t RESERVED1;    /*!< Reserved,                                                      0x1C */
  __IO uint32_t TR1;          /*!< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
  __IO uint32_t TR2;          /*!< ADC analog watchdog 2 threshold register,      Address offset: 0x24 */
  __IO uint32_t TR3;          /*!< ADC analog watchdog 3 threshold register,      Address offset: 0x28 */
       uint32_t RESERVED2;    /*!< Reserved,                                                      0x2C */
  __IO uint32_t SQR1;         /*!< ADC group regular sequencer register 1,        Address offset: 0x30 */
  __IO uint32_t SQR2;         /*!< ADC group regular sequencer register 2,        Address offset: 0x34 */
  __IO uint32_t SQR3;         /*!< ADC group regular sequencer register 3,        Address offset: 0x38 */
  __IO uint32_t SQR4;         /*!< ADC group regular sequencer register 4,        Address offset: 0x3C */
  __IO uint32_t DR;           /*!< ADC group regular data register,               Address offset: 0x40 */
       uint32_t RESERVED3;    /*!< Reserved,                                                      0x44 */
       uint32_t RESERVED4;    /*!< Reserved,                                                      0x48 */
  __IO uint32_t JSQR;         /*!< ADC group injected sequencer register,         Address offset: 0x4C */
       uint32_t RESERVED5[4]; /*!< Reserved,                                               0x50 - 0x5C */
  __IO uint32_t OFR1;         /*!< ADC offset register 1,                         Address offset: 0x60 */
  __IO uint32_t OFR2;         /*!< ADC offset register 2,                         Address offset: 0x64 */
  __IO uint32_t OFR3;         /*!< ADC offset register 3,                         Address offset: 0x68 */
  __IO uint32_t OFR4;         /*!< ADC offset register 4,                         Address offset: 0x6C */
       uint32_t RESERVED6[4]; /*!< Reserved,                                               0x70 - 0x7C */
  __IO uint32_t JDR1;         /*!< ADC group injected rank 1 data register,       Address offset: 0x80 */
  __IO uint32_t JDR2;         /*!< ADC group injected rank 2 data register,       Address offset: 0x84 */
  __IO uint32_t JDR3;         /*!< ADC group injected rank 3 data register,       Address offset: 0x88 */
  __IO uint32_t JDR4;         /*!< ADC group injected rank 4 data register,       Address offset: 0x8C */
       uint32_t RESERVED7[4]; /*!< Reserved,                                             0x090 - 0x09C */
  __IO uint32_t AWD2CR;       /*!< ADC analog watchdog 1 configuration register,  Address offset: 0xA0 */
  __IO uint32_t AWD3CR;       /*!< ADC analog watchdog 3 Configuration Register,  Address offset: 0xA4 */
       uint32_t RESERVED8;    /*!< Reserved,                                                     0x0A8 */
       uint32_t RESERVED9;    /*!< Reserved,                                                     0x0AC */
  __IO uint32_t DIFSEL;       /*!< ADC differential mode selection register,      Address offset: 0xB0 */
  __IO uint32_t CALFACT;      /*!< ADC calibration factors,                       Address offset: 0xB4 */

} ADC_TypeDef;

typedef struct
{
  uint32_t      RESERVED1;    /*!< Reserved,                                      Address offset: ADC1 base address + 0x300 */
  uint32_t      RESERVED2;    /*!< Reserved,                                      Address offset: ADC1 base address + 0x304 */
  __IO uint32_t CCR;          /*!< ADC common configuration register,             Address offset: ADC1 base address + 0x308 */
  uint32_t      RESERVED3;    /*!< Reserved,                                      Address offset: ADC1 base address + 0x30C */
} ADC_Common_TypeDef;

/**
  * @brief Comparator
  */
typedef struct
{
  __IO uint32_t CSR;         /*!< COMP control and status register,               Address offset: 0x00 */
} COMP_TypeDef;

typedef struct
{
  __IO uint32_t CSR;         /*!< COMP control and status register, used for bits common to several COMP instances, Address offset: 0x00 */
} COMP_Common_TypeDef;

/**
  * @brief CRC calculation unit
  */
typedef struct
{
  __IO uint32_t DR;          /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint32_t IDR;         /*!< CRC Independent data register,               Address offset: 0x04 */
  __IO uint32_t CR;          /*!< CRC Control register,                        Address offset: 0x08 */
       uint32_t RESERVED2;   /*!< Reserved,                                                    0x0C */
  __IO uint32_t INIT;        /*!< Initial CRC value register,                  Address offset: 0x10 */
  __IO uint32_t POL;         /*!< CRC polynomial register,                     Address offset: 0x14 */
} CRC_TypeDef;

/**
  * @brief Debug MCU
  */
typedef struct
{
  __IO uint32_t IDCODE;      /*!< MCU device ID code,                          Address offset: 0x00 */
  __IO uint32_t CR;          /*!< Debug MCU configuration register,            Address offset: 0x04 */
  uint32_t RESERVED1[13];    /*!< Reserved,                                               0x08-0x38 */
  __IO uint32_t APB1FZR1;    /*!< Debug MCU CPU1 APB1 freeze register,         Address offset: 0x3C */
  __IO uint32_t C2APB1FZR1;  /*!< Debug MCU CPU2 APB1 freeze register,         Address offset: 0x40 */
  __IO uint32_t APB1FZR2;    /*!< Debug MCU CPU1 APB1 freeze register,         Address offset: 0x44 */
  __IO uint32_t C2APB1FZR2;  /*!< Debug MCU CPU2 APB1 freeze register,         Address offset: 0x48 */
  __IO uint32_t APB2FZR;     /*!< Debug MCU CPU1 APB2 freeze register,         Address offset: 0x4C */
  __IO uint32_t C2APB2FZR;   /*!< Debug MCU CPU2 APB2 freeze register,         Address offset: 0x50 */
} DBGMCU_TypeDef;

/**
  * @brief DMA Controller
  */
typedef struct
{
  __IO uint32_t CCR;         /*!< DMA channel x configuration register        0x00 */
  __IO uint32_t CNDTR;       /*!< DMA channel x number of data register       0x04 */
  __IO uint32_t CPAR;        /*!< DMA channel x peripheral address register   0x08 */
  __IO uint32_t CMAR;        /*!< DMA channel x memory address register       0x0C */
  uint32_t RESERVED;         /*!< Reserved,                                   0x10 */
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;         /*!< DMA interrupt status register,                 Address offset: 0x00 */
  __IO uint32_t IFCR;        /*!< DMA interrupt flag clear register,             Address offset: 0x04 */
} DMA_TypeDef;

/**
  * @brief DMA Multiplexer
  */
typedef struct
{
  __IO uint32_t   CCR;       /*!< DMA Multiplexer Channel x Control Register    Address offset: 0x0004 * (channel x) */
}DMAMUX_Channel_TypeDef;

typedef struct
{
  __IO uint32_t   CSR;       /*!< DMA Channel Status Register                    Address offset: 0x0080   */
  __IO uint32_t   CFR;       /*!< DMA Channel Clear Flag Register                Address offset: 0x0084   */
}DMAMUX_ChannelStatus_TypeDef;

typedef struct
{
  __IO uint32_t   RGCR;        /*!< DMA Request Generator x Control Register     Address offset: 0x0100 + 0x0004 * (Req Gen x) */
}DMAMUX_RequestGen_TypeDef;

typedef struct
{
  __IO uint32_t   RGSR;        /*!< DMA Request Generator Status Register        Address offset: 0x0140   */
  __IO uint32_t   RGCFR;       /*!< DMA Request Generator Clear Flag Register    Address offset: 0x0144   */
}DMAMUX_RequestGenStatus_TypeDef;

/**
  * @brief FLASH Registers 
  */
typedef struct
{
  __IO uint32_t ACR;           /*!< FLASH Access control register,                      Address offset: 0x00      */
  __IO uint32_t RESERVED;      /*!< Reserved,                                           Address offset: 0x04      */
  __IO uint32_t KEYR;          /*!< FLASH Key register,                                 Address offset: 0x08      */
  __IO uint32_t OPTKEYR;       /*!< FLASH Option Key register,                          Address offset: 0x0C      */
  __IO uint32_t SR;            /*!< FLASH Status register,                              Address offset: 0x10      */
  __IO uint32_t CR;            /*!< FLASH Control register,                             Address offset: 0x14      */
  __IO uint32_t ECCR;          /*!< FLASH ECC register,                                 Address offset: 0x18      */
  uint32_t RESERVED1;          /*!< Reserved,                                           Address offset: 0x1C      */
  __IO uint32_t OPTR;          /*!< FLASH Option register,                              Address offset: 0x20      */
  __IO uint32_t PCROP1ASR;     /*!< FLASH Bank 1 PCROP area A Start address register,   Address offset: 0x24      */
  __IO uint32_t PCROP1AER;     /*!< FLASH Bank 1 PCROP area A End address register,     Address offset: 0x28      */
  __IO uint32_t WRP1AR;        /*!< FLASH Bank 1 WRP area A address register,           Address offset: 0x2C      */
  __IO uint32_t WRP1BR;        /*!< FLASH Bank 1 WRP area B address register,           Address offset: 0x30      */
  __IO uint32_t PCROP1BSR;     /*!< FLASH Bank 1 PCROP area B Start address register,   Address offset: 0x34      */
  __IO uint32_t PCROP1BER;     /*!< FLASH Bank 1 PCROP area B End address register,     Address offset: 0x38      */
  __IO uint32_t IPCCBR;        /*!< FLASH IPCC data buffer address,                     Address offset: 0x3C      */
  uint32_t RESERVED2[7];       /*!< Reserved,                                           Address offset: 0x40-0x58 */
  __IO uint32_t C2ACR;         /*!< FLASH Core MO+ Access Control Register ,            Address offset: 0x5C      */
  __IO uint32_t C2SR;          /*!< FLASH Core MO+ Status Register,                     Address offset: 0x60      */
  __IO uint32_t C2CR;          /*!< FLASH Core MO+ Control register,                    Address offset: 0x64      */
  uint32_t RESERVED3[6];       /*!< Reserved,                                           Address offset: 0x68-0x7C */
  __IO uint32_t SFR;           /*!< FLASH secure start address,                         Address offset: 0x80      */
  __IO uint32_t SRRVR;         /*!< FlASH secure SRAM2 start addr and CPU2 reset vector Address offset: 0x84      */
} FLASH_TypeDef;

/**
  * @brief General Purpose I/O
  */
typedef struct
{
  __IO uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
  __IO uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __IO uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */
} GPIO_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< I2C Control register 1,            Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< I2C Control register 2,            Address offset: 0x04 */
  __IO uint32_t OAR1;        /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;        /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;     /*!< I2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR;    /*!< I2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;         /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;         /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;        /*!< I2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;        /*!< I2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;        /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */
typedef struct
{
  __IO uint32_t KR;          /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;          /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;         /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;          /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR;        /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;

/**
  * @brief LPTIMER
  */
typedef struct
{
  __IO uint32_t ISR;         /*!< LPTIM Interrupt and Status register,                Address offset: 0x00 */
  __IO uint32_t ICR;         /*!< LPTIM Interrupt Clear register,                     Address offset: 0x04 */
  __IO uint32_t IER;         /*!< LPTIM Interrupt Enable register,                    Address offset: 0x08 */
  __IO uint32_t CFGR;        /*!< LPTIM Configuration register,                       Address offset: 0x0C */
  __IO uint32_t CR;          /*!< LPTIM Control register,                             Address offset: 0x10 */
  __IO uint32_t CMP;         /*!< LPTIM Compare register,                             Address offset: 0x14 */
  __IO uint32_t ARR;         /*!< LPTIM Autoreload register,                          Address offset: 0x18 */
  __IO uint32_t CNT;         /*!< LPTIM Counter register,                             Address offset: 0x1C */
  __IO uint32_t OR;          /*!< LPTIM Option register,                              Address offset: 0x20 */
} LPTIM_TypeDef;

/**
  * @brief Power Control
  */
typedef struct
{
  __IO uint32_t CR1;          /*!< PWR Power Control Register 1,                     Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< PWR Power Control Register 2,                     Address offset: 0x04 */
  __IO uint32_t CR3;          /*!< PWR Power Control Register 3,                     Address offset: 0x08 */
  __IO uint32_t CR4;          /*!< PWR Power Control Register 4,                     Address offset: 0x0C */
  __IO uint32_t SR1;          /*!< PWR Power Status Register 1,                      Address offset: 0x10 */
  __IO uint32_t SR2;          /*!< PWR Power Status Register 2,                      Address offset: 0x14 */
  __IO uint32_t SCR;          /*!< PWR Power Status Reset Register,                  Address offset: 0x18 */
  __IO uint32_t CR5;          /*!< PWR Power Control Register 5,                     Address offset: 0x1C */
  __IO uint32_t PUCRA;        /*!< PWR Pull-Up Control Register of port A,           Address offset: 0x20 */
  __IO uint32_t PDCRA;        /*!< PWR Pull-Down Control Register of port A,         Address offset: 0x24 */
  __IO uint32_t PUCRB;        /*!< PWR Pull-Up Control Register of port B,           Address offset: 0x28 */
  __IO uint32_t PDCRB;        /*!< PWR Pull-Down Control Register of port B,         Address offset: 0x2C */
  __IO uint32_t PUCRC;        /*!< PWR Pull-Up Control Register of port C,           Address offset: 0x30 */
  __IO uint32_t PDCRC;        /*!< PWR Pull-Down Control Register of port C,         Address offset: 0x34 */
  __IO uint32_t PUCRD;        /*!< PWR Pull-Up Control Register of port D,           Address offset: 0x38 */
  __IO uint32_t PDCRD;        /*!< PWR Pull-Down Control Register of port D,         Address offset: 0x3C */
  __IO uint32_t PUCRE;        /*!< PWR Pull-Up Control Register of port E,           Address offset: 0x40 */
  __IO uint32_t PDCRE;        /*!< PWR Pull-Down Control Register of port E,         Address offset: 0x44 */
       uint32_t RESERVED0[4]; /*!< Reserved,                                         Address offset: 0x48-0x54 */
  __IO uint32_t PUCRH;        /*!< PWR Pull-Up Control Register of port H,           Address offset: 0x58 */
  __IO uint32_t PDCRH;        /*!< PWR Pull-Down Control Register of port H,         Address offset: 0x5C */
       uint32_t RESERVED1[8]; /*!< Reserved,                                         Address offset: 0x60-0x7C */
  __IO uint32_t C2CR1;        /*!< PWR Power Control Register 1 for CPU2,            Address offset: 0x80 */
  __IO uint32_t C2CR3;        /*!< PWR Power Control Register 3 for CPU2,            Address offset: 0x84 */
  __IO uint32_t EXTSCR;       /*!< PWR Power Status Reset Register for CPU2,         Address offset: 0x88 */
} PWR_TypeDef;

/**
  * @brief QUAD Serial Peripheral Interface
  */
typedef struct
{
  __IO uint32_t CR;          /*!< QUADSPI Control register,                           Address offset: 0x00 */
  __IO uint32_t DCR;         /*!< QUADSPI Device Configuration register,              Address offset: 0x04 */
  __IO uint32_t SR;          /*!< QUADSPI Status register,                            Address offset: 0x08 */
  __IO uint32_t FCR;         /*!< QUADSPI Flag Clear register,                        Address offset: 0x0C */
  __IO uint32_t DLR;         /*!< QUADSPI Data Length register,                       Address offset: 0x10 */
  __IO uint32_t CCR;         /*!< QUADSPI Communication Configuration register,       Address offset: 0x14 */
  __IO uint32_t AR;          /*!< QUADSPI Address register,                           Address offset: 0x18 */
  __IO uint32_t ABR;         /*!< QUADSPI Alternate Bytes register,                   Address offset: 0x1C */
  __IO uint32_t DR;          /*!< QUADSPI Data register,                              Address offset: 0x20 */
  __IO uint32_t PSMKR;       /*!< QUADSPI Polling Status Mask register,               Address offset: 0x24 */
  __IO uint32_t PSMAR;       /*!< QUADSPI Polling Status Match register,              Address offset: 0x28 */
  __IO uint32_t PIR;         /*!< QUADSPI Polling Interval register,                  Address offset: 0x2C */
  __IO uint32_t LPTR;        /*!< QUADSPI Low Power Timeout register,                 Address offset: 0x30 */
} QUADSPI_TypeDef;

/**
  * @brief Reset and Clock Control
  */
typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock  Control Register,                                                    Address offset: 0x00 */
  __IO uint32_t ICSCR;         /*!< RCC Internal Clock Sources Calibration Register,                                Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC Clocks Configuration Register,                                              Address offset: 0x08 */
  __IO uint32_t PLLCFGR;       /*!< RCC System PLL configuration Register,                                          Address offset: 0x0C */
  __IO uint32_t PLLSAI1CFGR;   /*!< RCC  PLL SAI1 configuration Register,                                           Address offset: 0x10 */
uint32_t RESERVED0;            /*!< Reserved,                                                                       Address offset: 0x14 */
  __IO uint32_t CIER;          /*!< RCC Clock Interrupt Enable Register,                                            Address offset: 0x18 */
  __IO uint32_t CIFR;          /*!< RCC Clock Interrupt Flag Register,                                              Address offset: 0x1C */
  __IO uint32_t CICR;          /*!< RCC Clock Interrupt Clear Register,                                             Address offset: 0x20 */
  __IO uint32_t SMPSCR;        /*!< RCC SMPS step-down converter control register,                                  Address offset: 0x24 */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                                             Address offset: 0x28 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                                             Address offset: 0x2C */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 & AHB4 peripheral reset register,                                      Address offset: 0x30 */
uint32_t RESERVED1;            /*!< Reserved,                                                                       Address offset: 0x34 */
  __IO uint32_t APB1RSTR1;     /*!< RCC APB1 peripheral reset register 1,                                           Address offset: 0x38 */
  __IO uint32_t APB1RSTR2;     /*!< RCC APB1 peripheral reset register 2,                                           Address offset: 0x3C */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                                             Address offset: 0x40 */
  __IO uint32_t APB3RSTR;      /*!< RCC APB3 peripheral reset register,                                             Address offset: 0x44 */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clocks enable register,                                     Address offset: 0x48 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clocks enable register,                                     Address offset: 0x4C */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 & AHB4 peripheral clocks enable register,                              Address offset: 0x50 */
uint32_t RESERVED2;            /*!< Reserved,                                                                       Address offset: 0x54 */
  __IO uint32_t APB1ENR1;      /*!< RCC APB1 peripheral clocks enable register 1,                                   Address offset: 0x58 */
  __IO uint32_t APB1ENR2;      /*!< RCC APB1 peripheral clocks enable register 2,                                   Address offset: 0x5C */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clocks enable register,                                     Address offset: 0x60 */
uint32_t RESERVED3;            /*!< Reserved,                                                                       Address offset: 0x64 */
  __IO uint32_t AHB1SMENR;     /*!< RCC AHB1 peripheral clocks enable in sleep and stop modes register,             Address offset: 0x68 */
  __IO uint32_t AHB2SMENR;     /*!< RCC AHB2 peripheral clocks enable in sleep and stop modes register,             Address offset: 0x6C */
  __IO uint32_t AHB3SMENR;     /*!< RCC AHB3 & AHB4 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x70 */
uint32_t RESERVED4;            /*!< Reserved,                                                                       Address offset: 0x74 */
  __IO uint32_t APB1SMENR1;    /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 1,      Address offset: 0x78 */
  __IO uint32_t APB1SMENR2;    /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 2,      Address offset: 0x7C */
  __IO uint32_t APB2SMENR;     /*!< RCC APB2 peripheral clocks enable in sleep mode and stop modes register,        Address offset: 0x80 */
uint32_t RESERVED5;            /*!< Reserved,                                                                       Address offset: 0x84 */ 
  __IO uint32_t CCIPR;         /*!< RCC Peripherals Clock Configuration Independent Register,                       Address offset: 0x88 */
uint32_t RESERVED6;            /*!< Reserved,                                                                       Address offset: 0x8C */
  __IO uint32_t BDCR;          /*!< RCC Backup Domain Control Register,                                             Address offset: 0x90 */
  __IO uint32_t CSR;           /*!< RCC Control and Status Register,                                                Address offset: 0x94 */ 
  __IO uint32_t CRRCR;         /*!< RCC Clock Recovery RC Register,                                                 Address offset: 0x98 */
  __IO uint32_t HSECR;         /*!< RCC HSE Clock Register,                                                         Address offset: 0x9C */
uint32_t RESERVED7[26];        /*!< Reserved,                                                                       Address offset: 0xA0-0x104 */
  __IO uint32_t EXTCFGR;       /*!< RCC Extended Clock Recovery Register,                                           Address offset: 0x108 */
uint32_t RESERVED8[15];        /*!< Reserved,                                                                      Address offset: 0x10C-0x144 */
  __IO uint32_t C2AHB1ENR;     /*!< RRCC AHB1 peripheral CPU2 clocks enable register,                               Address offset: 0x148 */
  __IO uint32_t C2AHB2ENR;     /*!< RCC AHB2 peripheral CPU2 clocks enable register,                                Address offset: 0x14C */
  __IO uint32_t C2AHB3ENR;     /*!< RCC AHB3 & AHB4 peripheral CPU2 clocks enable register,,                        Address offset: 0x150 */
uint32_t RESERVED9;            /*!< Reserved,                                                                       Address offset: 0x154 */
  __IO uint32_t C2APB1ENR1;    /*!< RCC APB1 peripheral CPU2 clocks enable register 1,                              Address offset: 0x158 */
  __IO uint32_t C2APB1ENR2;    /*!< RCC APB1 peripheral CPU2 clocks enable register 2,                              Address offset: 0x15C */
  __IO uint32_t C2APB2ENR;     /*!< RCC APB2 peripheral CPU2 clocks enable register 1,                              Address offset: 0x160 */
  __IO uint32_t C2APB3ENR;     /*!< RCC APB3 peripheral CPU2 clocks enable register 1,                              Address offset: 0x164 */
  __IO uint32_t C2AHB1SMENR;   /*!< RCC AHB1 peripheral CPU2 clocks enable in sleep and stop modes register,        Address offset: 0x168 */
  __IO uint32_t C2AHB2SMENR;   /*!< RCC AHB2 peripheral CPU2 clocks enable in sleep and stop modes register,        Address offset: 0x16C */
  __IO uint32_t C2AHB3SMENR;   /*!< RCC AHB3 & AHB4 peripheral CPU2 clocks enable in sleep and stop modes register, Address offset: 0x170 */
uint32_t RESERVED10;           /*!< Reserved,                                                                                             */
  __IO uint32_t C2APB1SMENR1;  /*!< RCC APB1 peripheral CPU2 clocks enable in sleep mode and stop modes register 1, Address offset: 0x178 */
  __IO uint32_t C2APB1SMENR2;  /*!< RCC APB1 peripheral CPU2 clocks enable in sleep mode and stop modes register 2, Address offset: 0x17C */
  __IO uint32_t C2APB2SMENR;   /*!< RCC APB2 peripheral CPU2 clocks enable in sleep mode and stop modes register,   Address offset: 0x180 */
  __IO uint32_t C2APB3SMENR;   /*!< RCC APB3 peripheral CPU2 clocks enable in sleep mode and stop modes register,   Address offset: 0x184 */
} RCC_TypeDef;



/** 
  * @brief Real-Time Clock
  */
typedef struct
{
  __IO uint32_t TR;         /*!< RTC time register,                                         Address offset: 0x00 */
  __IO uint32_t DR;         /*!< RTC date register,                                         Address offset: 0x04 */
  __IO uint32_t CR;         /*!< RTC control register,                                      Address offset: 0x08 */
  __IO uint32_t ISR;        /*!< RTC initialization and status register,                    Address offset: 0x0C */
  __IO uint32_t PRER;       /*!< RTC prescaler register,                                    Address offset: 0x10 */
  __IO uint32_t WUTR;       /*!< RTC wakeup timer register,                                 Address offset: 0x14 */
       uint32_t RESERVED;   /*!< Reserved,                                                  Address offset: 0x18 */
  __IO uint32_t ALRMAR;     /*!< RTC alarm A register,                                      Address offset: 0x1C */
  __IO uint32_t ALRMBR;     /*!< RTC alarm B register,                                      Address offset: 0x20 */
  __IO uint32_t WPR;        /*!< RTC write protection register,                             Address offset: 0x24 */
  __IO uint32_t SSR;        /*!< RTC sub second register,                                   Address offset: 0x28 */
  __IO uint32_t SHIFTR;     /*!< RTC shift control register,                                Address offset: 0x2C */
  __IO uint32_t TSTR;       /*!< RTC time stamp time register,                              Address offset: 0x30 */
  __IO uint32_t TSDR;       /*!< RTC time stamp date register,                              Address offset: 0x34 */
  __IO uint32_t TSSSR;      /*!< RTC time-stamp sub second register,                        Address offset: 0x38 */
  __IO uint32_t CALR;       /*!< RTC calibration register,                                  Address offset: 0x3C */
  __IO uint32_t TAMPCR;     /*!< RTC tamper configuration register,                         Address offset: 0x40 */
  __IO uint32_t ALRMASSR;   /*!< RTC alarm A sub second register,                           Address offset: 0x44 */
  __IO uint32_t ALRMBSSR;   /*!< RTC alarm B sub second register,                           Address offset: 0x48 */
  __IO uint32_t OR;         /*!< RTC option register,                                       Address offset  0x4C */
  __IO uint32_t BKP0R;      /*!< RTC backup register 0,                                     Address offset: 0x50 */
  __IO uint32_t BKP1R;      /*!< RTC backup register 1,                                     Address offset: 0x54 */
  __IO uint32_t BKP2R;      /*!< RTC backup register 2,                                     Address offset: 0x58 */
  __IO uint32_t BKP3R;      /*!< RTC backup register 3,                                     Address offset: 0x5C */
  __IO uint32_t BKP4R;      /*!< RTC backup register 4,                                     Address offset: 0x60 */
  __IO uint32_t BKP5R;      /*!< RTC backup register 5,                                     Address offset: 0x64 */
  __IO uint32_t BKP6R;      /*!< RTC backup register 6,                                     Address offset: 0x68 */
  __IO uint32_t BKP7R;      /*!< RTC backup register 7,                                     Address offset: 0x6C */
  __IO uint32_t BKP8R;      /*!< RTC backup register 8,                                     Address offset: 0x70 */
  __IO uint32_t BKP9R;      /*!< RTC backup register 9,                                     Address offset: 0x74 */
  __IO uint32_t BKP10R;     /*!< RTC backup register 10,                                    Address offset: 0x78 */
  __IO uint32_t BKP11R;     /*!< RTC backup register 11,                                    Address offset: 0x7C */
  __IO uint32_t BKP12R;     /*!< RTC backup register 12,                                    Address offset: 0x80 */
  __IO uint32_t BKP13R;     /*!< RTC backup register 13,                                    Address offset: 0x84 */
  __IO uint32_t BKP14R;     /*!< RTC backup register 14,                                    Address offset: 0x88 */
  __IO uint32_t BKP15R;     /*!< RTC backup register 15,                                    Address offset: 0x8C */
  __IO uint32_t BKP16R;     /*!< RTC backup register 16,                                    Address offset: 0x90 */
  __IO uint32_t BKP17R;     /*!< RTC backup register 17,                                    Address offset: 0x94 */
  __IO uint32_t BKP18R;     /*!< RTC backup register 18,                                    Address offset: 0x98 */
  __IO uint32_t BKP19R;     /*!< RTC backup register 19,                                    Address offset: 0x9C */
} RTC_TypeDef;




/**
  * @brief Serial Peripheral Interface
  */
typedef struct
{
  __IO uint32_t CR1;      /*!< SPI Control register 1,       Address offset: 0x00 */
  __IO uint32_t CR2;      /*!< SPI Control register 2,       Address offset: 0x04 */
  __IO uint32_t SR;       /*!< SPI Status register,          Address offset: 0x08 */
  __IO uint32_t DR;       /*!< SPI data register,            Address offset: 0x0C */
  __IO uint32_t CRCPR;    /*!< SPI CRC polynomial register,  Address offset: 0x10 */
  __IO uint32_t RXCRCR;   /*!< SPI Rx CRC register,          Address offset: 0x14 */
  __IO uint32_t TXCRCR;   /*!< SPI Tx CRC register,          Address offset: 0x18 */
} SPI_TypeDef;

/**
  * @brief System configuration controller
  */
typedef struct
{
  __IO uint32_t MEMRMP;            /*!< SYSCFG memory remap register                                            Address offset: 0x00       */
  __IO uint32_t CFGR1;             /*!< SYSCFG configuration register 1,                                        Address offset: 0x04       */
  __IO uint32_t EXTICR[4];         /*!< SYSCFG external interrupt configuration registers,                      Address offset: 0x08-0x14  */
  __IO uint32_t SCSR;              /*!< SYSCFG SRAM2 control and status register,                               Address offset: 0x18       */
  __IO uint32_t CFGR2;             /*!< SYSCFG configuration register 2,                                        Address offset: 0x1C       */
  __IO uint32_t SWPR1;             /*!< SYSCFG SRAM2 write protection register part 1,                          Address offset: 0x20       */
  __IO uint32_t SKR;               /*!< SYSCFG SRAM2 key register,                                              Address offset: 0x24       */
  __IO uint32_t SWPR2;             /*!< SYSCFG write protection register part 2,                                Address offset: 0x28       */
       uint32_t RESERVED1[53];     /*!< Reserved,                                                               Address offset: 0x2C-0xFC  */
  __IO uint32_t IMR1;              /*!< SYSCFG CPU1 (CORTEX M4) interrupt masks control-status register part 1, Address offset: 0x100      */
  __IO uint32_t IMR2;              /*!< SYSCFG CPU1 (CORTEX M4) interrupt masks control-status register part 2, Address offset: 0x104      */
  __IO uint32_t C2IMR1;            /*!< SYSCFG CPU2 (CORTEX M0) interrupt masks control-status register part 1, Address offset: 0x108      */
  __IO uint32_t C2IMR2;            /*!< SYSCFG CPU2 (CORTEX M0) interrupt masks control-status register part 2, Address offset: 0x10C      */
  __IO uint32_t SIPCR;             /*!< SYSCFG secure IP control register,                                      Address offset: 0x110      */

} SYSCFG_TypeDef;

/**
  * @brief VREFBUF
  */
typedef struct
{
  __IO uint32_t CSR;         /*!< VREFBUF control and status register,         Address offset: 0x00 */
  __IO uint32_t CCR;         /*!< VREFBUF calibration and control register,    Address offset: 0x04 */
} VREFBUF_TypeDef;

/**
  * @brief TIM
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler register,                   Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,                 Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
  __IO uint32_t OR;          /*!< TIM option register                       Address offset: 0x50 */
  __IO uint32_t CCMR3;       /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
  __IO uint32_t CCR5;        /*!< TIM capture/compare register5,            Address offset: 0x58 */
  __IO uint32_t CCR6;        /*!< TIM capture/compare register6,            Address offset: 0x5C */
  __IO uint32_t AF1;         /*!< TIM Alternate function option register 1, Address offset: 0x60 */
  __IO uint32_t AF2;         /*!< TIM Alternate function option register 2, Address offset: 0x64 */
} TIM_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< USART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< USART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;         /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;         /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint32_t GTPR;        /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  __IO uint32_t RTOR;        /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  __IO uint32_t RQR;         /*!< USART Request register,                   Address offset: 0x18 */
  __IO uint32_t ISR;         /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;         /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint32_t RDR;         /*!< USART Receive Data register,              Address offset: 0x24 */
  __IO uint32_t TDR;         /*!< USART Transmit Data register,             Address offset: 0x28 */
  __IO uint32_t PRESC;       /*!< USART Prescaler register,                 Address offset: 0x2C */
} USART_TypeDef;


/**
  * @brief Window WATCHDOG
  */
typedef struct
{
  __IO uint32_t CR;          /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;         /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;          /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;


/**
  * @brief AES hardware accelerator
  */
typedef struct
{
  __IO uint32_t CR;          /*!< AES control register,                        Address offset: 0x00 */
  __IO uint32_t SR;          /*!< AES status register,                         Address offset: 0x04 */
  __IO uint32_t DINR;        /*!< AES data input register,                     Address offset: 0x08 */
  __IO uint32_t DOUTR;       /*!< AES data output register,                    Address offset: 0x0C */
  __IO uint32_t KEYR0;       /*!< AES key register 0,                          Address offset: 0x10 */
  __IO uint32_t KEYR1;       /*!< AES key register 1,                          Address offset: 0x14 */
  __IO uint32_t KEYR2;       /*!< AES key register 2,                          Address offset: 0x18 */
  __IO uint32_t KEYR3;       /*!< AES key register 3,                          Address offset: 0x1C */
  __IO uint32_t IVR0;        /*!< AES initialization vector register 0,        Address offset: 0x20 */
  __IO uint32_t IVR1;        /*!< AES initialization vector register 1,        Address offset: 0x24 */
  __IO uint32_t IVR2;        /*!< AES initialization vector register 2,        Address offset: 0x28 */
  __IO uint32_t IVR3;        /*!< AES initialization vector register 3,        Address offset: 0x2C */
  __IO uint32_t KEYR4;       /*!< AES key register 4,                          Address offset: 0x30 */
  __IO uint32_t KEYR5;       /*!< AES key register 5,                          Address offset: 0x34 */
  __IO uint32_t KEYR6;       /*!< AES key register 6,                          Address offset: 0x38 */
  __IO uint32_t KEYR7;       /*!< AES key register 7,                          Address offset: 0x3C */
  __IO uint32_t SUSP0R;      /*!< AES Suspend register 0,                      Address offset: 0x40 */
  __IO uint32_t SUSP1R;      /*!< AES Suspend register 1,                      Address offset: 0x44 */
  __IO uint32_t SUSP2R;      /*!< AES Suspend register 2,                      Address offset: 0x48 */
  __IO uint32_t SUSP3R;      /*!< AES Suspend register 3,                      Address offset: 0x4C */
  __IO uint32_t SUSP4R;      /*!< AES Suspend register 4,                      Address offset: 0x50 */
  __IO uint32_t SUSP5R;      /*!< AES Suspend register 5,                      Address offset: 0x54 */
  __IO uint32_t SUSP6R;      /*!< AES Suspend register 6,                      Address offset: 0x58 */
  __IO uint32_t SUSP7R;      /*!< AES Suspend register 7,                      Address offset: 0x6C */
} AES_TypeDef;

/**
  * @brief RNG
  */
typedef struct
{
  __IO uint32_t CR;  /*!< RNG control register, Address offset: 0x00 */
  __IO uint32_t SR;  /*!< RNG status register,  Address offset: 0x04 */
  __IO uint32_t DR;  /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;

/**
  * @brief Touch Sensing Controller (TSC)
  */
typedef struct
{
  __IO uint32_t CR;            /*!< TSC control register,                     Address offset: 0x00 */
  __IO uint32_t IER;           /*!< TSC interrupt enable register,            Address offset: 0x04 */
  __IO uint32_t ICR;           /*!< TSC interrupt clear register,             Address offset: 0x08 */
  __IO uint32_t ISR;           /*!< TSC interrupt status register,            Address offset: 0x0C */
  __IO uint32_t IOHCR;         /*!< TSC I/O hysteresis control register,      Address offset: 0x10 */
  uint32_t      RESERVED1;     /*!< Reserved,                                 Address offset: 0x14 */
  __IO uint32_t IOASCR;        /*!< TSC I/O analog switch control register,   Address offset: 0x18 */
  uint32_t      RESERVED2;     /*!< Reserved,                                 Address offset: 0x1C */
  __IO uint32_t IOSCR;         /*!< TSC I/O sampling control register,        Address offset: 0x20 */
  uint32_t      RESERVED3;     /*!< Reserved,                                 Address offset: 0x24 */
  __IO uint32_t IOCCR;         /*!< TSC I/O channel control register,         Address offset: 0x28 */
  uint32_t      RESERVED4;     /*!< Reserved,                                 Address offset: 0x2C */
  __IO uint32_t IOGCSR;        /*!< TSC I/O group control status register,    Address offset: 0x30 */
  __IO uint32_t IOGXCR[7];     /*!< TSC I/O group x counter register,         Address offset: 0x34-4C */
} TSC_TypeDef;

/**
  * @brief LCD
  */
typedef struct
{
  __IO uint32_t CR;          /*!< LCD control register,              Address offset: 0x00 */
  __IO uint32_t FCR;         /*!< LCD frame control register,        Address offset: 0x04 */
  __IO uint32_t SR;          /*!< LCD status register,               Address offset: 0x08 */
  __IO uint32_t CLR;         /*!< LCD clear register,                Address offset: 0x0C */
  uint32_t RESERVED;         /*!< Reserved,                          Address offset: 0x10 */
  __IO uint32_t RAM[16];     /*!< LCD display memory,           Address offset: 0x14-0x50 */
} LCD_TypeDef;

/**
  * @brief Universal Serial Bus Full Speed Device
  */
typedef struct
{
  __IO uint16_t EP0R;            /*!< USB Endpoint 0 register,                Address offset: 0x00 */
  __IO uint16_t RESERVED0;       /*!< Reserved */
  __IO uint16_t EP1R;            /*!< USB Endpoint 1 register,                Address offset: 0x04 */
  __IO uint16_t RESERVED1;       /*!< Reserved */
  __IO uint16_t EP2R;            /*!< USB Endpoint 2 register,                Address offset: 0x08 */
  __IO uint16_t RESERVED2;       /*!< Reserved */
  __IO uint16_t EP3R;            /*!< USB Endpoint 3 register,                Address offset: 0x0C */
  __IO uint16_t RESERVED3;       /*!< Reserved */
  __IO uint16_t EP4R;            /*!< USB Endpoint 4 register,                Address offset: 0x10 */
  __IO uint16_t RESERVED4;       /*!< Reserved */
  __IO uint16_t EP5R;            /*!< USB Endpoint 5 register,                Address offset: 0x14 */
  __IO uint16_t RESERVED5;       /*!< Reserved */
  __IO uint16_t EP6R;            /*!< USB Endpoint 6 register,                Address offset: 0x18 */
  __IO uint16_t RESERVED6;       /*!< Reserved */
  __IO uint16_t EP7R;            /*!< USB Endpoint 7 register,                Address offset: 0x1C */
  __IO uint16_t RESERVED7[17];   /*!< Reserved */
  __IO uint16_t CNTR;            /*!< Control register,                       Address offset: 0x40 */
  __IO uint16_t RESERVED8;       /*!< Reserved */
  __IO uint16_t ISTR;            /*!< Interrupt status register,              Address offset: 0x44 */
  __IO uint16_t RESERVED9;       /*!< Reserved */
  __IO uint16_t FNR;             /*!< Frame number register,                  Address offset: 0x48 */
  __IO uint16_t RESERVEDA;       /*!< Reserved */
  __IO uint16_t DADDR;           /*!< Device address register,                Address offset: 0x4C */
  __IO uint16_t RESERVEDB;       /*!< Reserved */
  __IO uint16_t BTABLE;          /*!< Buffer Table address register,          Address offset: 0x50 */
  __IO uint16_t RESERVEDC;       /*!< Reserved */
  __IO uint16_t LPMCSR;          /*!< LPM Control and Status register,        Address offset: 0x54 */
  __IO uint16_t RESERVEDD;       /*!< Reserved */
  __IO uint16_t BCDR;            /*!< Battery Charging detector register,     Address offset: 0x58 */
  __IO uint16_t RESERVEDE;       /*!< Reserved */
} USB_TypeDef;

/**
  * @brief Clock Recovery System
  */
typedef struct
{
  __IO uint32_t CR;            /*!< CRS control register,               Address offset: 0x00 */
  __IO uint32_t CFGR;          /*!< CRS configuration register,         Address offset: 0x04 */
  __IO uint32_t ISR;           /*!< CRS interrupt and status register,  Address offset: 0x08 */
  __IO uint32_t ICR;           /*!< CRS interrupt flag clear register,  Address offset: 0x0C */
} CRS_TypeDef;

/**
  * @brief Inter-Processor Communication
  */
typedef struct
{
  __IO uint32_t C1CR;             /*!< Inter-Processor Communication: C1 control register,                  Address offset: 0x000 */
  __IO uint32_t C1MR ;            /*!< Inter-Processor Communication: C1 mask register,                     Address offset: 0x004 */
  __IO uint32_t C1SCR;            /*!< Inter-Processor Communication: C1 status set clear register,         Address offset: 0x008 */
  __IO uint32_t C1TOC2SR;         /*!< Inter-Processor Communication: C1 to processor M4  status register,  Address offset: 0x00C */
  __IO uint32_t C2CR;             /*!< Inter-Processor Communication: C2 control register,                  Address offset: 0x010 */
  __IO uint32_t C2MR ;            /*!< Inter-Processor Communication: C2 mask register,                     Address offset: 0x014 */
  __IO uint32_t C2SCR;            /*!< Inter-Processor Communication: C2 status set clear register,         Address offset: 0x018 */
  __IO uint32_t C2TOC1SR;         /*!< Inter-Processor Communication: C2 to processor M4 status register,   Address offset: 0x01C */
} IPCC_TypeDef;

typedef struct
{
  __IO uint32_t CR;               /*!< Control register,                                                    Address offset: 0x000 */
  __IO uint32_t MR;               /*!< Mask register,                                                       Address offset: 0x004 */
  __IO uint32_t SCR;              /*!< Status set clear register,                                           Address offset: 0x008 */
  __IO uint32_t SR;               /*!< Status register,                                                     Address offset: 0x00C */
} IPCC_CommonTypeDef;

/**
  * @brief Async Interrupts and Events Controller
  */
typedef struct
{
  __IO uint32_t RTSR1;          /*!< EXTI rising trigger selection register [31:0],            Address offset: 0x00 */
  __IO uint32_t FTSR1;          /*!< EXTI falling trigger selection register [31:0],           Address offset: 0x04 */
  __IO uint32_t SWIER1;         /*!< EXTI software interrupt event register [31:0],            Address offset: 0x08 */
  __IO uint32_t PR1;            /*!< EXTI pending register [31:0],                             Address offset: 0x0C */
  __IO uint32_t RESERVED1[4];   /*!< Reserved,                                                 Address offset: 0x10 - 0x1C */
  __IO uint32_t RTSR2;          /*!< EXTI rising trigger selection register [31:0],            Address offset: 0x20 */
  __IO uint32_t FTSR2;          /*!< EXTI falling trigger selection register [31:0],           Address offset: 0x24 */
  __IO uint32_t SWIER2;         /*!< EXTI software interrupt event register [31:0],            Address offset: 0x28 */
  __IO uint32_t PR2;            /*!< EXTI pending register [31:0],                             Address offset: 0x2C */
  __IO uint32_t RESERVED2[4];   /*!< Reserved,                                                 Address offset: 0x30 - 0x3C */
  __IO uint32_t RESERVED3[8];   /*!< Reserved,                                                 Address offset: 0x40 - 0x5C */
  __IO uint32_t RESERVED4[8];   /*!< Reserved,                                                 Address offset: 0x60 - 0x7C */
  __IO uint32_t IMR1;           /*!< EXTI wakeup with interrupt mask register for cpu1 [31:0], Address offset: 0x80 */
  __IO uint32_t EMR1;           /*!< EXTI wakeup with event mask register for cpu1 [31:0],     Address offset: 0x84 */
  __IO uint32_t RESERVED5[2];   /*!< Reserved,                                                 Address offset: 0x88 - 0x8C */
  __IO uint32_t IMR2;           /*!< EXTI wakeup with interrupt mask register for cpu1 [31:0], Address offset: 0x90 */
  __IO uint32_t EMR2;           /*!< EXTI wakeup with event mask register for cpu1 [31:0],     Address offset: 0x94 */
  __IO uint32_t RESERVED8[10];  /*!< Reserved,                                                 Address offset: 0x98 - 0xBC */
  __IO uint32_t C2IMR1;         /*!< EXTI wakeup with interrupt mask register for cpu2 [31:0], Address offset: 0xC0 */
  __IO uint32_t C2EMR1;         /*!< EXTI wakeup with event mask register for cpu2 [31:0],     Address offset: 0xC4 */
  __IO uint32_t RESERVED9[2];   /*!< Reserved,                                                 Address offset: 0xC8 - 0xCC */
  __IO uint32_t C2IMR2;         /*!< EXTI wakeup with interrupt mask register for cpu2 [31:0], Address offset: 0xD0 */
  __IO uint32_t C2EMR2;         /*!< EXTI wakeup with event mask register for cpu2 [31:0],     Address offset: 0xD4 */ 
}EXTI_TypeDef;

/**
  * @brief Serial Audio Interface
  */
typedef struct
{
  __IO uint32_t GCR;          /*!< SAI global configuration register,        Address offset: 0x00 */
  uint32_t      RESERVED[16]; /*!< Reserved,                         Address offset: 0x04 to 0x40 */
  __IO uint32_t PDMCR;        /*!< SAI PDM control register,                 Address offset: 0x44 */
  __IO uint32_t PDMDLY;       /*!< SAI PDM delay register,                   Address offset: 0x48 */
} SAI_TypeDef;

typedef struct
{
  __IO uint32_t CR1;         /*!< SAI block x configuration register 1,     Address offset: 0x04 */
  __IO uint32_t CR2;         /*!< SAI block x configuration register 2,     Address offset: 0x08 */
  __IO uint32_t FRCR;        /*!< SAI block x frame configuration register, Address offset: 0x0C */
  __IO uint32_t SLOTR;       /*!< SAI block x slot register,                Address offset: 0x10 */
  __IO uint32_t IMR;         /*!< SAI block x interrupt mask register,      Address offset: 0x14 */
  __IO uint32_t SR;          /*!< SAI block x status register,              Address offset: 0x18 */
  __IO uint32_t CLRFR;       /*!< SAI block x clear flag register,          Address offset: 0x1C */
  __IO uint32_t DR;          /*!< SAI block x data register,                Address offset: 0x20 */
} SAI_Block_TypeDef;

/**
  * @brief Public Key Accelerator (PKA)
  */
typedef struct
{
  __IO uint32_t CR;          /*!< PKA control register,                 Address offset: 0x00 */
  __IO uint32_t SR;          /*!< PKA status register,                  Address offset: 0x04 */
  __IO uint32_t CLRFR;       /*!< PKA clear flag register,              Address offset: 0x08 */
  uint32_t  Reserved1[253];  /*!< Reserved                              Address offset: 0x000C-0x03FC*/
  __IO uint32_t RAM[894];    /*!< PKA RAM,                              Address offset: 0x0400-0x11F4 */
} PKA_TypeDef;

/**
  * @brief HW Semaphore HSEM
  */
typedef struct
{
  __IO uint32_t R[32];      /*!< HSEM 2-step write lock and read back registers, Address offset: 00h-7Ch  */
  __IO uint32_t RLR[32];    /*!< HSEM 1-step read lock registers,                Address offset: 80h-FCh  */
  __IO uint32_t C1IER;      /*!< HSEM CPU1 interrupt enable register ,           Address offset: 100h     */
  __IO uint32_t C1ICR;      /*!< HSEM CPU1 interrupt clear register ,            Address offset: 104h     */
  __IO uint32_t C1ISR;      /*!< HSEM CPU1 interrupt status register ,           Address offset: 108h     */
  __IO uint32_t C1MISR;     /*!< HSEM CPU1 masked interrupt status register ,    Address offset: 10Ch     */
  __IO uint32_t C2IER;      /*!< HSEM CPU2 interrupt enable register ,           Address offset: 110h     */
  __IO uint32_t C2ICR;      /*!< HSEM CPU2 interrupt clear register ,            Address offset: 114h     */
  __IO uint32_t C2ISR;      /*!< HSEM CPU2 interrupt status register ,           Address offset: 118h     */
  __IO uint32_t C2MISR;     /*!< HSEM CPU2 masked interrupt status register ,    Address offset: 11Ch     */
   uint32_t  Reserved[8];   /*!< Reserved                                        Address offset: 120h-13Ch*/
  __IO uint32_t CR;         /*!< HSEM Semaphore clear register ,                 Address offset: 140h     */
  __IO uint32_t KEYR;       /*!< HSEM Semaphore clear key register ,             Address offset: 144h     */
} HSEM_TypeDef;

typedef struct
{
  __IO uint32_t IER;        /*!< HSEM interrupt enable register ,                Address offset:   0h     */
  __IO uint32_t ICR;        /*!< HSEM interrupt clear register ,                 Address offset:   4h     */
  __IO uint32_t ISR;        /*!< HSEM interrupt status register ,                Address offset:   8h     */
  __IO uint32_t MISR;       /*!< HSEM masked interrupt status register ,         Address offset:   Ch     */
} HSEM_Common_TypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */

/*!< Boundary memory map */
#define FLASH_BASE             (0x08000000UL)/*!< FLASH(up to 1 MB) base address */
#define SRAM_BASE              (0x20000000UL)/*!< SRAM(up to 256 KB) base address */
#define PERIPH_BASE            (0x40000000UL)/*!< Peripheral base address */

/*!< Memory, OTP and Option bytes */

/* Base addresses */
