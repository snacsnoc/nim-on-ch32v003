# Peripheral access API for CH32V00XXX microcontrollers (generated using svd2nim)

import std/volatile
import std/bitops
import uncheckedenums

export volatile
export uncheckedenums

{.hint[name]: off.}

when NimMajor < 2:
  {.experimental: "overloadableEnums".}

# Some information about this device.
const DEVICE* = "CH32V00xxx"
const other_REV* = 0x0000
const MPU_PRESENT* = false
const FPU_PRESENT* = false
const VTOR_PRESENT* = true
const NVIC_PRIO_BITS* = 2
const Vendor_SysTickConfig* = false

################################################################################
# Interrupt Number Definition
################################################################################
type IRQn* = enum
# #### CPU Core Exception Numbers ##############################################
# Unknown CPU, svd2nim could not generate CPU exception numbers

# #### Device Peripheral Interrupts ############################################
  irqWWDG              =   16 # Window Watchdog interrupt
  irqPVD               =   17 # PVD through EXTI line detection interrupt
  irqFLASH             =   18 # Flash global interrupt
  irqRCC               =   19 # RCC global interrupt
  irqEXTI7_0           =   20 # EXTI Line[7:0] interrupt
  irqAWU               =   21 # AWU global interrupt
  irqDMA1_Channel1     =   22 # DMA1 Channel1 global interrupt
  irqDMA1_Channel2     =   23 # DMA1 Channel2 global interrupt
  irqDMA1_Channel3     =   24 # DMA1 Channel3 global interrupt
  irqDMA1_Channel4     =   25 # DMA1 Channel4 global interrupt
  irqDMA1_Channel5     =   26 # DMA1 Channel5 global interrupt
  irqDMA1_Channel6     =   27 # DMA1 Channel6 global interrupt
  irqDMA1_Channel7     =   28 # DMA1 Channel7 global interrupt
  irqADC               =   29 # ADC global interrupt
  irqI2C1_EV           =   30 # I2C1 event interrupt
  irqI2C1_ER           =   31 # I2C1 error interrupt
  irqUSART1            =   32 # USART1 global interrupt
  irqSPI1              =   33 # SPI1 global interrupt
  irqTIM1_BRK          =   34 # TIM1 Break interrupt
  irqTIM1_UP           =   35 # TIM1 Update interrupt
  irqTIM1_TRG_COM      =   36 # TIM1 Trigger and Commutation interrupts
  irqTIM1_CC           =   37 # TIM1 Capture Compare interrupt
  irqTIM2              =   38 # TIM2 global interrupt

################################################################################
# Type definitions for peripheral registers
################################################################################
type PWR_CTLR_Type* = object
  loc: uint

type PWR_CSR_Type* = object
  loc: uint

type PWR_AWUCSR_Type* = object
  loc: uint

type PWR_AWUAPR_Type* = object
  loc: uint

type PWR_AWUPSC_Type* = object
  loc: uint

type PWR_Type* = object
  CTLR*: PWR_CTLR_Type
  CSR*: PWR_CSR_Type
  AWUCSR*: PWR_AWUCSR_Type
  AWUAPR*: PWR_AWUAPR_Type
  AWUPSC*: PWR_AWUPSC_Type

type RCC_CTLR_Type* = object
  loc: uint

type RCC_CFGR0_Type* = object
  loc: uint

type RCC_INTR_Type* = object
  loc: uint

type RCC_APB2PRSTR_Type* = object
  loc: uint

type RCC_APB1PRSTR_Type* = object
  loc: uint

type RCC_AHBPCENR_Type* = object
  loc: uint

type RCC_APB2PCENR_Type* = object
  loc: uint

type RCC_APB1PCENR_Type* = object
  loc: uint

type RCC_RSTSCKR_Type* = object
  loc: uint

type RCC_Type* = object
  CTLR*: RCC_CTLR_Type
  CFGR0*: RCC_CFGR0_Type
  INTR*: RCC_INTR_Type
  APB2PRSTR*: RCC_APB2PRSTR_Type
  APB1PRSTR*: RCC_APB1PRSTR_Type
  AHBPCENR*: RCC_AHBPCENR_Type
  APB2PCENR*: RCC_APB2PCENR_Type
  APB1PCENR*: RCC_APB1PCENR_Type
  RSTSCKR*: RCC_RSTSCKR_Type

type EXTEND_EXTEND_CTR_Type* = object
  loc: uint

type EXTEND_EXTEND_KR_Type* = object
  loc: uint

type EXTEND_Type* = object
  EXTEND_CTR*: EXTEND_EXTEND_CTR_Type
  EXTEND_KR*: EXTEND_EXTEND_KR_Type

type GPIOA_CFGLR_Type* = object
  loc: uint

type GPIOA_INDR_Type* = object
  loc: uint

type GPIOA_OUTDR_Type* = object
  loc: uint

type GPIOA_BSHR_Type* = object
  loc: uint

type GPIOA_BCR_Type* = object
  loc: uint

type GPIOA_LCKR_Type* = object
  loc: uint

type GPIOA_Type* = object
  CFGLR*: GPIOA_CFGLR_Type
  INDR*: GPIOA_INDR_Type
  OUTDR*: GPIOA_OUTDR_Type
  BSHR*: GPIOA_BSHR_Type
  BCR*: GPIOA_BCR_Type
  LCKR*: GPIOA_LCKR_Type

type AFIO_PCFR_Type* = object
  loc: uint

type AFIO_EXTICR_Type* = object
  loc: uint

type AFIO_Type* = object
  PCFR*: AFIO_PCFR_Type
  EXTICR*: AFIO_EXTICR_Type

type EXTI_INTENR_Type* = object
  loc: uint

type EXTI_EVENR_Type* = object
  loc: uint

type EXTI_RTENR_Type* = object
  loc: uint

type EXTI_FTENR_Type* = object
  loc: uint

type EXTI_SWIEVR_Type* = object
  loc: uint

type EXTI_INTFR_Type* = object
  loc: uint

type EXTI_Type* = object
  INTENR*: EXTI_INTENR_Type
  EVENR*: EXTI_EVENR_Type
  RTENR*: EXTI_RTENR_Type
  FTENR*: EXTI_FTENR_Type
  SWIEVR*: EXTI_SWIEVR_Type
  INTFR*: EXTI_INTFR_Type

type DMA1_INTFR_Type* = object
  loc: uint

type DMA1_INTFCR_Type* = object
  loc: uint

type DMA1_CFGR1_Type* = object
  loc: uint

type DMA1_CNTR1_Type* = object
  loc: uint

type DMA1_PADDR1_Type* = object
  loc: uint

type DMA1_MADDR1_Type* = object
  loc: uint

type DMA1_CFGR2_Type* = object
  loc: uint

type DMA1_CNTR2_Type* = object
  loc: uint

type DMA1_PADDR2_Type* = object
  loc: uint

type DMA1_MADDR2_Type* = object
  loc: uint

type DMA1_CFGR3_Type* = object
  loc: uint

type DMA1_CNTR3_Type* = object
  loc: uint

type DMA1_PADDR3_Type* = object
  loc: uint

type DMA1_MADDR3_Type* = object
  loc: uint

type DMA1_CFGR4_Type* = object
  loc: uint

type DMA1_CNTR4_Type* = object
  loc: uint

type DMA1_PADDR4_Type* = object
  loc: uint

type DMA1_MADDR4_Type* = object
  loc: uint

type DMA1_CFGR5_Type* = object
  loc: uint

type DMA1_CNTR5_Type* = object
  loc: uint

type DMA1_PADDR5_Type* = object
  loc: uint

type DMA1_MADDR5_Type* = object
  loc: uint

type DMA1_CFGR6_Type* = object
  loc: uint

type DMA1_CNTR6_Type* = object
  loc: uint

type DMA1_PADDR6_Type* = object
  loc: uint

type DMA1_MADDR6_Type* = object
  loc: uint

type DMA1_CFGR7_Type* = object
  loc: uint

type DMA1_CNTR7_Type* = object
  loc: uint

type DMA1_PADDR7_Type* = object
  loc: uint

type DMA1_MADDR7_Type* = object
  loc: uint

type DMA1_Type* = object
  INTFR*: DMA1_INTFR_Type
  INTFCR*: DMA1_INTFCR_Type
  CFGR1*: DMA1_CFGR1_Type
  CNTR1*: DMA1_CNTR1_Type
  PADDR1*: DMA1_PADDR1_Type
  MADDR1*: DMA1_MADDR1_Type
  CFGR2*: DMA1_CFGR2_Type
  CNTR2*: DMA1_CNTR2_Type
  PADDR2*: DMA1_PADDR2_Type
  MADDR2*: DMA1_MADDR2_Type
  CFGR3*: DMA1_CFGR3_Type
  CNTR3*: DMA1_CNTR3_Type
  PADDR3*: DMA1_PADDR3_Type
  MADDR3*: DMA1_MADDR3_Type
  CFGR4*: DMA1_CFGR4_Type
  CNTR4*: DMA1_CNTR4_Type
  PADDR4*: DMA1_PADDR4_Type
  MADDR4*: DMA1_MADDR4_Type
  CFGR5*: DMA1_CFGR5_Type
  CNTR5*: DMA1_CNTR5_Type
  PADDR5*: DMA1_PADDR5_Type
  MADDR5*: DMA1_MADDR5_Type
  CFGR6*: DMA1_CFGR6_Type
  CNTR6*: DMA1_CNTR6_Type
  PADDR6*: DMA1_PADDR6_Type
  MADDR6*: DMA1_MADDR6_Type
  CFGR7*: DMA1_CFGR7_Type
  CNTR7*: DMA1_CNTR7_Type
  PADDR7*: DMA1_PADDR7_Type
  MADDR7*: DMA1_MADDR7_Type

type IWDG_CTLR_Type* = object
  loc: uint

type IWDG_PSCR_Type* = object
  loc: uint

type IWDG_RLDR_Type* = object
  loc: uint

type IWDG_STATR_Type* = object
  loc: uint

type IWDG_Type* = object
  CTLR*: IWDG_CTLR_Type
  PSCR*: IWDG_PSCR_Type
  RLDR*: IWDG_RLDR_Type
  STATR*: IWDG_STATR_Type

type WWDG_CTLR_Type* = object
  loc: uint

type WWDG_CFGR_Type* = object
  loc: uint

type WWDG_STATR_Type* = object
  loc: uint

type WWDG_Type* = object
  CTLR*: WWDG_CTLR_Type
  CFGR*: WWDG_CFGR_Type
  STATR*: WWDG_STATR_Type

type TIM1_CTLR1_Type* = object
  loc: uint

type TIM1_CTLR2_Type* = object
  loc: uint

type TIM1_SMCFGR_Type* = object
  loc: uint

type TIM1_DMAINTENR_Type* = object
  loc: uint

type TIM1_INTFR_Type* = object
  loc: uint

type TIM1_SWEVGR_Type* = object
  loc: uint

type TIM1_CHCTLR1_Output_Type* = object
  loc: uint

type TIM1_CHCTLR1_Input_Type* = object
  loc: uint

type TIM1_CHCTLR2_Output_Type* = object
  loc: uint

type TIM1_CHCTLR2_Input_Type* = object
  loc: uint

type TIM1_CCER_Type* = object
  loc: uint

type TIM1_CNT_Type* = object
  loc: uint

type TIM1_PSC_Type* = object
  loc: uint

type TIM1_ATRLR_Type* = object
  loc: uint

type TIM1_RPTCR_Type* = object
  loc: uint

type TIM1_CH1CVR_Type* = object
  loc: uint

type TIM1_CH2CVR_Type* = object
  loc: uint

type TIM1_CH3CVR_Type* = object
  loc: uint

type TIM1_CH4CVR_Type* = object
  loc: uint

type TIM1_BDTR_Type* = object
  loc: uint

type TIM1_DMACFGR_Type* = object
  loc: uint

type TIM1_DMAADR_Type* = object
  loc: uint

type TIM1_Type* = object
  CTLR1*: TIM1_CTLR1_Type
  CTLR2*: TIM1_CTLR2_Type
  SMCFGR*: TIM1_SMCFGR_Type
  DMAINTENR*: TIM1_DMAINTENR_Type
  INTFR*: TIM1_INTFR_Type
  SWEVGR*: TIM1_SWEVGR_Type
  CHCTLR1_Output*: TIM1_CHCTLR1_Output_Type
  CHCTLR1_Input*: TIM1_CHCTLR1_Input_Type
  CHCTLR2_Output*: TIM1_CHCTLR2_Output_Type
  CHCTLR2_Input*: TIM1_CHCTLR2_Input_Type
  CCER*: TIM1_CCER_Type
  CNT*: TIM1_CNT_Type
  PSC*: TIM1_PSC_Type
  ATRLR*: TIM1_ATRLR_Type
  RPTCR*: TIM1_RPTCR_Type
  CH1CVR*: TIM1_CH1CVR_Type
  CH2CVR*: TIM1_CH2CVR_Type
  CH3CVR*: TIM1_CH3CVR_Type
  CH4CVR*: TIM1_CH4CVR_Type
  BDTR*: TIM1_BDTR_Type
  DMACFGR*: TIM1_DMACFGR_Type
  DMAADR*: TIM1_DMAADR_Type

type TIM2_CTLR1_Type* = object
  loc: uint

type TIM2_CTLR2_Type* = object
  loc: uint

type TIM2_SMCFGR_Type* = object
  loc: uint

type TIM2_DMAINTENR_Type* = object
  loc: uint

type TIM2_INTFR_Type* = object
  loc: uint

type TIM2_SWEVGR_Type* = object
  loc: uint

type TIM2_CHCTLR1_Output_Type* = object
  loc: uint

type TIM2_CHCTLR1_Input_Type* = object
  loc: uint

type TIM2_CHCTLR2_Output_Type* = object
  loc: uint

type TIM2_CHCTLR2_Input_Type* = object
  loc: uint

type TIM2_CCER_Type* = object
  loc: uint

type TIM2_CNT_Type* = object
  loc: uint

type TIM2_PSC_Type* = object
  loc: uint

type TIM2_ATRLR_Type* = object
  loc: uint

type TIM2_CH1CVR_Type* = object
  loc: uint

type TIM2_CH2CVR_Type* = object
  loc: uint

type TIM2_CH3CVR_Type* = object
  loc: uint

type TIM2_CH4CVR_Type* = object
  loc: uint

type TIM2_DMACFGR_Type* = object
  loc: uint

type TIM2_DMAADR_Type* = object
  loc: uint

type TIM2_Type* = object
  CTLR1*: TIM2_CTLR1_Type
  CTLR2*: TIM2_CTLR2_Type
  SMCFGR*: TIM2_SMCFGR_Type
  DMAINTENR*: TIM2_DMAINTENR_Type
  INTFR*: TIM2_INTFR_Type
  SWEVGR*: TIM2_SWEVGR_Type
  CHCTLR1_Output*: TIM2_CHCTLR1_Output_Type
  CHCTLR1_Input*: TIM2_CHCTLR1_Input_Type
  CHCTLR2_Output*: TIM2_CHCTLR2_Output_Type
  CHCTLR2_Input*: TIM2_CHCTLR2_Input_Type
  CCER*: TIM2_CCER_Type
  CNT*: TIM2_CNT_Type
  PSC*: TIM2_PSC_Type
  ATRLR*: TIM2_ATRLR_Type
  CH1CVR*: TIM2_CH1CVR_Type
  CH2CVR*: TIM2_CH2CVR_Type
  CH3CVR*: TIM2_CH3CVR_Type
  CH4CVR*: TIM2_CH4CVR_Type
  DMACFGR*: TIM2_DMACFGR_Type
  DMAADR*: TIM2_DMAADR_Type

type I2C1_CTLR1_Type* = object
  loc: uint

type I2C1_CTLR2_Type* = object
  loc: uint

type I2C1_OADDR1_Type* = object
  loc: uint

type I2C1_OADDR2_Type* = object
  loc: uint

type I2C1_DATAR_Type* = object
  loc: uint

type I2C1_STAR1_Type* = object
  loc: uint

type I2C1_STAR2_Type* = object
  loc: uint

type I2C1_CKCFGR_Type* = object
  loc: uint

type I2C1_Type* = object
  CTLR1*: I2C1_CTLR1_Type
  CTLR2*: I2C1_CTLR2_Type
  OADDR1*: I2C1_OADDR1_Type
  OADDR2*: I2C1_OADDR2_Type
  DATAR*: I2C1_DATAR_Type
  STAR1*: I2C1_STAR1_Type
  STAR2*: I2C1_STAR2_Type
  CKCFGR*: I2C1_CKCFGR_Type

type SPI1_CTLR1_Type* = object
  loc: uint

type SPI1_CTLR2_Type* = object
  loc: uint

type SPI1_STATR_Type* = object
  loc: uint

type SPI1_DATAR_Type* = object
  loc: uint

type SPI1_CRCR_Type* = object
  loc: uint

type SPI1_RCRCR_Type* = object
  loc: uint

type SPI1_TCRCR_Type* = object
  loc: uint

type SPI1_HSCR_Type* = object
  loc: uint

type SPI1_Type* = object
  CTLR1*: SPI1_CTLR1_Type
  CTLR2*: SPI1_CTLR2_Type
  STATR*: SPI1_STATR_Type
  DATAR*: SPI1_DATAR_Type
  CRCR*: SPI1_CRCR_Type
  RCRCR*: SPI1_RCRCR_Type
  TCRCR*: SPI1_TCRCR_Type
  HSCR*: SPI1_HSCR_Type

type USART1_STATR_Type* = object
  loc: uint

type USART1_DATAR_Type* = object
  loc: uint

type USART1_BRR_Type* = object
  loc: uint

type USART1_CTLR1_Type* = object
  loc: uint

type USART1_CTLR2_Type* = object
  loc: uint

type USART1_CTLR3_Type* = object
  loc: uint

type USART1_GPR_Type* = object
  loc: uint

type USART1_Type* = object
  STATR*: USART1_STATR_Type
  DATAR*: USART1_DATAR_Type
  BRR*: USART1_BRR_Type
  CTLR1*: USART1_CTLR1_Type
  CTLR2*: USART1_CTLR2_Type
  CTLR3*: USART1_CTLR3_Type
  GPR*: USART1_GPR_Type

type ADC1_STATR_Type* = object
  loc: uint

type ADC1_CTLR1_Type* = object
  loc: uint

type ADC1_CTLR2_Type* = object
  loc: uint

type ADC1_SAMPTR1_CHARGE1_Type* = object
  loc: uint

type ADC1_SAMPTR2_CHARGE2_Type* = object
  loc: uint

type ADC1_IOFR1_Type* = object
  loc: uint

type ADC1_IOFR2_Type* = object
  loc: uint

type ADC1_IOFR3_Type* = object
  loc: uint

type ADC1_IOFR4_Type* = object
  loc: uint

type ADC1_WDHTR_Type* = object
  loc: uint

type ADC1_WDLTR_Type* = object
  loc: uint

type ADC1_RSQR1_Type* = object
  loc: uint

type ADC1_RSQR2_Type* = object
  loc: uint

type ADC1_RSQR3_Type* = object
  loc: uint

type ADC1_ISQR_Type* = object
  loc: uint

type ADC1_IDATAR1_Type* = object
  loc: uint

type ADC1_IDATAR2_Type* = object
  loc: uint

type ADC1_IDATAR3_Type* = object
  loc: uint

type ADC1_IDATAR4_Type* = object
  loc: uint

type ADC1_RDATAR_Type* = object
  loc: uint

type ADC1_DLYR_Type* = object
  loc: uint

type ADC1_Type* = object
  STATR*: ADC1_STATR_Type
  CTLR1*: ADC1_CTLR1_Type
  CTLR2*: ADC1_CTLR2_Type
  SAMPTR1_CHARGE1*: ADC1_SAMPTR1_CHARGE1_Type
  SAMPTR2_CHARGE2*: ADC1_SAMPTR2_CHARGE2_Type
  IOFR1*: ADC1_IOFR1_Type
  IOFR2*: ADC1_IOFR2_Type
  IOFR3*: ADC1_IOFR3_Type
  IOFR4*: ADC1_IOFR4_Type
  WDHTR*: ADC1_WDHTR_Type
  WDLTR*: ADC1_WDLTR_Type
  RSQR1*: ADC1_RSQR1_Type
  RSQR2*: ADC1_RSQR2_Type
  RSQR3*: ADC1_RSQR3_Type
  ISQR*: ADC1_ISQR_Type
  IDATAR1*: ADC1_IDATAR1_Type
  IDATAR2*: ADC1_IDATAR2_Type
  IDATAR3*: ADC1_IDATAR3_Type
  IDATAR4*: ADC1_IDATAR4_Type
  RDATAR*: ADC1_RDATAR_Type
  DLYR*: ADC1_DLYR_Type

type DBG_CFGR1_Type* = object
  loc: uint

type DBG_CFGR2_Type* = object
  loc: uint

type DBG_Type* = object
  CFGR1*: DBG_CFGR1_Type
  CFGR2*: DBG_CFGR2_Type

type ESIG_FLACAP_Type* = object
  loc: uint

type ESIG_UNIID1_Type* = object
  loc: uint

type ESIG_UNIID2_Type* = object
  loc: uint

type ESIG_UNIID3_Type* = object
  loc: uint

type ESIG_Type* = object
  FLACAP*: ESIG_FLACAP_Type
  UNIID1*: ESIG_UNIID1_Type
  UNIID2*: ESIG_UNIID2_Type
  UNIID3*: ESIG_UNIID3_Type

type FLASH_ACTLR_Type* = object
  loc: uint

type FLASH_KEYR_Type* = object
  loc: uint

type FLASH_OBKEYR_Type* = object
  loc: uint

type FLASH_STATR_Type* = object
  loc: uint

type FLASH_CTLR_Type* = object
  loc: uint

type FLASH_ADDR_Type* = object
  loc: uint

type FLASH_OBR_Type* = object
  loc: uint

type FLASH_WPR_Type* = object
  loc: uint

type FLASH_MODEKEYR_Type* = object
  loc: uint

type FLASH_BOOT_MODEKEYP_Type* = object
  loc: uint

type FLASH_Type* = object
  ACTLR*: FLASH_ACTLR_Type
  KEYR*: FLASH_KEYR_Type
  OBKEYR*: FLASH_OBKEYR_Type
  STATR*: FLASH_STATR_Type
  CTLR*: FLASH_CTLR_Type
  ADDRx*: FLASH_ADDR_Type
  OBR*: FLASH_OBR_Type
  WPR*: FLASH_WPR_Type
  MODEKEYR*: FLASH_MODEKEYR_Type
  BOOT_MODEKEYP*: FLASH_BOOT_MODEKEYP_Type

type PFIC_ISR1_Type* = object
  loc: uint

type PFIC_ISR2_Type* = object
  loc: uint

type PFIC_ISR3_Type* = object
  loc: uint

type PFIC_ISR4_Type* = object
  loc: uint

type PFIC_IPR1_Type* = object
  loc: uint

type PFIC_IPR2_Type* = object
  loc: uint

type PFIC_IPR3_Type* = object
  loc: uint

type PFIC_IPR4_Type* = object
  loc: uint

type PFIC_ITHRESDR_Type* = object
  loc: uint

type PFIC_CFGR_Type* = object
  loc: uint

type PFIC_GISR_Type* = object
  loc: uint

type PFIC_VTFIDR_Type* = object
  loc: uint

type PFIC_VTFADDRR0_Type* = object
  loc: uint

type PFIC_VTFADDRR1_Type* = object
  loc: uint

type PFIC_VTFADDRR2_Type* = object
  loc: uint

type PFIC_VTFADDRR3_Type* = object
  loc: uint

type PFIC_IENR1_Type* = object
  loc: uint

type PFIC_IENR2_Type* = object
  loc: uint

type PFIC_IENR3_Type* = object
  loc: uint

type PFIC_IENR4_Type* = object
  loc: uint

type PFIC_IRER1_Type* = object
  loc: uint

type PFIC_IRER2_Type* = object
  loc: uint

type PFIC_IRER3_Type* = object
  loc: uint

type PFIC_IRER4_Type* = object
  loc: uint

type PFIC_IPSR1_Type* = object
  loc: uint

type PFIC_IPSR2_Type* = object
  loc: uint

type PFIC_IPSR3_Type* = object
  loc: uint

type PFIC_IPSR4_Type* = object
  loc: uint

type PFIC_IPRR1_Type* = object
  loc: uint

type PFIC_IPRR2_Type* = object
  loc: uint

type PFIC_IPRR3_Type* = object
  loc: uint

type PFIC_IPRR4_Type* = object
  loc: uint

type PFIC_IACTR1_Type* = object
  loc: uint

type PFIC_IACTR2_Type* = object
  loc: uint

type PFIC_IACTR3_Type* = object
  loc: uint

type PFIC_IACTR4_Type* = object
  loc: uint

type PFIC_IPRIOR0_Type* = object
  loc: uint

type PFIC_IPRIOR1_Type* = object
  loc: uint

type PFIC_IPRIOR2_Type* = object
  loc: uint

type PFIC_IPRIOR3_Type* = object
  loc: uint

type PFIC_IPRIOR4_Type* = object
  loc: uint

type PFIC_IPRIOR5_Type* = object
  loc: uint

type PFIC_IPRIOR6_Type* = object
  loc: uint

type PFIC_IPRIOR7_Type* = object
  loc: uint

type PFIC_IPRIOR8_Type* = object
  loc: uint

type PFIC_IPRIOR9_Type* = object
  loc: uint

type PFIC_IPRIOR10_Type* = object
  loc: uint

type PFIC_IPRIOR11_Type* = object
  loc: uint

type PFIC_IPRIOR12_Type* = object
  loc: uint

type PFIC_IPRIOR13_Type* = object
  loc: uint

type PFIC_IPRIOR14_Type* = object
  loc: uint

type PFIC_IPRIOR15_Type* = object
  loc: uint

type PFIC_IPRIOR16_Type* = object
  loc: uint

type PFIC_IPRIOR17_Type* = object
  loc: uint

type PFIC_IPRIOR18_Type* = object
  loc: uint

type PFIC_IPRIOR19_Type* = object
  loc: uint

type PFIC_IPRIOR20_Type* = object
  loc: uint

type PFIC_IPRIOR21_Type* = object
  loc: uint

type PFIC_IPRIOR22_Type* = object
  loc: uint

type PFIC_IPRIOR23_Type* = object
  loc: uint

type PFIC_IPRIOR24_Type* = object
  loc: uint

type PFIC_IPRIOR25_Type* = object
  loc: uint

type PFIC_IPRIOR26_Type* = object
  loc: uint

type PFIC_IPRIOR27_Type* = object
  loc: uint

type PFIC_IPRIOR28_Type* = object
  loc: uint

type PFIC_IPRIOR29_Type* = object
  loc: uint

type PFIC_IPRIOR30_Type* = object
  loc: uint

type PFIC_IPRIOR31_Type* = object
  loc: uint

type PFIC_IPRIOR32_Type* = object
  loc: uint

type PFIC_IPRIOR33_Type* = object
  loc: uint

type PFIC_IPRIOR34_Type* = object
  loc: uint

type PFIC_IPRIOR35_Type* = object
  loc: uint

type PFIC_IPRIOR36_Type* = object
  loc: uint

type PFIC_IPRIOR37_Type* = object
  loc: uint

type PFIC_IPRIOR38_Type* = object
  loc: uint

type PFIC_IPRIOR39_Type* = object
  loc: uint

type PFIC_IPRIOR40_Type* = object
  loc: uint

type PFIC_IPRIOR41_Type* = object
  loc: uint

type PFIC_IPRIOR42_Type* = object
  loc: uint

type PFIC_IPRIOR43_Type* = object
  loc: uint

type PFIC_IPRIOR44_Type* = object
  loc: uint

type PFIC_IPRIOR45_Type* = object
  loc: uint

type PFIC_IPRIOR46_Type* = object
  loc: uint

type PFIC_IPRIOR47_Type* = object
  loc: uint

type PFIC_IPRIOR48_Type* = object
  loc: uint

type PFIC_IPRIOR49_Type* = object
  loc: uint

type PFIC_IPRIOR50_Type* = object
  loc: uint

type PFIC_IPRIOR51_Type* = object
  loc: uint

type PFIC_IPRIOR52_Type* = object
  loc: uint

type PFIC_IPRIOR53_Type* = object
  loc: uint

type PFIC_IPRIOR54_Type* = object
  loc: uint

type PFIC_IPRIOR55_Type* = object
  loc: uint

type PFIC_IPRIOR56_Type* = object
  loc: uint

type PFIC_IPRIOR57_Type* = object
  loc: uint

type PFIC_IPRIOR58_Type* = object
  loc: uint

type PFIC_IPRIOR59_Type* = object
  loc: uint

type PFIC_IPRIOR60_Type* = object
  loc: uint

type PFIC_IPRIOR61_Type* = object
  loc: uint

type PFIC_IPRIOR62_Type* = object
  loc: uint

type PFIC_IPRIOR63_Type* = object
  loc: uint

type PFIC_SCTLR_Type* = object
  loc: uint

type PFIC_STK_CTLR_Type* = object
  loc: uint

type PFIC_STK_SR_Type* = object
  loc: uint

type PFIC_STK_CNTL_Type* = object
  loc: uint

type PFIC_STK_CMPLR_Type* = object
  loc: uint

type PFIC_Type* = object
  ISR1*: PFIC_ISR1_Type
  ISR2*: PFIC_ISR2_Type
  ISR3*: PFIC_ISR3_Type
  ISR4*: PFIC_ISR4_Type
  IPR1*: PFIC_IPR1_Type
  IPR2*: PFIC_IPR2_Type
  IPR3*: PFIC_IPR3_Type
  IPR4*: PFIC_IPR4_Type
  ITHRESDR*: PFIC_ITHRESDR_Type
  CFGR*: PFIC_CFGR_Type
  GISR*: PFIC_GISR_Type
  VTFIDR*: PFIC_VTFIDR_Type
  VTFADDRR0*: PFIC_VTFADDRR0_Type
  VTFADDRR1*: PFIC_VTFADDRR1_Type
  VTFADDRR2*: PFIC_VTFADDRR2_Type
  VTFADDRR3*: PFIC_VTFADDRR3_Type
  IENR1*: PFIC_IENR1_Type
  IENR2*: PFIC_IENR2_Type
  IENR3*: PFIC_IENR3_Type
  IENR4*: PFIC_IENR4_Type
  IRER1*: PFIC_IRER1_Type
  IRER2*: PFIC_IRER2_Type
  IRER3*: PFIC_IRER3_Type
  IRER4*: PFIC_IRER4_Type
  IPSR1*: PFIC_IPSR1_Type
  IPSR2*: PFIC_IPSR2_Type
  IPSR3*: PFIC_IPSR3_Type
  IPSR4*: PFIC_IPSR4_Type
  IPRR1*: PFIC_IPRR1_Type
  IPRR2*: PFIC_IPRR2_Type
  IPRR3*: PFIC_IPRR3_Type
  IPRR4*: PFIC_IPRR4_Type
  IACTR1*: PFIC_IACTR1_Type
  IACTR2*: PFIC_IACTR2_Type
  IACTR3*: PFIC_IACTR3_Type
  IACTR4*: PFIC_IACTR4_Type
  IPRIOR0*: PFIC_IPRIOR0_Type
  IPRIOR1*: PFIC_IPRIOR1_Type
  IPRIOR2*: PFIC_IPRIOR2_Type
  IPRIOR3*: PFIC_IPRIOR3_Type
  IPRIOR4*: PFIC_IPRIOR4_Type
  IPRIOR5*: PFIC_IPRIOR5_Type
  IPRIOR6*: PFIC_IPRIOR6_Type
  IPRIOR7*: PFIC_IPRIOR7_Type
  IPRIOR8*: PFIC_IPRIOR8_Type
  IPRIOR9*: PFIC_IPRIOR9_Type
  IPRIOR10*: PFIC_IPRIOR10_Type
  IPRIOR11*: PFIC_IPRIOR11_Type
  IPRIOR12*: PFIC_IPRIOR12_Type
  IPRIOR13*: PFIC_IPRIOR13_Type
  IPRIOR14*: PFIC_IPRIOR14_Type
  IPRIOR15*: PFIC_IPRIOR15_Type
  IPRIOR16*: PFIC_IPRIOR16_Type
  IPRIOR17*: PFIC_IPRIOR17_Type
  IPRIOR18*: PFIC_IPRIOR18_Type
  IPRIOR19*: PFIC_IPRIOR19_Type
  IPRIOR20*: PFIC_IPRIOR20_Type
  IPRIOR21*: PFIC_IPRIOR21_Type
  IPRIOR22*: PFIC_IPRIOR22_Type
  IPRIOR23*: PFIC_IPRIOR23_Type
  IPRIOR24*: PFIC_IPRIOR24_Type
  IPRIOR25*: PFIC_IPRIOR25_Type
  IPRIOR26*: PFIC_IPRIOR26_Type
  IPRIOR27*: PFIC_IPRIOR27_Type
  IPRIOR28*: PFIC_IPRIOR28_Type
  IPRIOR29*: PFIC_IPRIOR29_Type
  IPRIOR30*: PFIC_IPRIOR30_Type
  IPRIOR31*: PFIC_IPRIOR31_Type
  IPRIOR32*: PFIC_IPRIOR32_Type
  IPRIOR33*: PFIC_IPRIOR33_Type
  IPRIOR34*: PFIC_IPRIOR34_Type
  IPRIOR35*: PFIC_IPRIOR35_Type
  IPRIOR36*: PFIC_IPRIOR36_Type
  IPRIOR37*: PFIC_IPRIOR37_Type
  IPRIOR38*: PFIC_IPRIOR38_Type
  IPRIOR39*: PFIC_IPRIOR39_Type
  IPRIOR40*: PFIC_IPRIOR40_Type
  IPRIOR41*: PFIC_IPRIOR41_Type
  IPRIOR42*: PFIC_IPRIOR42_Type
  IPRIOR43*: PFIC_IPRIOR43_Type
  IPRIOR44*: PFIC_IPRIOR44_Type
  IPRIOR45*: PFIC_IPRIOR45_Type
  IPRIOR46*: PFIC_IPRIOR46_Type
  IPRIOR47*: PFIC_IPRIOR47_Type
  IPRIOR48*: PFIC_IPRIOR48_Type
  IPRIOR49*: PFIC_IPRIOR49_Type
  IPRIOR50*: PFIC_IPRIOR50_Type
  IPRIOR51*: PFIC_IPRIOR51_Type
  IPRIOR52*: PFIC_IPRIOR52_Type
  IPRIOR53*: PFIC_IPRIOR53_Type
  IPRIOR54*: PFIC_IPRIOR54_Type
  IPRIOR55*: PFIC_IPRIOR55_Type
  IPRIOR56*: PFIC_IPRIOR56_Type
  IPRIOR57*: PFIC_IPRIOR57_Type
  IPRIOR58*: PFIC_IPRIOR58_Type
  IPRIOR59*: PFIC_IPRIOR59_Type
  IPRIOR60*: PFIC_IPRIOR60_Type
  IPRIOR61*: PFIC_IPRIOR61_Type
  IPRIOR62*: PFIC_IPRIOR62_Type
  IPRIOR63*: PFIC_IPRIOR63_Type
  SCTLR*: PFIC_SCTLR_Type
  STK_CTLR*: PFIC_STK_CTLR_Type
  STK_SR*: PFIC_STK_SR_Type
  STK_CNTL*: PFIC_STK_CNTL_Type
  STK_CMPLR*: PFIC_STK_CMPLR_Type


################################################################################
# Peripheral object instances
################################################################################
const PWR* = PWR_Type(
  CTLR: PWR_CTLR_Type(loc: 0x40007000'u),
  CSR: PWR_CSR_Type(loc: 0x40007004'u),
  AWUCSR: PWR_AWUCSR_Type(loc: 0x40007008'u),
  AWUAPR: PWR_AWUAPR_Type(loc: 0x4000700c'u),
  AWUPSC: PWR_AWUPSC_Type(loc: 0x40007010'u),
)

const RCC* = RCC_Type(
  CTLR: RCC_CTLR_Type(loc: 0x40021000'u),
  CFGR0: RCC_CFGR0_Type(loc: 0x40021004'u),
  INTR: RCC_INTR_Type(loc: 0x40021008'u),
  APB2PRSTR: RCC_APB2PRSTR_Type(loc: 0x4002100c'u),
  APB1PRSTR: RCC_APB1PRSTR_Type(loc: 0x40021010'u),
  AHBPCENR: RCC_AHBPCENR_Type(loc: 0x40021014'u),
  APB2PCENR: RCC_APB2PCENR_Type(loc: 0x40021018'u),
  APB1PCENR: RCC_APB1PCENR_Type(loc: 0x4002101c'u),
  RSTSCKR: RCC_RSTSCKR_Type(loc: 0x40021024'u),
)

const EXTEND* = EXTEND_Type(
  EXTEND_CTR: EXTEND_EXTEND_CTR_Type(loc: 0x40023800'u),
  EXTEND_KR: EXTEND_EXTEND_KR_Type(loc: 0x40023804'u),
)

const GPIOA* = GPIOA_Type(
  CFGLR: GPIOA_CFGLR_Type(loc: 0x40010800'u),
  INDR: GPIOA_INDR_Type(loc: 0x40010808'u),
  OUTDR: GPIOA_OUTDR_Type(loc: 0x4001080c'u),
  BSHR: GPIOA_BSHR_Type(loc: 0x40010810'u),
  BCR: GPIOA_BCR_Type(loc: 0x40010814'u),
  LCKR: GPIOA_LCKR_Type(loc: 0x40010818'u),
)

const GPIOC* = GPIOA_Type(
  CFGLR: GPIOA_CFGLR_Type(loc: 0x40011000'u),
  INDR: GPIOA_INDR_Type(loc: 0x40011008'u),
  OUTDR: GPIOA_OUTDR_Type(loc: 0x4001100c'u),
  BSHR: GPIOA_BSHR_Type(loc: 0x40011010'u),
  BCR: GPIOA_BCR_Type(loc: 0x40011014'u),
  LCKR: GPIOA_LCKR_Type(loc: 0x40011018'u),
)

const GPIOD* = GPIOA_Type(
  CFGLR: GPIOA_CFGLR_Type(loc: 0x40011400'u),
  INDR: GPIOA_INDR_Type(loc: 0x40011408'u),
  OUTDR: GPIOA_OUTDR_Type(loc: 0x4001140c'u),
  BSHR: GPIOA_BSHR_Type(loc: 0x40011410'u),
  BCR: GPIOA_BCR_Type(loc: 0x40011414'u),
  LCKR: GPIOA_LCKR_Type(loc: 0x40011418'u),
)

const AFIO* = AFIO_Type(
  PCFR: AFIO_PCFR_Type(loc: 0x40010004'u),
  EXTICR: AFIO_EXTICR_Type(loc: 0x40010008'u),
)

const EXTI* = EXTI_Type(
  INTENR: EXTI_INTENR_Type(loc: 0x40010400'u),
  EVENR: EXTI_EVENR_Type(loc: 0x40010404'u),
  RTENR: EXTI_RTENR_Type(loc: 0x40010408'u),
  FTENR: EXTI_FTENR_Type(loc: 0x4001040c'u),
  SWIEVR: EXTI_SWIEVR_Type(loc: 0x40010410'u),
  INTFR: EXTI_INTFR_Type(loc: 0x40010414'u),
)

const DMA1* = DMA1_Type(
  INTFR: DMA1_INTFR_Type(loc: 0x40020000'u),
  INTFCR: DMA1_INTFCR_Type(loc: 0x40020004'u),
  CFGR1: DMA1_CFGR1_Type(loc: 0x40020008'u),
  CNTR1: DMA1_CNTR1_Type(loc: 0x4002000c'u),
  PADDR1: DMA1_PADDR1_Type(loc: 0x40020010'u),
  MADDR1: DMA1_MADDR1_Type(loc: 0x40020014'u),
  CFGR2: DMA1_CFGR2_Type(loc: 0x4002001c'u),
  CNTR2: DMA1_CNTR2_Type(loc: 0x40020020'u),
  PADDR2: DMA1_PADDR2_Type(loc: 0x40020024'u),
  MADDR2: DMA1_MADDR2_Type(loc: 0x40020028'u),
  CFGR3: DMA1_CFGR3_Type(loc: 0x40020030'u),
  CNTR3: DMA1_CNTR3_Type(loc: 0x40020034'u),
  PADDR3: DMA1_PADDR3_Type(loc: 0x40020038'u),
  MADDR3: DMA1_MADDR3_Type(loc: 0x4002003c'u),
  CFGR4: DMA1_CFGR4_Type(loc: 0x40020044'u),
  CNTR4: DMA1_CNTR4_Type(loc: 0x40020048'u),
  PADDR4: DMA1_PADDR4_Type(loc: 0x4002004c'u),
  MADDR4: DMA1_MADDR4_Type(loc: 0x40020050'u),
  CFGR5: DMA1_CFGR5_Type(loc: 0x40020058'u),
  CNTR5: DMA1_CNTR5_Type(loc: 0x4002005c'u),
  PADDR5: DMA1_PADDR5_Type(loc: 0x40020060'u),
  MADDR5: DMA1_MADDR5_Type(loc: 0x40020064'u),
  CFGR6: DMA1_CFGR6_Type(loc: 0x4002006c'u),
  CNTR6: DMA1_CNTR6_Type(loc: 0x40020070'u),
  PADDR6: DMA1_PADDR6_Type(loc: 0x40020074'u),
  MADDR6: DMA1_MADDR6_Type(loc: 0x40020078'u),
  CFGR7: DMA1_CFGR7_Type(loc: 0x40020080'u),
  CNTR7: DMA1_CNTR7_Type(loc: 0x40020084'u),
  PADDR7: DMA1_PADDR7_Type(loc: 0x40020088'u),
  MADDR7: DMA1_MADDR7_Type(loc: 0x4002008c'u),
)

const IWDG* = IWDG_Type(
  CTLR: IWDG_CTLR_Type(loc: 0x40003000'u),
  PSCR: IWDG_PSCR_Type(loc: 0x40003004'u),
  RLDR: IWDG_RLDR_Type(loc: 0x40003008'u),
  STATR: IWDG_STATR_Type(loc: 0x4000300c'u),
)

const WWDG* = WWDG_Type(
  CTLR: WWDG_CTLR_Type(loc: 0x40002c00'u),
  CFGR: WWDG_CFGR_Type(loc: 0x40002c04'u),
  STATR: WWDG_STATR_Type(loc: 0x40002c08'u),
)

const TIM1* = TIM1_Type(
  CTLR1: TIM1_CTLR1_Type(loc: 0x40012c00'u),
  CTLR2: TIM1_CTLR2_Type(loc: 0x40012c04'u),
  SMCFGR: TIM1_SMCFGR_Type(loc: 0x40012c08'u),
  DMAINTENR: TIM1_DMAINTENR_Type(loc: 0x40012c0c'u),
  INTFR: TIM1_INTFR_Type(loc: 0x40012c10'u),
  SWEVGR: TIM1_SWEVGR_Type(loc: 0x40012c14'u),
  CHCTLR1_Output: TIM1_CHCTLR1_Output_Type(loc: 0x40012c18'u),
  CHCTLR1_Input: TIM1_CHCTLR1_Input_Type(loc: 0x40012c18'u),
  CHCTLR2_Output: TIM1_CHCTLR2_Output_Type(loc: 0x40012c1c'u),
  CHCTLR2_Input: TIM1_CHCTLR2_Input_Type(loc: 0x40012c1c'u),
  CCER: TIM1_CCER_Type(loc: 0x40012c20'u),
  CNT: TIM1_CNT_Type(loc: 0x40012c24'u),
  PSC: TIM1_PSC_Type(loc: 0x40012c28'u),
  ATRLR: TIM1_ATRLR_Type(loc: 0x40012c2c'u),
  RPTCR: TIM1_RPTCR_Type(loc: 0x40012c30'u),
  CH1CVR: TIM1_CH1CVR_Type(loc: 0x40012c34'u),
  CH2CVR: TIM1_CH2CVR_Type(loc: 0x40012c38'u),
  CH3CVR: TIM1_CH3CVR_Type(loc: 0x40012c3c'u),
  CH4CVR: TIM1_CH4CVR_Type(loc: 0x40012c40'u),
  BDTR: TIM1_BDTR_Type(loc: 0x40012c44'u),
  DMACFGR: TIM1_DMACFGR_Type(loc: 0x40012c48'u),
  DMAADR: TIM1_DMAADR_Type(loc: 0x40012c4c'u),
)

const TIM2* = TIM2_Type(
  CTLR1: TIM2_CTLR1_Type(loc: 0x40000000'u),
  CTLR2: TIM2_CTLR2_Type(loc: 0x40000004'u),
  SMCFGR: TIM2_SMCFGR_Type(loc: 0x40000008'u),
  DMAINTENR: TIM2_DMAINTENR_Type(loc: 0x4000000c'u),
  INTFR: TIM2_INTFR_Type(loc: 0x40000010'u),
  SWEVGR: TIM2_SWEVGR_Type(loc: 0x40000014'u),
  CHCTLR1_Output: TIM2_CHCTLR1_Output_Type(loc: 0x40000018'u),
  CHCTLR1_Input: TIM2_CHCTLR1_Input_Type(loc: 0x40000018'u),
  CHCTLR2_Output: TIM2_CHCTLR2_Output_Type(loc: 0x4000001c'u),
  CHCTLR2_Input: TIM2_CHCTLR2_Input_Type(loc: 0x4000001c'u),
  CCER: TIM2_CCER_Type(loc: 0x40000020'u),
  CNT: TIM2_CNT_Type(loc: 0x40000024'u),
  PSC: TIM2_PSC_Type(loc: 0x40000028'u),
  ATRLR: TIM2_ATRLR_Type(loc: 0x4000002c'u),
  CH1CVR: TIM2_CH1CVR_Type(loc: 0x40000034'u),
  CH2CVR: TIM2_CH2CVR_Type(loc: 0x40000038'u),
  CH3CVR: TIM2_CH3CVR_Type(loc: 0x4000003c'u),
  CH4CVR: TIM2_CH4CVR_Type(loc: 0x40000040'u),
  DMACFGR: TIM2_DMACFGR_Type(loc: 0x40000048'u),
  DMAADR: TIM2_DMAADR_Type(loc: 0x4000004c'u),
)

const I2C1* = I2C1_Type(
  CTLR1: I2C1_CTLR1_Type(loc: 0x40005400'u),
  CTLR2: I2C1_CTLR2_Type(loc: 0x40005404'u),
  OADDR1: I2C1_OADDR1_Type(loc: 0x40005408'u),
  OADDR2: I2C1_OADDR2_Type(loc: 0x4000540c'u),
  DATAR: I2C1_DATAR_Type(loc: 0x40005410'u),
  STAR1: I2C1_STAR1_Type(loc: 0x40005414'u),
  STAR2: I2C1_STAR2_Type(loc: 0x40005418'u),
  CKCFGR: I2C1_CKCFGR_Type(loc: 0x4000541c'u),
)

const SPI1* = SPI1_Type(
  CTLR1: SPI1_CTLR1_Type(loc: 0x40013000'u),
  CTLR2: SPI1_CTLR2_Type(loc: 0x40013004'u),
  STATR: SPI1_STATR_Type(loc: 0x40013008'u),
  DATAR: SPI1_DATAR_Type(loc: 0x4001300c'u),
  CRCR: SPI1_CRCR_Type(loc: 0x40013010'u),
  RCRCR: SPI1_RCRCR_Type(loc: 0x40013014'u),
  TCRCR: SPI1_TCRCR_Type(loc: 0x40013018'u),
  HSCR: SPI1_HSCR_Type(loc: 0x40013024'u),
)

const USART1* = USART1_Type(
  STATR: USART1_STATR_Type(loc: 0x40013800'u),
  DATAR: USART1_DATAR_Type(loc: 0x40013804'u),
  BRR: USART1_BRR_Type(loc: 0x40013808'u),
  CTLR1: USART1_CTLR1_Type(loc: 0x4001380c'u),
  CTLR2: USART1_CTLR2_Type(loc: 0x40013810'u),
  CTLR3: USART1_CTLR3_Type(loc: 0x40013814'u),
  GPR: USART1_GPR_Type(loc: 0x40013818'u),
)

const ADC1* = ADC1_Type(
  STATR: ADC1_STATR_Type(loc: 0x40012400'u),
  CTLR1: ADC1_CTLR1_Type(loc: 0x40012404'u),
  CTLR2: ADC1_CTLR2_Type(loc: 0x40012408'u),
  SAMPTR1_CHARGE1: ADC1_SAMPTR1_CHARGE1_Type(loc: 0x4001240c'u),
  SAMPTR2_CHARGE2: ADC1_SAMPTR2_CHARGE2_Type(loc: 0x40012410'u),
  IOFR1: ADC1_IOFR1_Type(loc: 0x40012414'u),
  IOFR2: ADC1_IOFR2_Type(loc: 0x40012418'u),
  IOFR3: ADC1_IOFR3_Type(loc: 0x4001241c'u),
  IOFR4: ADC1_IOFR4_Type(loc: 0x40012420'u),
  WDHTR: ADC1_WDHTR_Type(loc: 0x40012424'u),
  WDLTR: ADC1_WDLTR_Type(loc: 0x40012428'u),
  RSQR1: ADC1_RSQR1_Type(loc: 0x4001242c'u),
  RSQR2: ADC1_RSQR2_Type(loc: 0x40012430'u),
  RSQR3: ADC1_RSQR3_Type(loc: 0x40012434'u),
  ISQR: ADC1_ISQR_Type(loc: 0x40012438'u),
  IDATAR1: ADC1_IDATAR1_Type(loc: 0x4001243c'u),
  IDATAR2: ADC1_IDATAR2_Type(loc: 0x40012440'u),
  IDATAR3: ADC1_IDATAR3_Type(loc: 0x40012444'u),
  IDATAR4: ADC1_IDATAR4_Type(loc: 0x40012448'u),
  RDATAR: ADC1_RDATAR_Type(loc: 0x4001244c'u),
  DLYR: ADC1_DLYR_Type(loc: 0x40012450'u),
)

const DBG* = DBG_Type(
  CFGR1: DBG_CFGR1_Type(loc: 0xe000d000'u),
  CFGR2: DBG_CFGR2_Type(loc: 0xe000d004'u),
)

const ESIG* = ESIG_Type(
  FLACAP: ESIG_FLACAP_Type(loc: 0x1ffff7e0'u),
  UNIID1: ESIG_UNIID1_Type(loc: 0x1ffff7e8'u),
  UNIID2: ESIG_UNIID2_Type(loc: 0x1ffff7ec'u),
  UNIID3: ESIG_UNIID3_Type(loc: 0x1ffff7f0'u),
)

const FLASH* = FLASH_Type(
  ACTLR: FLASH_ACTLR_Type(loc: 0x40022000'u),
  KEYR: FLASH_KEYR_Type(loc: 0x40022004'u),
  OBKEYR: FLASH_OBKEYR_Type(loc: 0x40022008'u),
  STATR: FLASH_STATR_Type(loc: 0x4002200c'u),
  CTLR: FLASH_CTLR_Type(loc: 0x40022010'u),
  ADDRx: FLASH_ADDR_Type(loc: 0x40022014'u),
  OBR: FLASH_OBR_Type(loc: 0x4002201c'u),
  WPR: FLASH_WPR_Type(loc: 0x40022020'u),
  MODEKEYR: FLASH_MODEKEYR_Type(loc: 0x40022024'u),
  BOOT_MODEKEYP: FLASH_BOOT_MODEKEYP_Type(loc: 0x40022028'u),
)

const PFIC* = PFIC_Type(
  ISR1: PFIC_ISR1_Type(loc: 0xe000e000'u),
  ISR2: PFIC_ISR2_Type(loc: 0xe000e004'u),
  ISR3: PFIC_ISR3_Type(loc: 0xe000e008'u),
  ISR4: PFIC_ISR4_Type(loc: 0xe000e00c'u),
  IPR1: PFIC_IPR1_Type(loc: 0xe000e020'u),
  IPR2: PFIC_IPR2_Type(loc: 0xe000e024'u),
  IPR3: PFIC_IPR3_Type(loc: 0xe000e028'u),
  IPR4: PFIC_IPR4_Type(loc: 0xe000e02c'u),
  ITHRESDR: PFIC_ITHRESDR_Type(loc: 0xe000e040'u),
  CFGR: PFIC_CFGR_Type(loc: 0xe000e048'u),
  GISR: PFIC_GISR_Type(loc: 0xe000e04c'u),
  VTFIDR: PFIC_VTFIDR_Type(loc: 0xe000e050'u),
  VTFADDRR0: PFIC_VTFADDRR0_Type(loc: 0xe000e060'u),
  VTFADDRR1: PFIC_VTFADDRR1_Type(loc: 0xe000e064'u),
  VTFADDRR2: PFIC_VTFADDRR2_Type(loc: 0xe000e068'u),
  VTFADDRR3: PFIC_VTFADDRR3_Type(loc: 0xe000e06c'u),
  IENR1: PFIC_IENR1_Type(loc: 0xe000e100'u),
  IENR2: PFIC_IENR2_Type(loc: 0xe000e104'u),
  IENR3: PFIC_IENR3_Type(loc: 0xe000e108'u),
  IENR4: PFIC_IENR4_Type(loc: 0xe000e10c'u),
  IRER1: PFIC_IRER1_Type(loc: 0xe000e180'u),
  IRER2: PFIC_IRER2_Type(loc: 0xe000e184'u),
  IRER3: PFIC_IRER3_Type(loc: 0xe000e188'u),
  IRER4: PFIC_IRER4_Type(loc: 0xe000e18c'u),
  IPSR1: PFIC_IPSR1_Type(loc: 0xe000e200'u),
  IPSR2: PFIC_IPSR2_Type(loc: 0xe000e204'u),
  IPSR3: PFIC_IPSR3_Type(loc: 0xe000e208'u),
  IPSR4: PFIC_IPSR4_Type(loc: 0xe000e20c'u),
  IPRR1: PFIC_IPRR1_Type(loc: 0xe000e280'u),
  IPRR2: PFIC_IPRR2_Type(loc: 0xe000e284'u),
  IPRR3: PFIC_IPRR3_Type(loc: 0xe000e288'u),
  IPRR4: PFIC_IPRR4_Type(loc: 0xe000e28c'u),
  IACTR1: PFIC_IACTR1_Type(loc: 0xe000e300'u),
  IACTR2: PFIC_IACTR2_Type(loc: 0xe000e304'u),
  IACTR3: PFIC_IACTR3_Type(loc: 0xe000e308'u),
  IACTR4: PFIC_IACTR4_Type(loc: 0xe000e30c'u),
  IPRIOR0: PFIC_IPRIOR0_Type(loc: 0xe000e400'u),
  IPRIOR1: PFIC_IPRIOR1_Type(loc: 0xe000e401'u),
  IPRIOR2: PFIC_IPRIOR2_Type(loc: 0xe000e402'u),
  IPRIOR3: PFIC_IPRIOR3_Type(loc: 0xe000e403'u),
  IPRIOR4: PFIC_IPRIOR4_Type(loc: 0xe000e404'u),
  IPRIOR5: PFIC_IPRIOR5_Type(loc: 0xe000e405'u),
  IPRIOR6: PFIC_IPRIOR6_Type(loc: 0xe000e406'u),
  IPRIOR7: PFIC_IPRIOR7_Type(loc: 0xe000e407'u),
  IPRIOR8: PFIC_IPRIOR8_Type(loc: 0xe000e408'u),
  IPRIOR9: PFIC_IPRIOR9_Type(loc: 0xe000e409'u),
  IPRIOR10: PFIC_IPRIOR10_Type(loc: 0xe000e40a'u),
  IPRIOR11: PFIC_IPRIOR11_Type(loc: 0xe000e40b'u),
  IPRIOR12: PFIC_IPRIOR12_Type(loc: 0xe000e40c'u),
  IPRIOR13: PFIC_IPRIOR13_Type(loc: 0xe000e40d'u),
  IPRIOR14: PFIC_IPRIOR14_Type(loc: 0xe000e40e'u),
  IPRIOR15: PFIC_IPRIOR15_Type(loc: 0xe000e40f'u),
  IPRIOR16: PFIC_IPRIOR16_Type(loc: 0xe000e410'u),
  IPRIOR17: PFIC_IPRIOR17_Type(loc: 0xe000e411'u),
  IPRIOR18: PFIC_IPRIOR18_Type(loc: 0xe000e412'u),
  IPRIOR19: PFIC_IPRIOR19_Type(loc: 0xe000e413'u),
  IPRIOR20: PFIC_IPRIOR20_Type(loc: 0xe000e414'u),
  IPRIOR21: PFIC_IPRIOR21_Type(loc: 0xe000e415'u),
  IPRIOR22: PFIC_IPRIOR22_Type(loc: 0xe000e416'u),
  IPRIOR23: PFIC_IPRIOR23_Type(loc: 0xe000e417'u),
  IPRIOR24: PFIC_IPRIOR24_Type(loc: 0xe000e418'u),
  IPRIOR25: PFIC_IPRIOR25_Type(loc: 0xe000e419'u),
  IPRIOR26: PFIC_IPRIOR26_Type(loc: 0xe000e41a'u),
  IPRIOR27: PFIC_IPRIOR27_Type(loc: 0xe000e41b'u),
  IPRIOR28: PFIC_IPRIOR28_Type(loc: 0xe000e41c'u),
  IPRIOR29: PFIC_IPRIOR29_Type(loc: 0xe000e41d'u),
  IPRIOR30: PFIC_IPRIOR30_Type(loc: 0xe000e41e'u),
  IPRIOR31: PFIC_IPRIOR31_Type(loc: 0xe000e41f'u),
  IPRIOR32: PFIC_IPRIOR32_Type(loc: 0xe000e420'u),
  IPRIOR33: PFIC_IPRIOR33_Type(loc: 0xe000e421'u),
  IPRIOR34: PFIC_IPRIOR34_Type(loc: 0xe000e422'u),
  IPRIOR35: PFIC_IPRIOR35_Type(loc: 0xe000e423'u),
  IPRIOR36: PFIC_IPRIOR36_Type(loc: 0xe000e424'u),
  IPRIOR37: PFIC_IPRIOR37_Type(loc: 0xe000e425'u),
  IPRIOR38: PFIC_IPRIOR38_Type(loc: 0xe000e426'u),
  IPRIOR39: PFIC_IPRIOR39_Type(loc: 0xe000e427'u),
  IPRIOR40: PFIC_IPRIOR40_Type(loc: 0xe000e428'u),
  IPRIOR41: PFIC_IPRIOR41_Type(loc: 0xe000e429'u),
  IPRIOR42: PFIC_IPRIOR42_Type(loc: 0xe000e42a'u),
  IPRIOR43: PFIC_IPRIOR43_Type(loc: 0xe000e42b'u),
  IPRIOR44: PFIC_IPRIOR44_Type(loc: 0xe000e42c'u),
  IPRIOR45: PFIC_IPRIOR45_Type(loc: 0xe000e42d'u),
  IPRIOR46: PFIC_IPRIOR46_Type(loc: 0xe000e42e'u),
  IPRIOR47: PFIC_IPRIOR47_Type(loc: 0xe000e42f'u),
  IPRIOR48: PFIC_IPRIOR48_Type(loc: 0xe000e430'u),
  IPRIOR49: PFIC_IPRIOR49_Type(loc: 0xe000e431'u),
  IPRIOR50: PFIC_IPRIOR50_Type(loc: 0xe000e432'u),
  IPRIOR51: PFIC_IPRIOR51_Type(loc: 0xe000e433'u),
  IPRIOR52: PFIC_IPRIOR52_Type(loc: 0xe000e434'u),
  IPRIOR53: PFIC_IPRIOR53_Type(loc: 0xe000e435'u),
  IPRIOR54: PFIC_IPRIOR54_Type(loc: 0xe000e436'u),
  IPRIOR55: PFIC_IPRIOR55_Type(loc: 0xe000e437'u),
  IPRIOR56: PFIC_IPRIOR56_Type(loc: 0xe000e438'u),
  IPRIOR57: PFIC_IPRIOR57_Type(loc: 0xe000e439'u),
  IPRIOR58: PFIC_IPRIOR58_Type(loc: 0xe000e43a'u),
  IPRIOR59: PFIC_IPRIOR59_Type(loc: 0xe000e43b'u),
  IPRIOR60: PFIC_IPRIOR60_Type(loc: 0xe000e43c'u),
  IPRIOR61: PFIC_IPRIOR61_Type(loc: 0xe000e43d'u),
  IPRIOR62: PFIC_IPRIOR62_Type(loc: 0xe000e43e'u),
  IPRIOR63: PFIC_IPRIOR63_Type(loc: 0xe000e43f'u),
  SCTLR: PFIC_SCTLR_Type(loc: 0xe000ed10'u),
  STK_CTLR: PFIC_STK_CTLR_Type(loc: 0xe000f000'u),
  STK_SR: PFIC_STK_SR_Type(loc: 0xe000f004'u),
  STK_CNTL: PFIC_STK_CNTL_Type(loc: 0xe000f008'u),
  STK_CMPLR: PFIC_STK_CMPLR_Type(loc: 0xe000f010'u),
)


################################################################################
# Accessors for peripheral registers
################################################################################
type
  PWR_CTLR_Fields* = distinct uint32
  PWR_CSR_Fields* = distinct uint32
  PWR_AWUCSR_Fields* = distinct uint32
  PWR_AWUAPR_Fields* = distinct uint32
  PWR_AWUPSC_Fields* = distinct uint32

proc read*(reg: PWR_CTLR_Type): PWR_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_CTLR_Fields](reg.loc))

proc read*(reg: static PWR_CTLR_Type): PWR_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_CTLR_Fields](reg.loc))

proc write*(reg: PWR_CTLR_Type, val: PWR_CTLR_Fields) {.inline.} =
  volatileStore(cast[ptr PWR_CTLR_Fields](reg.loc), val)

proc write*(reg: PWR_CTLR_Type, PDDS: bool = false, PVDE: bool = false, PLS: uint32 = 0) =
  var x: uint32
  x.setMask((PDDS.uint32 shl 1).masked(1 .. 1))
  x.setMask((PVDE.uint32 shl 4).masked(4 .. 4))
  x.setMask((PLS shl 5).masked(5 .. 7))
  reg.write x.PWR_CTLR_Fields

template modifyIt*(reg: PWR_CTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PWR_CSR_Type): PWR_CSR_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_CSR_Fields](reg.loc))

proc read*(reg: static PWR_CSR_Type): PWR_CSR_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_CSR_Fields](reg.loc))

proc write*(reg: PWR_CSR_Type, val: PWR_CSR_Fields) {.inline.} =
  volatileStore(cast[ptr PWR_CSR_Fields](reg.loc), val)

template modifyIt*(reg: PWR_CSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PWR_AWUCSR_Type): PWR_AWUCSR_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_AWUCSR_Fields](reg.loc))

proc read*(reg: static PWR_AWUCSR_Type): PWR_AWUCSR_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_AWUCSR_Fields](reg.loc))

proc write*(reg: PWR_AWUCSR_Type, val: PWR_AWUCSR_Fields) {.inline.} =
  volatileStore(cast[ptr PWR_AWUCSR_Fields](reg.loc), val)

proc write*(reg: PWR_AWUCSR_Type, AWUEN: bool = false) =
  var x: uint32
  x.setMask((AWUEN.uint32 shl 1).masked(1 .. 1))
  reg.write x.PWR_AWUCSR_Fields

template modifyIt*(reg: PWR_AWUCSR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PWR_AWUAPR_Type): PWR_AWUAPR_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_AWUAPR_Fields](reg.loc))

proc read*(reg: static PWR_AWUAPR_Type): PWR_AWUAPR_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_AWUAPR_Fields](reg.loc))

proc write*(reg: PWR_AWUAPR_Type, val: PWR_AWUAPR_Fields) {.inline.} =
  volatileStore(cast[ptr PWR_AWUAPR_Fields](reg.loc), val)

proc write*(reg: PWR_AWUAPR_Type, AWUAPR: uint32 = 63) =
  var x: uint32
  x.setMask((AWUAPR shl 0).masked(0 .. 5))
  reg.write x.PWR_AWUAPR_Fields

template modifyIt*(reg: PWR_AWUAPR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PWR_AWUPSC_Type): PWR_AWUPSC_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_AWUPSC_Fields](reg.loc))

proc read*(reg: static PWR_AWUPSC_Type): PWR_AWUPSC_Fields {.inline.} =
  volatileLoad(cast[ptr PWR_AWUPSC_Fields](reg.loc))

proc write*(reg: PWR_AWUPSC_Type, val: PWR_AWUPSC_Fields) {.inline.} =
  volatileStore(cast[ptr PWR_AWUPSC_Fields](reg.loc), val)

proc write*(reg: PWR_AWUPSC_Type, AWUPSC: uint32 = 0) =
  var x: uint32
  x.setMask((AWUPSC shl 0).masked(0 .. 3))
  reg.write x.PWR_AWUPSC_Fields

template modifyIt*(reg: PWR_AWUPSC_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func PDDS*(r: PWR_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `PDDS=`*(r: var PWR_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.PWR_CTLR_Fields

func PVDE*(r: PWR_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `PVDE=`*(r: var PWR_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.PWR_CTLR_Fields

func PLS*(r: PWR_CTLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(5 .. 7)

proc `PLS=`*(r: var PWR_CTLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 7)
  tmp.setMask((val shl 5).masked(5 .. 7))
  r = tmp.PWR_CTLR_Fields

func PVDO*(r: PWR_CSR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func AWUEN*(r: PWR_AWUCSR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `AWUEN=`*(r: var PWR_AWUCSR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.PWR_AWUCSR_Fields

func AWUAPR*(r: PWR_AWUAPR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 5)

proc `AWUAPR=`*(r: var PWR_AWUAPR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 5)
  tmp.setMask((val shl 0).masked(0 .. 5))
  r = tmp.PWR_AWUAPR_Fields

func AWUPSC*(r: PWR_AWUPSC_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 3)

proc `AWUPSC=`*(r: var PWR_AWUPSC_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 3)
  tmp.setMask((val shl 0).masked(0 .. 3))
  r = tmp.PWR_AWUPSC_Fields

type
  RCC_CTLR_Fields* = distinct uint32
  RCC_CFGR0_Fields* = distinct uint32
  RCC_INTR_Fields* = distinct uint32
  RCC_APB2PRSTR_Fields* = distinct uint32
  RCC_APB1PRSTR_Fields* = distinct uint32
  RCC_AHBPCENR_Fields* = distinct uint32
  RCC_APB2PCENR_Fields* = distinct uint32
  RCC_APB1PCENR_Fields* = distinct uint32
  RCC_RSTSCKR_Fields* = distinct uint32

proc read*(reg: RCC_CTLR_Type): RCC_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_CTLR_Fields](reg.loc))

proc read*(reg: static RCC_CTLR_Type): RCC_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_CTLR_Fields](reg.loc))

proc write*(reg: RCC_CTLR_Type, val: RCC_CTLR_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_CTLR_Fields](reg.loc), val)

proc write*(reg: RCC_CTLR_Type, HSION: bool = true, HSITRIM: uint32 = 16, HSEON: bool = false, HSEBYP: bool = false, CSSON: bool = false, PLLON: bool = false) =
  var x: uint32
  x.setMask((HSION.uint32 shl 0).masked(0 .. 0))
  x.setMask((HSITRIM shl 3).masked(3 .. 7))
  x.setMask((HSEON.uint32 shl 16).masked(16 .. 16))
  x.setMask((HSEBYP.uint32 shl 18).masked(18 .. 18))
  x.setMask((CSSON.uint32 shl 19).masked(19 .. 19))
  x.setMask((PLLON.uint32 shl 24).masked(24 .. 24))
  reg.write x.RCC_CTLR_Fields

template modifyIt*(reg: RCC_CTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: RCC_CFGR0_Type): RCC_CFGR0_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_CFGR0_Fields](reg.loc))

proc read*(reg: static RCC_CFGR0_Type): RCC_CFGR0_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_CFGR0_Fields](reg.loc))

proc write*(reg: RCC_CFGR0_Type, val: RCC_CFGR0_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_CFGR0_Fields](reg.loc), val)

proc write*(reg: RCC_CFGR0_Type, SW: uint32 = 0, HPRE: uint32 = 0, PPRE1: uint32 = 0, PPRE2: uint32 = 0, ADCPRE: uint32 = 0, PLLSRC: bool = false, MCO: uint32 = 0) =
  var x: uint32
  x.setMask((SW shl 0).masked(0 .. 1))
  x.setMask((HPRE shl 4).masked(4 .. 7))
  x.setMask((PPRE1 shl 8).masked(8 .. 10))
  x.setMask((PPRE2 shl 11).masked(11 .. 13))
  x.setMask((ADCPRE shl 14).masked(14 .. 15))
  x.setMask((PLLSRC.uint32 shl 16).masked(16 .. 16))
  x.setMask((MCO shl 24).masked(24 .. 26))
  reg.write x.RCC_CFGR0_Fields

template modifyIt*(reg: RCC_CFGR0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: RCC_INTR_Type): RCC_INTR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_INTR_Fields](reg.loc))

proc read*(reg: static RCC_INTR_Type): RCC_INTR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_INTR_Fields](reg.loc))

proc write*(reg: RCC_INTR_Type, val: RCC_INTR_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_INTR_Fields](reg.loc), val)

proc write*(reg: RCC_INTR_Type, LSIRDYIE: bool = false, HSIRDYIE: bool = false, HSERDYIE: bool = false, PLLRDYIE: bool = false, LSIRDYC: bool = false, HSIRDYC: bool = false, HSERDYC: bool = false, PLLRDYC: bool = false, CSSC: bool = false) =
  var x: uint32
  x.setMask((LSIRDYIE.uint32 shl 8).masked(8 .. 8))
  x.setMask((HSIRDYIE.uint32 shl 10).masked(10 .. 10))
  x.setMask((HSERDYIE.uint32 shl 11).masked(11 .. 11))
  x.setMask((PLLRDYIE.uint32 shl 12).masked(12 .. 12))
  x.setMask((LSIRDYC.uint32 shl 16).masked(16 .. 16))
  x.setMask((HSIRDYC.uint32 shl 18).masked(18 .. 18))
  x.setMask((HSERDYC.uint32 shl 19).masked(19 .. 19))
  x.setMask((PLLRDYC.uint32 shl 20).masked(20 .. 20))
  x.setMask((CSSC.uint32 shl 23).masked(23 .. 23))
  reg.write x.RCC_INTR_Fields

# Add a write overload that accepts a uint32, converting it to RCC_INTR_Fields
proc write*(reg: RCC_INTR_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr RCC_INTR_Fields](reg.loc), val.RCC_INTR_Fields)

template modifyIt*(reg: RCC_INTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: RCC_APB2PRSTR_Type): RCC_APB2PRSTR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_APB2PRSTR_Fields](reg.loc))

proc read*(reg: static RCC_APB2PRSTR_Type): RCC_APB2PRSTR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_APB2PRSTR_Fields](reg.loc))

proc write*(reg: RCC_APB2PRSTR_Type, val: RCC_APB2PRSTR_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_APB2PRSTR_Fields](reg.loc), val)

proc write*(reg: RCC_APB2PRSTR_Type, AFIORST: bool = false, IOPARST: bool = false, IOPCRST: bool = false, IOPDRST: bool = false, ADC1RST: bool = false, TIM1RST: bool = false, SPI1RST: bool = false, USART1RST: bool = false) =
  var x: uint32
  x.setMask((AFIORST.uint32 shl 0).masked(0 .. 0))
  x.setMask((IOPARST.uint32 shl 2).masked(2 .. 2))
  x.setMask((IOPCRST.uint32 shl 4).masked(4 .. 4))
  x.setMask((IOPDRST.uint32 shl 5).masked(5 .. 5))
  x.setMask((ADC1RST.uint32 shl 9).masked(9 .. 9))
  x.setMask((TIM1RST.uint32 shl 11).masked(11 .. 11))
  x.setMask((SPI1RST.uint32 shl 12).masked(12 .. 12))
  x.setMask((USART1RST.uint32 shl 14).masked(14 .. 14))
  reg.write x.RCC_APB2PRSTR_Fields

template modifyIt*(reg: RCC_APB2PRSTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: RCC_APB1PRSTR_Type): RCC_APB1PRSTR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_APB1PRSTR_Fields](reg.loc))

proc read*(reg: static RCC_APB1PRSTR_Type): RCC_APB1PRSTR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_APB1PRSTR_Fields](reg.loc))

proc write*(reg: RCC_APB1PRSTR_Type, val: RCC_APB1PRSTR_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_APB1PRSTR_Fields](reg.loc), val)

proc write*(reg: RCC_APB1PRSTR_Type, WWDGRST: bool = false, I2C1RST: bool = false, PWRRST: bool = false) =
  var x: uint32
  x.setMask((WWDGRST.uint32 shl 11).masked(11 .. 11))
  x.setMask((I2C1RST.uint32 shl 21).masked(21 .. 21))
  x.setMask((PWRRST.uint32 shl 28).masked(28 .. 28))
  reg.write x.RCC_APB1PRSTR_Fields

template modifyIt*(reg: RCC_APB1PRSTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: RCC_AHBPCENR_Type): RCC_AHBPCENR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_AHBPCENR_Fields](reg.loc))

proc read*(reg: static RCC_AHBPCENR_Type): RCC_AHBPCENR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_AHBPCENR_Fields](reg.loc))

proc write*(reg: RCC_AHBPCENR_Type, val: RCC_AHBPCENR_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_AHBPCENR_Fields](reg.loc), val)

proc write*(reg: RCC_AHBPCENR_Type, DMA1EN: bool = false, SRAMEN: bool = true) =
  var x: uint32
  x.setMask((DMA1EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((SRAMEN.uint32 shl 2).masked(2 .. 2))
  reg.write x.RCC_AHBPCENR_Fields

template modifyIt*(reg: RCC_AHBPCENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: RCC_APB2PCENR_Type): RCC_APB2PCENR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_APB2PCENR_Fields](reg.loc))

proc read*(reg: static RCC_APB2PCENR_Type): RCC_APB2PCENR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_APB2PCENR_Fields](reg.loc))

proc write*(reg: RCC_APB2PCENR_Type, val: RCC_APB2PCENR_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_APB2PCENR_Fields](reg.loc), val)

proc write*(reg: RCC_APB2PCENR_Type, AFIOEN: bool = false, IOPAEN: bool = false, IOPCEN: bool = false, IOPDEN: bool = false, ADC1EN: bool = false, TIM1EN: bool = false, SPI1EN: bool = false, USART1EN: bool = false) =
  var x: uint32
  x.setMask((AFIOEN.uint32 shl 0).masked(0 .. 0))
  x.setMask((IOPAEN.uint32 shl 2).masked(2 .. 2))
  x.setMask((IOPCEN.uint32 shl 4).masked(4 .. 4))
  x.setMask((IOPDEN.uint32 shl 5).masked(5 .. 5))
  x.setMask((ADC1EN.uint32 shl 9).masked(9 .. 9))
  x.setMask((TIM1EN.uint32 shl 11).masked(11 .. 11))
  x.setMask((SPI1EN.uint32 shl 12).masked(12 .. 12))
  x.setMask((USART1EN.uint32 shl 14).masked(14 .. 14))
  reg.write x.RCC_APB2PCENR_Fields

template modifyIt*(reg: RCC_APB2PCENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: RCC_APB1PCENR_Type): RCC_APB1PCENR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_APB1PCENR_Fields](reg.loc))

proc read*(reg: static RCC_APB1PCENR_Type): RCC_APB1PCENR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_APB1PCENR_Fields](reg.loc))

proc write*(reg: RCC_APB1PCENR_Type, val: RCC_APB1PCENR_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_APB1PCENR_Fields](reg.loc), val)

proc write*(reg: RCC_APB1PCENR_Type, TIM2EN: bool = false, WWDGEN: bool = false, I2C1EN: bool = false, PWREN: bool = false) =
  var x: uint32
  x.setMask((TIM2EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((WWDGEN.uint32 shl 11).masked(11 .. 11))
  x.setMask((I2C1EN.uint32 shl 21).masked(21 .. 21))
  x.setMask((PWREN.uint32 shl 28).masked(28 .. 28))
  reg.write x.RCC_APB1PCENR_Fields

template modifyIt*(reg: RCC_APB1PCENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: RCC_RSTSCKR_Type): RCC_RSTSCKR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_RSTSCKR_Fields](reg.loc))

proc read*(reg: static RCC_RSTSCKR_Type): RCC_RSTSCKR_Fields {.inline.} =
  volatileLoad(cast[ptr RCC_RSTSCKR_Fields](reg.loc))

proc write*(reg: RCC_RSTSCKR_Type, val: RCC_RSTSCKR_Fields) {.inline.} =
  volatileStore(cast[ptr RCC_RSTSCKR_Fields](reg.loc), val)

proc write*(reg: RCC_RSTSCKR_Type, LSION: bool = false, RMVF: bool = false) =
  var x: uint32
  x.setMask((LSION.uint32 shl 0).masked(0 .. 0))
  x.setMask((RMVF.uint32 shl 24).masked(24 .. 24))
  reg.write x.RCC_RSTSCKR_Fields

template modifyIt*(reg: RCC_RSTSCKR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func HSION*(r: RCC_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `HSION=`*(r: var RCC_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.RCC_CTLR_Fields

func HSIRDY*(r: RCC_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func HSITRIM*(r: RCC_CTLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(3 .. 7)

proc `HSITRIM=`*(r: var RCC_CTLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 7)
  tmp.setMask((val shl 3).masked(3 .. 7))
  r = tmp.RCC_CTLR_Fields

func HSICAL*(r: RCC_CTLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 15)

func HSEON*(r: RCC_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(16 .. 16).bool

proc `HSEON=`*(r: var RCC_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 16)
  tmp.setMask((val.uint32 shl 16).masked(16 .. 16))
  r = tmp.RCC_CTLR_Fields

func HSERDY*(r: RCC_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(17 .. 17).bool

func HSEBYP*(r: RCC_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(18 .. 18).bool

proc `HSEBYP=`*(r: var RCC_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 18)
  tmp.setMask((val.uint32 shl 18).masked(18 .. 18))
  r = tmp.RCC_CTLR_Fields

func CSSON*(r: RCC_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(19 .. 19).bool

proc `CSSON=`*(r: var RCC_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(19 .. 19)
  tmp.setMask((val.uint32 shl 19).masked(19 .. 19))
  r = tmp.RCC_CTLR_Fields

func PLLON*(r: RCC_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(24 .. 24).bool

proc `PLLON=`*(r: var RCC_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(24 .. 24)
  tmp.setMask((val.uint32 shl 24).masked(24 .. 24))
  r = tmp.RCC_CTLR_Fields

func PLLRDY*(r: RCC_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(25 .. 25).bool

func SW*(r: RCC_CFGR0_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `SW=`*(r: var RCC_CFGR0_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.RCC_CFGR0_Fields

func SWS*(r: RCC_CFGR0_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

func HPRE*(r: RCC_CFGR0_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 7)

proc `HPRE=`*(r: var RCC_CFGR0_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 7)
  tmp.setMask((val shl 4).masked(4 .. 7))
  r = tmp.RCC_CFGR0_Fields

func PPRE1*(r: RCC_CFGR0_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 10)

proc `PPRE1=`*(r: var RCC_CFGR0_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 10)
  tmp.setMask((val shl 8).masked(8 .. 10))
  r = tmp.RCC_CFGR0_Fields

func PPRE2*(r: RCC_CFGR0_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(11 .. 13)

proc `PPRE2=`*(r: var RCC_CFGR0_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 13)
  tmp.setMask((val shl 11).masked(11 .. 13))
  r = tmp.RCC_CFGR0_Fields

func ADCPRE*(r: RCC_CFGR0_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(14 .. 15)

proc `ADCPRE=`*(r: var RCC_CFGR0_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 15)
  tmp.setMask((val shl 14).masked(14 .. 15))
  r = tmp.RCC_CFGR0_Fields

func PLLSRC*(r: RCC_CFGR0_Fields): bool {.inline.} =
  r.uint32.bitsliced(16 .. 16).bool

proc `PLLSRC=`*(r: var RCC_CFGR0_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 16)
  tmp.setMask((val.uint32 shl 16).masked(16 .. 16))
  r = tmp.RCC_CFGR0_Fields

func MCO*(r: RCC_CFGR0_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(24 .. 26)

proc `MCO=`*(r: var RCC_CFGR0_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(24 .. 26)
  tmp.setMask((val shl 24).masked(24 .. 26))
  r = tmp.RCC_CFGR0_Fields

func LSIRDYF*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func HSIRDYF*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func HSERDYF*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

func PLLRDYF*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

func CSSF*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

func LSIRDYIE*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `LSIRDYIE=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.RCC_INTR_Fields

func HSIRDYIE*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `HSIRDYIE=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.RCC_INTR_Fields

func HSERDYIE*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `HSERDYIE=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.RCC_INTR_Fields

func PLLRDYIE*(r: RCC_INTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `PLLRDYIE=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.RCC_INTR_Fields

proc `LSIRDYC=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 16)
  tmp.setMask((val.uint32 shl 16).masked(16 .. 16))
  r = tmp.RCC_INTR_Fields

proc `HSIRDYC=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 18)
  tmp.setMask((val.uint32 shl 18).masked(18 .. 18))
  r = tmp.RCC_INTR_Fields

proc `HSERDYC=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(19 .. 19)
  tmp.setMask((val.uint32 shl 19).masked(19 .. 19))
  r = tmp.RCC_INTR_Fields

proc `PLLRDYC=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 20)
  tmp.setMask((val.uint32 shl 20).masked(20 .. 20))
  r = tmp.RCC_INTR_Fields

proc `CSSC=`*(r: var RCC_INTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(23 .. 23)
  tmp.setMask((val.uint32 shl 23).masked(23 .. 23))
  r = tmp.RCC_INTR_Fields

func AFIORST*(r: RCC_APB2PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `AFIORST=`*(r: var RCC_APB2PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.RCC_APB2PRSTR_Fields

func IOPARST*(r: RCC_APB2PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `IOPARST=`*(r: var RCC_APB2PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.RCC_APB2PRSTR_Fields

func IOPCRST*(r: RCC_APB2PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `IOPCRST=`*(r: var RCC_APB2PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.RCC_APB2PRSTR_Fields

func IOPDRST*(r: RCC_APB2PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `IOPDRST=`*(r: var RCC_APB2PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.RCC_APB2PRSTR_Fields

func ADC1RST*(r: RCC_APB2PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `ADC1RST=`*(r: var RCC_APB2PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.RCC_APB2PRSTR_Fields

func TIM1RST*(r: RCC_APB2PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `TIM1RST=`*(r: var RCC_APB2PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.RCC_APB2PRSTR_Fields

func SPI1RST*(r: RCC_APB2PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `SPI1RST=`*(r: var RCC_APB2PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.RCC_APB2PRSTR_Fields

func USART1RST*(r: RCC_APB2PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `USART1RST=`*(r: var RCC_APB2PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.RCC_APB2PRSTR_Fields

func WWDGRST*(r: RCC_APB1PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `WWDGRST=`*(r: var RCC_APB1PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.RCC_APB1PRSTR_Fields

func I2C1RST*(r: RCC_APB1PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(21 .. 21).bool

proc `I2C1RST=`*(r: var RCC_APB1PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(21 .. 21)
  tmp.setMask((val.uint32 shl 21).masked(21 .. 21))
  r = tmp.RCC_APB1PRSTR_Fields

func PWRRST*(r: RCC_APB1PRSTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(28 .. 28).bool

proc `PWRRST=`*(r: var RCC_APB1PRSTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(28 .. 28)
  tmp.setMask((val.uint32 shl 28).masked(28 .. 28))
  r = tmp.RCC_APB1PRSTR_Fields

func DMA1EN*(r: RCC_AHBPCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `DMA1EN=`*(r: var RCC_AHBPCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.RCC_AHBPCENR_Fields

func SRAMEN*(r: RCC_AHBPCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `SRAMEN=`*(r: var RCC_AHBPCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.RCC_AHBPCENR_Fields

func AFIOEN*(r: RCC_APB2PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `AFIOEN=`*(r: var RCC_APB2PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.RCC_APB2PCENR_Fields

func IOPAEN*(r: RCC_APB2PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `IOPAEN=`*(r: var RCC_APB2PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.RCC_APB2PCENR_Fields

func IOPCEN*(r: RCC_APB2PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `IOPCEN=`*(r: var RCC_APB2PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.RCC_APB2PCENR_Fields

func IOPDEN*(r: RCC_APB2PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `IOPDEN=`*(r: var RCC_APB2PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.RCC_APB2PCENR_Fields

func ADC1EN*(r: RCC_APB2PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `ADC1EN=`*(r: var RCC_APB2PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.RCC_APB2PCENR_Fields

func TIM1EN*(r: RCC_APB2PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `TIM1EN=`*(r: var RCC_APB2PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.RCC_APB2PCENR_Fields

func SPI1EN*(r: RCC_APB2PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `SPI1EN=`*(r: var RCC_APB2PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.RCC_APB2PCENR_Fields

func USART1EN*(r: RCC_APB2PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `USART1EN=`*(r: var RCC_APB2PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.RCC_APB2PCENR_Fields

func TIM2EN*(r: RCC_APB1PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `TIM2EN=`*(r: var RCC_APB1PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.RCC_APB1PCENR_Fields

func WWDGEN*(r: RCC_APB1PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `WWDGEN=`*(r: var RCC_APB1PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.RCC_APB1PCENR_Fields

func I2C1EN*(r: RCC_APB1PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(21 .. 21).bool

proc `I2C1EN=`*(r: var RCC_APB1PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(21 .. 21)
  tmp.setMask((val.uint32 shl 21).masked(21 .. 21))
  r = tmp.RCC_APB1PCENR_Fields

func PWREN*(r: RCC_APB1PCENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(28 .. 28).bool

proc `PWREN=`*(r: var RCC_APB1PCENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(28 .. 28)
  tmp.setMask((val.uint32 shl 28).masked(28 .. 28))
  r = tmp.RCC_APB1PCENR_Fields

func LSION*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `LSION=`*(r: var RCC_RSTSCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.RCC_RSTSCKR_Fields

func LSIRDY*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func RMVF*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(24 .. 24).bool

proc `RMVF=`*(r: var RCC_RSTSCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(24 .. 24)
  tmp.setMask((val.uint32 shl 24).masked(24 .. 24))
  r = tmp.RCC_RSTSCKR_Fields

func PINRSTF*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(26 .. 26).bool

func PORRSTF*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(27 .. 27).bool

func SFTRSTF*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(28 .. 28).bool

func IWDGRSTF*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(29 .. 29).bool

func WWDGRSTF*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(30 .. 30).bool

func LPWRRSTF*(r: RCC_RSTSCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(31 .. 31).bool

type
  EXTEND_EXTEND_CTR_Fields* = distinct uint32

proc read*(reg: EXTEND_EXTEND_CTR_Type): EXTEND_EXTEND_CTR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTEND_EXTEND_CTR_Fields](reg.loc))

proc read*(reg: static EXTEND_EXTEND_CTR_Type): EXTEND_EXTEND_CTR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTEND_EXTEND_CTR_Fields](reg.loc))

proc write*(reg: EXTEND_EXTEND_CTR_Type, val: EXTEND_EXTEND_CTR_Fields) {.inline.} =
  volatileStore(cast[ptr EXTEND_EXTEND_CTR_Fields](reg.loc), val)

proc write*(reg: EXTEND_EXTEND_CTR_Type, PLL_CFG: uint32 = 0, LOCKUP_EN: bool = true, LOCKUP_RESET: bool = false, LDO_TRIM: bool = false, FLASH_CLK_TRIM: uint32 = 0, WR_EN: bool = false, WR_LOCK: bool = false, OPA_EN: bool = false, OPA_NSEL: bool = false, OPA_PSEL: bool = false) =
  var x: uint32
  x.setMask((PLL_CFG shl 0).masked(0 .. 3))
  x.setMask((LOCKUP_EN.uint32 shl 6).masked(6 .. 6))
  x.setMask((LOCKUP_RESET.uint32 shl 7).masked(7 .. 7))
  x.setMask((LDO_TRIM.uint32 shl 10).masked(10 .. 10))
  x.setMask((FLASH_CLK_TRIM shl 11).masked(11 .. 13))
  x.setMask((WR_EN.uint32 shl 14).masked(14 .. 14))
  x.setMask((WR_LOCK.uint32 shl 15).masked(15 .. 15))
  x.setMask((OPA_EN.uint32 shl 16).masked(16 .. 16))
  x.setMask((OPA_NSEL.uint32 shl 17).masked(17 .. 17))
  x.setMask((OPA_PSEL.uint32 shl 18).masked(18 .. 18))
  reg.write x.EXTEND_EXTEND_CTR_Fields

template modifyIt*(reg: EXTEND_EXTEND_CTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: EXTEND_EXTEND_KR_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static EXTEND_EXTEND_KR_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: EXTEND_EXTEND_KR_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: EXTEND_EXTEND_KR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func PLL_CFG*(r: EXTEND_EXTEND_CTR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 3)

proc `PLL_CFG=`*(r: var EXTEND_EXTEND_CTR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 3)
  tmp.setMask((val shl 0).masked(0 .. 3))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func LOCKUP_EN*(r: EXTEND_EXTEND_CTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `LOCKUP_EN=`*(r: var EXTEND_EXTEND_CTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func LOCKUP_RESET*(r: EXTEND_EXTEND_CTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `LOCKUP_RESET=`*(r: var EXTEND_EXTEND_CTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func LDO_TRIM*(r: EXTEND_EXTEND_CTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `LDO_TRIM=`*(r: var EXTEND_EXTEND_CTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func FLASH_CLK_TRIM*(r: EXTEND_EXTEND_CTR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(11 .. 13)

proc `FLASH_CLK_TRIM=`*(r: var EXTEND_EXTEND_CTR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 13)
  tmp.setMask((val shl 11).masked(11 .. 13))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func WR_EN*(r: EXTEND_EXTEND_CTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `WR_EN=`*(r: var EXTEND_EXTEND_CTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func WR_LOCK*(r: EXTEND_EXTEND_CTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `WR_LOCK=`*(r: var EXTEND_EXTEND_CTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func OPA_EN*(r: EXTEND_EXTEND_CTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(16 .. 16).bool

proc `OPA_EN=`*(r: var EXTEND_EXTEND_CTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 16)
  tmp.setMask((val.uint32 shl 16).masked(16 .. 16))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func OPA_NSEL*(r: EXTEND_EXTEND_CTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(17 .. 17).bool

proc `OPA_NSEL=`*(r: var EXTEND_EXTEND_CTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(17 .. 17)
  tmp.setMask((val.uint32 shl 17).masked(17 .. 17))
  r = tmp.EXTEND_EXTEND_CTR_Fields

func OPA_PSEL*(r: EXTEND_EXTEND_CTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(18 .. 18).bool

proc `OPA_PSEL=`*(r: var EXTEND_EXTEND_CTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 18)
  tmp.setMask((val.uint32 shl 18).masked(18 .. 18))
  r = tmp.EXTEND_EXTEND_CTR_Fields

type
  GPIOA_CFGLR_Fields* = distinct uint32
  GPIOA_INDR_Fields* = distinct uint32
  GPIOA_OUTDR_Fields* = distinct uint32
  GPIOA_BSHR_Fields* = distinct uint32
  GPIOA_BCR_Fields* = distinct uint32
  GPIOA_LCKR_Fields* = distinct uint32

proc read*(reg: GPIOA_CFGLR_Type): GPIOA_CFGLR_Fields {.inline.} =
  volatileLoad(cast[ptr GPIOA_CFGLR_Fields](reg.loc))

proc read*(reg: static GPIOA_CFGLR_Type): GPIOA_CFGLR_Fields {.inline.} =
  volatileLoad(cast[ptr GPIOA_CFGLR_Fields](reg.loc))

proc write*(reg: GPIOA_CFGLR_Type, val: GPIOA_CFGLR_Fields) {.inline.} =
  volatileStore(cast[ptr GPIOA_CFGLR_Fields](reg.loc), val)

proc write*(reg: GPIOA_CFGLR_Type, MODE0: uint32 = 0, CNF0: uint32 = 1, MODE1: uint32 = 0, CNF1: uint32 = 1, MODE2: uint32 = 0, CNF2: uint32 = 1, MODE3: uint32 = 0, CNF3: uint32 = 1, MODE4: uint32 = 0, CNF4: uint32 = 1, MODE5: uint32 = 0, CNF5: uint32 = 1, MODE6: uint32 = 0, CNF6: uint32 = 1, MODE7: uint32 = 0, CNF7: uint32 = 1) =
  var x: uint32
  x.setMask((MODE0 shl 0).masked(0 .. 1))
  x.setMask((CNF0 shl 2).masked(2 .. 3))
  x.setMask((MODE1 shl 4).masked(4 .. 5))
  x.setMask((CNF1 shl 6).masked(6 .. 7))
  x.setMask((MODE2 shl 8).masked(8 .. 9))
  x.setMask((CNF2 shl 10).masked(10 .. 11))
  x.setMask((MODE3 shl 12).masked(12 .. 13))
  x.setMask((CNF3 shl 14).masked(14 .. 15))
  x.setMask((MODE4 shl 16).masked(16 .. 17))
  x.setMask((CNF4 shl 18).masked(18 .. 19))
  x.setMask((MODE5 shl 20).masked(20 .. 21))
  x.setMask((CNF5 shl 22).masked(22 .. 23))
  x.setMask((MODE6 shl 24).masked(24 .. 25))
  x.setMask((CNF6 shl 26).masked(26 .. 27))
  x.setMask((MODE7 shl 28).masked(28 .. 29))
  x.setMask((CNF7 shl 30).masked(30 .. 31))
  reg.write x.GPIOA_CFGLR_Fields

template modifyIt*(reg: GPIOA_CFGLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: GPIOA_INDR_Type): GPIOA_INDR_Fields {.inline.} =
  volatileLoad(cast[ptr GPIOA_INDR_Fields](reg.loc))

proc read*(reg: static GPIOA_INDR_Type): GPIOA_INDR_Fields {.inline.} =
  volatileLoad(cast[ptr GPIOA_INDR_Fields](reg.loc))

proc read*(reg: GPIOA_OUTDR_Type): GPIOA_OUTDR_Fields {.inline.} =
  volatileLoad(cast[ptr GPIOA_OUTDR_Fields](reg.loc))

proc read*(reg: static GPIOA_OUTDR_Type): GPIOA_OUTDR_Fields {.inline.} =
  volatileLoad(cast[ptr GPIOA_OUTDR_Fields](reg.loc))

proc write*(reg: GPIOA_OUTDR_Type, val: GPIOA_OUTDR_Fields) {.inline.} =
  volatileStore(cast[ptr GPIOA_OUTDR_Fields](reg.loc), val)

proc write*(reg: GPIOA_OUTDR_Type, ODR0: bool = false, ODR1: bool = false, ODR2: bool = false, ODR3: bool = false, ODR4: bool = false, ODR5: bool = false, ODR6: bool = false, ODR7: bool = false) =
  var x: uint32
  x.setMask((ODR0.uint32 shl 0).masked(0 .. 0))
  x.setMask((ODR1.uint32 shl 1).masked(1 .. 1))
  x.setMask((ODR2.uint32 shl 2).masked(2 .. 2))
  x.setMask((ODR3.uint32 shl 3).masked(3 .. 3))
  x.setMask((ODR4.uint32 shl 4).masked(4 .. 4))
  x.setMask((ODR5.uint32 shl 5).masked(5 .. 5))
  x.setMask((ODR6.uint32 shl 6).masked(6 .. 6))
  x.setMask((ODR7.uint32 shl 7).masked(7 .. 7))
  reg.write x.GPIOA_OUTDR_Fields

template modifyIt*(reg: GPIOA_OUTDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc write*(reg: GPIOA_BSHR_Type, val: GPIOA_BSHR_Fields) {.inline.} =
  volatileStore(cast[ptr GPIOA_BSHR_Fields](reg.loc), val)

proc write*(reg: GPIOA_BSHR_Type, BS0: bool = false, BS1: bool = false, BS2: bool = false, BS3: bool = false, BS4: bool = false, BS5: bool = false, BS6: bool = false, BS7: bool = false, BR0: bool = false, BR1: bool = false, BR2: bool = false, BR3: bool = false, BR4: bool = false, BR5: bool = false, BR6: bool = false, BR7: bool = false) =
  var x: uint32
  x.setMask((BS0.uint32 shl 0).masked(0 .. 0))
  x.setMask((BS1.uint32 shl 1).masked(1 .. 1))
  x.setMask((BS2.uint32 shl 2).masked(2 .. 2))
  x.setMask((BS3.uint32 shl 3).masked(3 .. 3))
  x.setMask((BS4.uint32 shl 4).masked(4 .. 4))
  x.setMask((BS5.uint32 shl 5).masked(5 .. 5))
  x.setMask((BS6.uint32 shl 6).masked(6 .. 6))
  x.setMask((BS7.uint32 shl 7).masked(7 .. 7))
  x.setMask((BR0.uint32 shl 16).masked(16 .. 16))
  x.setMask((BR1.uint32 shl 17).masked(17 .. 17))
  x.setMask((BR2.uint32 shl 18).masked(18 .. 18))
  x.setMask((BR3.uint32 shl 19).masked(19 .. 19))
  x.setMask((BR4.uint32 shl 20).masked(20 .. 20))
  x.setMask((BR5.uint32 shl 21).masked(21 .. 21))
  x.setMask((BR6.uint32 shl 22).masked(22 .. 22))
  x.setMask((BR7.uint32 shl 23).masked(23 .. 23))
  reg.write x.GPIOA_BSHR_Fields

proc write*(reg: GPIOA_BCR_Type, val: GPIOA_BCR_Fields) {.inline.} =
  volatileStore(cast[ptr GPIOA_BCR_Fields](reg.loc), val)

proc write*(reg: GPIOA_BCR_Type, BR0: bool = false, BR1: bool = false, BR2: bool = false, BR3: bool = false, BR4: bool = false, BR5: bool = false, BR6: bool = false, BR7: bool = false) =
  var x: uint32
  x.setMask((BR0.uint32 shl 0).masked(0 .. 0))
  x.setMask((BR1.uint32 shl 1).masked(1 .. 1))
  x.setMask((BR2.uint32 shl 2).masked(2 .. 2))
  x.setMask((BR3.uint32 shl 3).masked(3 .. 3))
  x.setMask((BR4.uint32 shl 4).masked(4 .. 4))
  x.setMask((BR5.uint32 shl 5).masked(5 .. 5))
  x.setMask((BR6.uint32 shl 6).masked(6 .. 6))
  x.setMask((BR7.uint32 shl 7).masked(7 .. 7))
  reg.write x.GPIOA_BCR_Fields

proc read*(reg: GPIOA_LCKR_Type): GPIOA_LCKR_Fields {.inline.} =
  volatileLoad(cast[ptr GPIOA_LCKR_Fields](reg.loc))

proc read*(reg: static GPIOA_LCKR_Type): GPIOA_LCKR_Fields {.inline.} =
  volatileLoad(cast[ptr GPIOA_LCKR_Fields](reg.loc))

proc write*(reg: GPIOA_LCKR_Type, val: GPIOA_LCKR_Fields) {.inline.} =
  volatileStore(cast[ptr GPIOA_LCKR_Fields](reg.loc), val)

proc write*(reg: GPIOA_LCKR_Type, LCK0: bool = false, LCK1: bool = false, LCK2: bool = false, LCK3: bool = false, LCK4: bool = false, LCK5: bool = false, LCK6: bool = false, LCK7: bool = false, LCKK: bool = false) =
  var x: uint32
  x.setMask((LCK0.uint32 shl 0).masked(0 .. 0))
  x.setMask((LCK1.uint32 shl 1).masked(1 .. 1))
  x.setMask((LCK2.uint32 shl 2).masked(2 .. 2))
  x.setMask((LCK3.uint32 shl 3).masked(3 .. 3))
  x.setMask((LCK4.uint32 shl 4).masked(4 .. 4))
  x.setMask((LCK5.uint32 shl 5).masked(5 .. 5))
  x.setMask((LCK6.uint32 shl 6).masked(6 .. 6))
  x.setMask((LCK7.uint32 shl 7).masked(7 .. 7))
  x.setMask((LCKK.uint32 shl 8).masked(8 .. 8))
  reg.write x.GPIOA_LCKR_Fields

template modifyIt*(reg: GPIOA_LCKR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func MODE0*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `MODE0=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.GPIOA_CFGLR_Fields

func CNF0*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

proc `CNF0=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.GPIOA_CFGLR_Fields

func MODE1*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 5)

proc `MODE1=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 5)
  tmp.setMask((val shl 4).masked(4 .. 5))
  r = tmp.GPIOA_CFGLR_Fields

func CNF1*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(6 .. 7)

proc `CNF1=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 7)
  tmp.setMask((val shl 6).masked(6 .. 7))
  r = tmp.GPIOA_CFGLR_Fields

func MODE2*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `MODE2=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.GPIOA_CFGLR_Fields

func CNF2*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `CNF2=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.GPIOA_CFGLR_Fields

func MODE3*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `MODE3=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.GPIOA_CFGLR_Fields

func CNF3*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(14 .. 15)

proc `CNF3=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 15)
  tmp.setMask((val shl 14).masked(14 .. 15))
  r = tmp.GPIOA_CFGLR_Fields

func MODE4*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(16 .. 17)

proc `MODE4=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 17)
  tmp.setMask((val shl 16).masked(16 .. 17))
  r = tmp.GPIOA_CFGLR_Fields

func CNF4*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(18 .. 19)

proc `CNF4=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 19)
  tmp.setMask((val shl 18).masked(18 .. 19))
  r = tmp.GPIOA_CFGLR_Fields

func MODE5*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(20 .. 21)

proc `MODE5=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 21)
  tmp.setMask((val shl 20).masked(20 .. 21))
  r = tmp.GPIOA_CFGLR_Fields

func CNF5*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(22 .. 23)

proc `CNF5=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(22 .. 23)
  tmp.setMask((val shl 22).masked(22 .. 23))
  r = tmp.GPIOA_CFGLR_Fields

func MODE6*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(24 .. 25)

proc `MODE6=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(24 .. 25)
  tmp.setMask((val shl 24).masked(24 .. 25))
  r = tmp.GPIOA_CFGLR_Fields

func CNF6*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(26 .. 27)

proc `CNF6=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(26 .. 27)
  tmp.setMask((val shl 26).masked(26 .. 27))
  r = tmp.GPIOA_CFGLR_Fields

func MODE7*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(28 .. 29)

proc `MODE7=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(28 .. 29)
  tmp.setMask((val shl 28).masked(28 .. 29))
  r = tmp.GPIOA_CFGLR_Fields

func CNF7*(r: GPIOA_CFGLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(30 .. 31)

proc `CNF7=`*(r: var GPIOA_CFGLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(30 .. 31)
  tmp.setMask((val shl 30).masked(30 .. 31))
  r = tmp.GPIOA_CFGLR_Fields

func IDR0*(r: GPIOA_INDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func IDR1*(r: GPIOA_INDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func IDR2*(r: GPIOA_INDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func IDR3*(r: GPIOA_INDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

func IDR4*(r: GPIOA_INDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

func IDR5*(r: GPIOA_INDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

func IDR6*(r: GPIOA_INDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

func IDR7*(r: GPIOA_INDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

func ODR0*(r: GPIOA_OUTDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `ODR0=`*(r: var GPIOA_OUTDR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.GPIOA_OUTDR_Fields

func ODR1*(r: GPIOA_OUTDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `ODR1=`*(r: var GPIOA_OUTDR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.GPIOA_OUTDR_Fields

func ODR2*(r: GPIOA_OUTDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `ODR2=`*(r: var GPIOA_OUTDR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.GPIOA_OUTDR_Fields

func ODR3*(r: GPIOA_OUTDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `ODR3=`*(r: var GPIOA_OUTDR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.GPIOA_OUTDR_Fields

func ODR4*(r: GPIOA_OUTDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `ODR4=`*(r: var GPIOA_OUTDR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.GPIOA_OUTDR_Fields

func ODR5*(r: GPIOA_OUTDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `ODR5=`*(r: var GPIOA_OUTDR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.GPIOA_OUTDR_Fields

func ODR6*(r: GPIOA_OUTDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `ODR6=`*(r: var GPIOA_OUTDR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.GPIOA_OUTDR_Fields

func ODR7*(r: GPIOA_OUTDR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `ODR7=`*(r: var GPIOA_OUTDR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.GPIOA_OUTDR_Fields

proc `BS0=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.GPIOA_BSHR_Fields

proc `BS1=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.GPIOA_BSHR_Fields

proc `BS2=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.GPIOA_BSHR_Fields

proc `BS3=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.GPIOA_BSHR_Fields

proc `BS4=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.GPIOA_BSHR_Fields

proc `BS5=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.GPIOA_BSHR_Fields

proc `BS6=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.GPIOA_BSHR_Fields

proc `BS7=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.GPIOA_BSHR_Fields

proc `BR0=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 16)
  tmp.setMask((val.uint32 shl 16).masked(16 .. 16))
  r = tmp.GPIOA_BSHR_Fields

proc `BR1=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(17 .. 17)
  tmp.setMask((val.uint32 shl 17).masked(17 .. 17))
  r = tmp.GPIOA_BSHR_Fields

proc `BR2=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 18)
  tmp.setMask((val.uint32 shl 18).masked(18 .. 18))
  r = tmp.GPIOA_BSHR_Fields

proc `BR3=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(19 .. 19)
  tmp.setMask((val.uint32 shl 19).masked(19 .. 19))
  r = tmp.GPIOA_BSHR_Fields

proc `BR4=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 20)
  tmp.setMask((val.uint32 shl 20).masked(20 .. 20))
  r = tmp.GPIOA_BSHR_Fields

proc `BR5=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(21 .. 21)
  tmp.setMask((val.uint32 shl 21).masked(21 .. 21))
  r = tmp.GPIOA_BSHR_Fields

proc `BR6=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(22 .. 22)
  tmp.setMask((val.uint32 shl 22).masked(22 .. 22))
  r = tmp.GPIOA_BSHR_Fields

proc `BR7=`*(r: var GPIOA_BSHR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(23 .. 23)
  tmp.setMask((val.uint32 shl 23).masked(23 .. 23))
  r = tmp.GPIOA_BSHR_Fields

proc `BR0=`*(r: var GPIOA_BCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.GPIOA_BCR_Fields

proc `BR1=`*(r: var GPIOA_BCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.GPIOA_BCR_Fields

proc `BR2=`*(r: var GPIOA_BCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.GPIOA_BCR_Fields

proc `BR3=`*(r: var GPIOA_BCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.GPIOA_BCR_Fields

proc `BR4=`*(r: var GPIOA_BCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.GPIOA_BCR_Fields

proc `BR5=`*(r: var GPIOA_BCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.GPIOA_BCR_Fields

proc `BR6=`*(r: var GPIOA_BCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.GPIOA_BCR_Fields

proc `BR7=`*(r: var GPIOA_BCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.GPIOA_BCR_Fields

func LCK0*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `LCK0=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.GPIOA_LCKR_Fields

func LCK1*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `LCK1=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.GPIOA_LCKR_Fields

func LCK2*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `LCK2=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.GPIOA_LCKR_Fields

func LCK3*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `LCK3=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.GPIOA_LCKR_Fields

func LCK4*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `LCK4=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.GPIOA_LCKR_Fields

func LCK5*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `LCK5=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.GPIOA_LCKR_Fields

func LCK6*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `LCK6=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.GPIOA_LCKR_Fields

func LCK7*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `LCK7=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.GPIOA_LCKR_Fields

func LCKK*(r: GPIOA_LCKR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `LCKK=`*(r: var GPIOA_LCKR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.GPIOA_LCKR_Fields

type
  AFIO_PCFR_Fields* = distinct uint32
  AFIO_EXTICR_Fields* = distinct uint32

proc read*(reg: AFIO_PCFR_Type): AFIO_PCFR_Fields {.inline.} =
  volatileLoad(cast[ptr AFIO_PCFR_Fields](reg.loc))

proc read*(reg: static AFIO_PCFR_Type): AFIO_PCFR_Fields {.inline.} =
  volatileLoad(cast[ptr AFIO_PCFR_Fields](reg.loc))

proc write*(reg: AFIO_PCFR_Type, val: AFIO_PCFR_Fields) {.inline.} =
  volatileStore(cast[ptr AFIO_PCFR_Fields](reg.loc), val)

proc write*(reg: AFIO_PCFR_Type, SPI1RM: bool = false, I2C1RM: bool = false, USART1RM: bool = false, TIM1RM: uint32 = 0, TIM2RM: uint32 = 0, PA12RM: bool = false, ADC1_ETRGINJ_RM: bool = false, ADC1_ETRGREG_RM: bool = false, USART1REMAP1: bool = false, I2C1REMAP1: bool = false, TIM1_IREMAP: bool = false, SWCFG: uint32 = 0) =
  var x: uint32
  x.setMask((SPI1RM.uint32 shl 0).masked(0 .. 0))
  x.setMask((I2C1RM.uint32 shl 1).masked(1 .. 1))
  x.setMask((USART1RM.uint32 shl 2).masked(2 .. 2))
  x.setMask((TIM1RM shl 6).masked(6 .. 7))
  x.setMask((TIM2RM shl 8).masked(8 .. 9))
  x.setMask((PA12RM.uint32 shl 15).masked(15 .. 15))
  x.setMask((ADC1_ETRGINJ_RM.uint32 shl 17).masked(17 .. 17))
  x.setMask((ADC1_ETRGREG_RM.uint32 shl 18).masked(18 .. 18))
  x.setMask((USART1REMAP1.uint32 shl 21).masked(21 .. 21))
  x.setMask((I2C1REMAP1.uint32 shl 22).masked(22 .. 22))
  x.setMask((TIM1_IREMAP.uint32 shl 23).masked(23 .. 23))
  x.setMask((SWCFG shl 24).masked(24 .. 26))
  reg.write x.AFIO_PCFR_Fields

template modifyIt*(reg: AFIO_PCFR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: AFIO_EXTICR_Type): AFIO_EXTICR_Fields {.inline.} =
  volatileLoad(cast[ptr AFIO_EXTICR_Fields](reg.loc))

proc read*(reg: static AFIO_EXTICR_Type): AFIO_EXTICR_Fields {.inline.} =
  volatileLoad(cast[ptr AFIO_EXTICR_Fields](reg.loc))

proc write*(reg: AFIO_EXTICR_Type, val: AFIO_EXTICR_Fields) {.inline.} =
  volatileStore(cast[ptr AFIO_EXTICR_Fields](reg.loc), val)

proc write*(reg: AFIO_EXTICR_Type, EXTI0: uint32 = 0, EXTI1: uint32 = 0, EXTI2: uint32 = 0, EXTI3: uint32 = 0, EXTI4: uint32 = 0, EXTI5: uint32 = 0, EXTI6: uint32 = 0, EXTI7: uint32 = 0) =
  var x: uint32
  x.setMask((EXTI0 shl 0).masked(0 .. 1))
  x.setMask((EXTI1 shl 2).masked(2 .. 3))
  x.setMask((EXTI2 shl 4).masked(4 .. 5))
  x.setMask((EXTI3 shl 6).masked(6 .. 7))
  x.setMask((EXTI4 shl 8).masked(8 .. 9))
  x.setMask((EXTI5 shl 10).masked(10 .. 11))
  x.setMask((EXTI6 shl 12).masked(12 .. 13))
  x.setMask((EXTI7 shl 14).masked(14 .. 15))
  reg.write x.AFIO_EXTICR_Fields

template modifyIt*(reg: AFIO_EXTICR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func SPI1RM*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `SPI1RM=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.AFIO_PCFR_Fields

func I2C1RM*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `I2C1RM=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.AFIO_PCFR_Fields

func USART1RM*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `USART1RM=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.AFIO_PCFR_Fields

func TIM1RM*(r: AFIO_PCFR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(6 .. 7)

proc `TIM1RM=`*(r: var AFIO_PCFR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 7)
  tmp.setMask((val shl 6).masked(6 .. 7))
  r = tmp.AFIO_PCFR_Fields

func TIM2RM*(r: AFIO_PCFR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `TIM2RM=`*(r: var AFIO_PCFR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.AFIO_PCFR_Fields

func PA12RM*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `PA12RM=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.AFIO_PCFR_Fields

func ADC1_ETRGINJ_RM*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(17 .. 17).bool

proc `ADC1_ETRGINJ_RM=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(17 .. 17)
  tmp.setMask((val.uint32 shl 17).masked(17 .. 17))
  r = tmp.AFIO_PCFR_Fields

func ADC1_ETRGREG_RM*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(18 .. 18).bool

proc `ADC1_ETRGREG_RM=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 18)
  tmp.setMask((val.uint32 shl 18).masked(18 .. 18))
  r = tmp.AFIO_PCFR_Fields

func USART1REMAP1*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(21 .. 21).bool

proc `USART1REMAP1=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(21 .. 21)
  tmp.setMask((val.uint32 shl 21).masked(21 .. 21))
  r = tmp.AFIO_PCFR_Fields

func I2C1REMAP1*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(22 .. 22).bool

proc `I2C1REMAP1=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(22 .. 22)
  tmp.setMask((val.uint32 shl 22).masked(22 .. 22))
  r = tmp.AFIO_PCFR_Fields

func TIM1_IREMAP*(r: AFIO_PCFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(23 .. 23).bool

proc `TIM1_IREMAP=`*(r: var AFIO_PCFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(23 .. 23)
  tmp.setMask((val.uint32 shl 23).masked(23 .. 23))
  r = tmp.AFIO_PCFR_Fields

proc `SWCFG=`*(r: var AFIO_PCFR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(24 .. 26)
  tmp.setMask((val shl 24).masked(24 .. 26))
  r = tmp.AFIO_PCFR_Fields

func EXTI0*(r: AFIO_EXTICR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `EXTI0=`*(r: var AFIO_EXTICR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.AFIO_EXTICR_Fields

func EXTI1*(r: AFIO_EXTICR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

proc `EXTI1=`*(r: var AFIO_EXTICR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.AFIO_EXTICR_Fields

func EXTI2*(r: AFIO_EXTICR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 5)

proc `EXTI2=`*(r: var AFIO_EXTICR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 5)
  tmp.setMask((val shl 4).masked(4 .. 5))
  r = tmp.AFIO_EXTICR_Fields

func EXTI3*(r: AFIO_EXTICR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(6 .. 7)

proc `EXTI3=`*(r: var AFIO_EXTICR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 7)
  tmp.setMask((val shl 6).masked(6 .. 7))
  r = tmp.AFIO_EXTICR_Fields

func EXTI4*(r: AFIO_EXTICR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `EXTI4=`*(r: var AFIO_EXTICR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.AFIO_EXTICR_Fields

func EXTI5*(r: AFIO_EXTICR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `EXTI5=`*(r: var AFIO_EXTICR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.AFIO_EXTICR_Fields

func EXTI6*(r: AFIO_EXTICR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `EXTI6=`*(r: var AFIO_EXTICR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.AFIO_EXTICR_Fields

func EXTI7*(r: AFIO_EXTICR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(14 .. 15)

proc `EXTI7=`*(r: var AFIO_EXTICR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 15)
  tmp.setMask((val shl 14).masked(14 .. 15))
  r = tmp.AFIO_EXTICR_Fields

type
  EXTI_INTENR_Fields* = distinct uint32
  EXTI_EVENR_Fields* = distinct uint32
  EXTI_RTENR_Fields* = distinct uint32
  EXTI_FTENR_Fields* = distinct uint32
  EXTI_SWIEVR_Fields* = distinct uint32
  EXTI_INTFR_Fields* = distinct uint32

proc read*(reg: EXTI_INTENR_Type): EXTI_INTENR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_INTENR_Fields](reg.loc))

proc read*(reg: static EXTI_INTENR_Type): EXTI_INTENR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_INTENR_Fields](reg.loc))

proc write*(reg: EXTI_INTENR_Type, val: EXTI_INTENR_Fields) {.inline.} =
  volatileStore(cast[ptr EXTI_INTENR_Fields](reg.loc), val)

proc write*(reg: EXTI_INTENR_Type, MR0: bool = false, MR1: bool = false, MR2: bool = false, MR3: bool = false, MR4: bool = false, MR5: bool = false, MR6: bool = false, MR7: bool = false, MR8: bool = false, MR9: bool = false) =
  var x: uint32
  x.setMask((MR0.uint32 shl 0).masked(0 .. 0))
  x.setMask((MR1.uint32 shl 1).masked(1 .. 1))
  x.setMask((MR2.uint32 shl 2).masked(2 .. 2))
  x.setMask((MR3.uint32 shl 3).masked(3 .. 3))
  x.setMask((MR4.uint32 shl 4).masked(4 .. 4))
  x.setMask((MR5.uint32 shl 5).masked(5 .. 5))
  x.setMask((MR6.uint32 shl 6).masked(6 .. 6))
  x.setMask((MR7.uint32 shl 7).masked(7 .. 7))
  x.setMask((MR8.uint32 shl 8).masked(8 .. 8))
  x.setMask((MR9.uint32 shl 9).masked(9 .. 9))
  reg.write x.EXTI_INTENR_Fields

template modifyIt*(reg: EXTI_INTENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: EXTI_EVENR_Type): EXTI_EVENR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_EVENR_Fields](reg.loc))

proc read*(reg: static EXTI_EVENR_Type): EXTI_EVENR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_EVENR_Fields](reg.loc))

proc write*(reg: EXTI_EVENR_Type, val: EXTI_EVENR_Fields) {.inline.} =
  volatileStore(cast[ptr EXTI_EVENR_Fields](reg.loc), val)

proc write*(reg: EXTI_EVENR_Type, MR0: bool = false, MR1: bool = false, MR2: bool = false, MR3: bool = false, MR4: bool = false, MR5: bool = false, MR6: bool = false, MR7: bool = false, MR8: bool = false, MR9: bool = false) =
  var x: uint32
  x.setMask((MR0.uint32 shl 0).masked(0 .. 0))
  x.setMask((MR1.uint32 shl 1).masked(1 .. 1))
  x.setMask((MR2.uint32 shl 2).masked(2 .. 2))
  x.setMask((MR3.uint32 shl 3).masked(3 .. 3))
  x.setMask((MR4.uint32 shl 4).masked(4 .. 4))
  x.setMask((MR5.uint32 shl 5).masked(5 .. 5))
  x.setMask((MR6.uint32 shl 6).masked(6 .. 6))
  x.setMask((MR7.uint32 shl 7).masked(7 .. 7))
  x.setMask((MR8.uint32 shl 8).masked(8 .. 8))
  x.setMask((MR9.uint32 shl 9).masked(9 .. 9))
  reg.write x.EXTI_EVENR_Fields

template modifyIt*(reg: EXTI_EVENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: EXTI_RTENR_Type): EXTI_RTENR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_RTENR_Fields](reg.loc))

proc read*(reg: static EXTI_RTENR_Type): EXTI_RTENR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_RTENR_Fields](reg.loc))

proc write*(reg: EXTI_RTENR_Type, val: EXTI_RTENR_Fields) {.inline.} =
  volatileStore(cast[ptr EXTI_RTENR_Fields](reg.loc), val)

proc write*(reg: EXTI_RTENR_Type, TR0: bool = false, TR1: bool = false, TR2: bool = false, TR3: bool = false, TR4: bool = false, TR5: bool = false, TR6: bool = false, TR7: bool = false, TR8: bool = false, TR9: bool = false) =
  var x: uint32
  x.setMask((TR0.uint32 shl 0).masked(0 .. 0))
  x.setMask((TR1.uint32 shl 1).masked(1 .. 1))
  x.setMask((TR2.uint32 shl 2).masked(2 .. 2))
  x.setMask((TR3.uint32 shl 3).masked(3 .. 3))
  x.setMask((TR4.uint32 shl 4).masked(4 .. 4))
  x.setMask((TR5.uint32 shl 5).masked(5 .. 5))
  x.setMask((TR6.uint32 shl 6).masked(6 .. 6))
  x.setMask((TR7.uint32 shl 7).masked(7 .. 7))
  x.setMask((TR8.uint32 shl 8).masked(8 .. 8))
  x.setMask((TR9.uint32 shl 9).masked(9 .. 9))
  reg.write x.EXTI_RTENR_Fields

template modifyIt*(reg: EXTI_RTENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: EXTI_FTENR_Type): EXTI_FTENR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_FTENR_Fields](reg.loc))

proc read*(reg: static EXTI_FTENR_Type): EXTI_FTENR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_FTENR_Fields](reg.loc))

proc write*(reg: EXTI_FTENR_Type, val: EXTI_FTENR_Fields) {.inline.} =
  volatileStore(cast[ptr EXTI_FTENR_Fields](reg.loc), val)

proc write*(reg: EXTI_FTENR_Type, TR0: bool = false, TR1: bool = false, TR2: bool = false, TR3: bool = false, TR4: bool = false, TR5: bool = false, TR6: bool = false, TR7: bool = false, TR8: bool = false, TR9: bool = false) =
  var x: uint32
  x.setMask((TR0.uint32 shl 0).masked(0 .. 0))
  x.setMask((TR1.uint32 shl 1).masked(1 .. 1))
  x.setMask((TR2.uint32 shl 2).masked(2 .. 2))
  x.setMask((TR3.uint32 shl 3).masked(3 .. 3))
  x.setMask((TR4.uint32 shl 4).masked(4 .. 4))
  x.setMask((TR5.uint32 shl 5).masked(5 .. 5))
  x.setMask((TR6.uint32 shl 6).masked(6 .. 6))
  x.setMask((TR7.uint32 shl 7).masked(7 .. 7))
  x.setMask((TR8.uint32 shl 8).masked(8 .. 8))
  x.setMask((TR9.uint32 shl 9).masked(9 .. 9))
  reg.write x.EXTI_FTENR_Fields

template modifyIt*(reg: EXTI_FTENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: EXTI_SWIEVR_Type): EXTI_SWIEVR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_SWIEVR_Fields](reg.loc))

proc read*(reg: static EXTI_SWIEVR_Type): EXTI_SWIEVR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_SWIEVR_Fields](reg.loc))

proc write*(reg: EXTI_SWIEVR_Type, val: EXTI_SWIEVR_Fields) {.inline.} =
  volatileStore(cast[ptr EXTI_SWIEVR_Fields](reg.loc), val)

proc write*(reg: EXTI_SWIEVR_Type, SWIER0: bool = false, SWIER1: bool = false, SWIER2: bool = false, SWIER3: bool = false, SWIER4: bool = false, SWIER5: bool = false, SWIER6: bool = false, SWIER7: bool = false, SWIER8: bool = false, SWIER9: bool = false) =
  var x: uint32
  x.setMask((SWIER0.uint32 shl 0).masked(0 .. 0))
  x.setMask((SWIER1.uint32 shl 1).masked(1 .. 1))
  x.setMask((SWIER2.uint32 shl 2).masked(2 .. 2))
  x.setMask((SWIER3.uint32 shl 3).masked(3 .. 3))
  x.setMask((SWIER4.uint32 shl 4).masked(4 .. 4))
  x.setMask((SWIER5.uint32 shl 5).masked(5 .. 5))
  x.setMask((SWIER6.uint32 shl 6).masked(6 .. 6))
  x.setMask((SWIER7.uint32 shl 7).masked(7 .. 7))
  x.setMask((SWIER8.uint32 shl 8).masked(8 .. 8))
  x.setMask((SWIER9.uint32 shl 9).masked(9 .. 9))
  reg.write x.EXTI_SWIEVR_Fields

template modifyIt*(reg: EXTI_SWIEVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: EXTI_INTFR_Type): EXTI_INTFR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_INTFR_Fields](reg.loc))

proc read*(reg: static EXTI_INTFR_Type): EXTI_INTFR_Fields {.inline.} =
  volatileLoad(cast[ptr EXTI_INTFR_Fields](reg.loc))

proc write*(reg: EXTI_INTFR_Type, val: EXTI_INTFR_Fields) {.inline.} =
  volatileStore(cast[ptr EXTI_INTFR_Fields](reg.loc), val)

proc write*(reg: EXTI_INTFR_Type, PR0: bool = false, PR1: bool = false, PR2: bool = false, PR3: bool = false, PR4: bool = false, PR5: bool = false, PR6: bool = false, PR7: bool = false, PR8: bool = false, PR9: bool = false) =
  var x: uint32
  x.setMask((PR0.uint32 shl 0).masked(0 .. 0))
  x.setMask((PR1.uint32 shl 1).masked(1 .. 1))
  x.setMask((PR2.uint32 shl 2).masked(2 .. 2))
  x.setMask((PR3.uint32 shl 3).masked(3 .. 3))
  x.setMask((PR4.uint32 shl 4).masked(4 .. 4))
  x.setMask((PR5.uint32 shl 5).masked(5 .. 5))
  x.setMask((PR6.uint32 shl 6).masked(6 .. 6))
  x.setMask((PR7.uint32 shl 7).masked(7 .. 7))
  x.setMask((PR8.uint32 shl 8).masked(8 .. 8))
  x.setMask((PR9.uint32 shl 9).masked(9 .. 9))
  reg.write x.EXTI_INTFR_Fields

template modifyIt*(reg: EXTI_INTFR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func MR0*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `MR0=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.EXTI_INTENR_Fields

func MR1*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `MR1=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.EXTI_INTENR_Fields

func MR2*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `MR2=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.EXTI_INTENR_Fields

func MR3*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `MR3=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.EXTI_INTENR_Fields

func MR4*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `MR4=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.EXTI_INTENR_Fields

func MR5*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `MR5=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.EXTI_INTENR_Fields

func MR6*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `MR6=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.EXTI_INTENR_Fields

func MR7*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MR7=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.EXTI_INTENR_Fields

func MR8*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `MR8=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.EXTI_INTENR_Fields

func MR9*(r: EXTI_INTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `MR9=`*(r: var EXTI_INTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.EXTI_INTENR_Fields

func MR0*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `MR0=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.EXTI_EVENR_Fields

func MR1*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `MR1=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.EXTI_EVENR_Fields

func MR2*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `MR2=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.EXTI_EVENR_Fields

func MR3*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `MR3=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.EXTI_EVENR_Fields

func MR4*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `MR4=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.EXTI_EVENR_Fields

func MR5*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `MR5=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.EXTI_EVENR_Fields

func MR6*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `MR6=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.EXTI_EVENR_Fields

func MR7*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MR7=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.EXTI_EVENR_Fields

func MR8*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `MR8=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.EXTI_EVENR_Fields

func MR9*(r: EXTI_EVENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `MR9=`*(r: var EXTI_EVENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.EXTI_EVENR_Fields

func TR0*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `TR0=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.EXTI_RTENR_Fields

func TR1*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TR1=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.EXTI_RTENR_Fields

func TR2*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `TR2=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.EXTI_RTENR_Fields

func TR3*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TR3=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.EXTI_RTENR_Fields

func TR4*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `TR4=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.EXTI_RTENR_Fields

func TR5*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `TR5=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.EXTI_RTENR_Fields

func TR6*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `TR6=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.EXTI_RTENR_Fields

func TR7*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `TR7=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.EXTI_RTENR_Fields

func TR8*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `TR8=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.EXTI_RTENR_Fields

func TR9*(r: EXTI_RTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `TR9=`*(r: var EXTI_RTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.EXTI_RTENR_Fields

func TR0*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `TR0=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.EXTI_FTENR_Fields

func TR1*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TR1=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.EXTI_FTENR_Fields

func TR2*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `TR2=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.EXTI_FTENR_Fields

func TR3*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TR3=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.EXTI_FTENR_Fields

func TR4*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `TR4=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.EXTI_FTENR_Fields

func TR5*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `TR5=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.EXTI_FTENR_Fields

func TR6*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `TR6=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.EXTI_FTENR_Fields

func TR7*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `TR7=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.EXTI_FTENR_Fields

func TR8*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `TR8=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.EXTI_FTENR_Fields

func TR9*(r: EXTI_FTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `TR9=`*(r: var EXTI_FTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.EXTI_FTENR_Fields

func SWIER0*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `SWIER0=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER1*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `SWIER1=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER2*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `SWIER2=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER3*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `SWIER3=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER4*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `SWIER4=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER5*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `SWIER5=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER6*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `SWIER6=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER7*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `SWIER7=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER8*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `SWIER8=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.EXTI_SWIEVR_Fields

func SWIER9*(r: EXTI_SWIEVR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `SWIER9=`*(r: var EXTI_SWIEVR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.EXTI_SWIEVR_Fields

func PR0*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `PR0=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.EXTI_INTFR_Fields

func PR1*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `PR1=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.EXTI_INTFR_Fields

func PR2*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `PR2=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.EXTI_INTFR_Fields

func PR3*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `PR3=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.EXTI_INTFR_Fields

func PR4*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `PR4=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.EXTI_INTFR_Fields

func PR5*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `PR5=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.EXTI_INTFR_Fields

func PR6*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `PR6=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.EXTI_INTFR_Fields

func PR7*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `PR7=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.EXTI_INTFR_Fields

func PR8*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `PR8=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.EXTI_INTFR_Fields

func PR9*(r: EXTI_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `PR9=`*(r: var EXTI_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.EXTI_INTFR_Fields

type
  DMA1_INTFR_Fields* = distinct uint32
  DMA1_INTFCR_Fields* = distinct uint32
  DMA1_CFGR1_Fields* = distinct uint32
  DMA1_CNTR1_Fields* = distinct uint32
  DMA1_CFGR2_Fields* = distinct uint32
  DMA1_CNTR2_Fields* = distinct uint32
  DMA1_CFGR3_Fields* = distinct uint32
  DMA1_CNTR3_Fields* = distinct uint32
  DMA1_CFGR4_Fields* = distinct uint32
  DMA1_CNTR4_Fields* = distinct uint32
  DMA1_CFGR5_Fields* = distinct uint32
  DMA1_CNTR5_Fields* = distinct uint32
  DMA1_CFGR6_Fields* = distinct uint32
  DMA1_CNTR6_Fields* = distinct uint32
  DMA1_CFGR7_Fields* = distinct uint32
  DMA1_CNTR7_Fields* = distinct uint32

proc read*(reg: DMA1_INTFR_Type): DMA1_INTFR_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_INTFR_Fields](reg.loc))

proc read*(reg: static DMA1_INTFR_Type): DMA1_INTFR_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_INTFR_Fields](reg.loc))

proc write*(reg: DMA1_INTFCR_Type, val: DMA1_INTFCR_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_INTFCR_Fields](reg.loc), val)

proc write*(reg: DMA1_INTFCR_Type, CGIF1: bool = false, CGIF2: bool = false, CGIF3: bool = false, CGIF4: bool = false, CGIF5: bool = false, CGIF6: bool = false, CGIF7: bool = false, CTCIF1: bool = false, CTCIF2: bool = false, CTCIF3: bool = false, CTCIF4: bool = false, CTCIF5: bool = false, CTCIF6: bool = false, CTCIF7: bool = false, CHTIF1: bool = false, CHTIF2: bool = false, CHTIF3: bool = false, CHTIF4: bool = false, CHTIF5: bool = false, CHTIF6: bool = false, CHTIF7: bool = false, CTEIF1: bool = false, CTEIF2: bool = false, CTEIF3: bool = false, CTEIF4: bool = false, CTEIF5: bool = false, CTEIF6: bool = false, CTEIF7: bool = false) =
  var x: uint32
  x.setMask((CGIF1.uint32 shl 0).masked(0 .. 0))
  x.setMask((CGIF2.uint32 shl 4).masked(4 .. 4))
  x.setMask((CGIF3.uint32 shl 8).masked(8 .. 8))
  x.setMask((CGIF4.uint32 shl 12).masked(12 .. 12))
  x.setMask((CGIF5.uint32 shl 16).masked(16 .. 16))
  x.setMask((CGIF6.uint32 shl 20).masked(20 .. 20))
  x.setMask((CGIF7.uint32 shl 24).masked(24 .. 24))
  x.setMask((CTCIF1.uint32 shl 1).masked(1 .. 1))
  x.setMask((CTCIF2.uint32 shl 5).masked(5 .. 5))
  x.setMask((CTCIF3.uint32 shl 9).masked(9 .. 9))
  x.setMask((CTCIF4.uint32 shl 13).masked(13 .. 13))
  x.setMask((CTCIF5.uint32 shl 17).masked(17 .. 17))
  x.setMask((CTCIF6.uint32 shl 21).masked(21 .. 21))
  x.setMask((CTCIF7.uint32 shl 25).masked(25 .. 25))
  x.setMask((CHTIF1.uint32 shl 2).masked(2 .. 2))
  x.setMask((CHTIF2.uint32 shl 6).masked(6 .. 6))
  x.setMask((CHTIF3.uint32 shl 10).masked(10 .. 10))
  x.setMask((CHTIF4.uint32 shl 14).masked(14 .. 14))
  x.setMask((CHTIF5.uint32 shl 18).masked(18 .. 18))
  x.setMask((CHTIF6.uint32 shl 22).masked(22 .. 22))
  x.setMask((CHTIF7.uint32 shl 26).masked(26 .. 26))
  x.setMask((CTEIF1.uint32 shl 3).masked(3 .. 3))
  x.setMask((CTEIF2.uint32 shl 7).masked(7 .. 7))
  x.setMask((CTEIF3.uint32 shl 11).masked(11 .. 11))
  x.setMask((CTEIF4.uint32 shl 15).masked(15 .. 15))
  x.setMask((CTEIF5.uint32 shl 19).masked(19 .. 19))
  x.setMask((CTEIF6.uint32 shl 23).masked(23 .. 23))
  x.setMask((CTEIF7.uint32 shl 27).masked(27 .. 27))
  reg.write x.DMA1_INTFCR_Fields

proc read*(reg: DMA1_CFGR1_Type): DMA1_CFGR1_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR1_Fields](reg.loc))

proc read*(reg: static DMA1_CFGR1_Type): DMA1_CFGR1_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR1_Fields](reg.loc))

proc write*(reg: DMA1_CFGR1_Type, val: DMA1_CFGR1_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CFGR1_Fields](reg.loc), val)

proc write*(reg: DMA1_CFGR1_Type, EN: bool = false, TCIE: bool = false, HTIE: bool = false, TEIE: bool = false, DIR: bool = false, CIRC: bool = false, PINC: bool = false, MINC: bool = false, PSIZE: uint32 = 0, MSIZE: uint32 = 0, PL: uint32 = 0, MEM2MEM: bool = false) =
  var x: uint32
  x.setMask((EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((TCIE.uint32 shl 1).masked(1 .. 1))
  x.setMask((HTIE.uint32 shl 2).masked(2 .. 2))
  x.setMask((TEIE.uint32 shl 3).masked(3 .. 3))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((CIRC.uint32 shl 5).masked(5 .. 5))
  x.setMask((PINC.uint32 shl 6).masked(6 .. 6))
  x.setMask((MINC.uint32 shl 7).masked(7 .. 7))
  x.setMask((PSIZE shl 8).masked(8 .. 9))
  x.setMask((MSIZE shl 10).masked(10 .. 11))
  x.setMask((PL shl 12).masked(12 .. 13))
  x.setMask((MEM2MEM.uint32 shl 14).masked(14 .. 14))
  reg.write x.DMA1_CFGR1_Fields

template modifyIt*(reg: DMA1_CFGR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CNTR1_Type): DMA1_CNTR1_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR1_Fields](reg.loc))

proc read*(reg: static DMA1_CNTR1_Type): DMA1_CNTR1_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR1_Fields](reg.loc))

proc write*(reg: DMA1_CNTR1_Type, val: DMA1_CNTR1_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CNTR1_Fields](reg.loc), val)

proc write*(reg: DMA1_CNTR1_Type, NDT: uint32 = 0) =
  var x: uint32
  x.setMask((NDT shl 0).masked(0 .. 15))
  reg.write x.DMA1_CNTR1_Fields

template modifyIt*(reg: DMA1_CNTR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_PADDR1_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_PADDR1_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_PADDR1_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_PADDR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_MADDR1_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_MADDR1_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_MADDR1_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_MADDR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CFGR2_Type): DMA1_CFGR2_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR2_Fields](reg.loc))

proc read*(reg: static DMA1_CFGR2_Type): DMA1_CFGR2_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR2_Fields](reg.loc))

proc write*(reg: DMA1_CFGR2_Type, val: DMA1_CFGR2_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CFGR2_Fields](reg.loc), val)

proc write*(reg: DMA1_CFGR2_Type, EN: bool = false, TCIE: bool = false, HTIE: bool = false, TEIE: bool = false, DIR: bool = false, CIRC: bool = false, PINC: bool = false, MINC: bool = false, PSIZE: uint32 = 0, MSIZE: uint32 = 0, PL: uint32 = 0, MEM2MEM: bool = false) =
  var x: uint32
  x.setMask((EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((TCIE.uint32 shl 1).masked(1 .. 1))
  x.setMask((HTIE.uint32 shl 2).masked(2 .. 2))
  x.setMask((TEIE.uint32 shl 3).masked(3 .. 3))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((CIRC.uint32 shl 5).masked(5 .. 5))
  x.setMask((PINC.uint32 shl 6).masked(6 .. 6))
  x.setMask((MINC.uint32 shl 7).masked(7 .. 7))
  x.setMask((PSIZE shl 8).masked(8 .. 9))
  x.setMask((MSIZE shl 10).masked(10 .. 11))
  x.setMask((PL shl 12).masked(12 .. 13))
  x.setMask((MEM2MEM.uint32 shl 14).masked(14 .. 14))
  reg.write x.DMA1_CFGR2_Fields

template modifyIt*(reg: DMA1_CFGR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CNTR2_Type): DMA1_CNTR2_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR2_Fields](reg.loc))

proc read*(reg: static DMA1_CNTR2_Type): DMA1_CNTR2_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR2_Fields](reg.loc))

proc write*(reg: DMA1_CNTR2_Type, val: DMA1_CNTR2_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CNTR2_Fields](reg.loc), val)

proc write*(reg: DMA1_CNTR2_Type, NDT: uint32 = 0) =
  var x: uint32
  x.setMask((NDT shl 0).masked(0 .. 15))
  reg.write x.DMA1_CNTR2_Fields

template modifyIt*(reg: DMA1_CNTR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_PADDR2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_PADDR2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_PADDR2_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_PADDR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_MADDR2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_MADDR2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_MADDR2_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_MADDR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CFGR3_Type): DMA1_CFGR3_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR3_Fields](reg.loc))

proc read*(reg: static DMA1_CFGR3_Type): DMA1_CFGR3_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR3_Fields](reg.loc))

proc write*(reg: DMA1_CFGR3_Type, val: DMA1_CFGR3_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CFGR3_Fields](reg.loc), val)

proc write*(reg: DMA1_CFGR3_Type, EN: bool = false, TCIE: bool = false, HTIE: bool = false, TEIE: bool = false, DIR: bool = false, CIRC: bool = false, PINC: bool = false, MINC: bool = false, PSIZE: uint32 = 0, MSIZE: uint32 = 0, PL: uint32 = 0, MEM2MEM: bool = false) =
  var x: uint32
  x.setMask((EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((TCIE.uint32 shl 1).masked(1 .. 1))
  x.setMask((HTIE.uint32 shl 2).masked(2 .. 2))
  x.setMask((TEIE.uint32 shl 3).masked(3 .. 3))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((CIRC.uint32 shl 5).masked(5 .. 5))
  x.setMask((PINC.uint32 shl 6).masked(6 .. 6))
  x.setMask((MINC.uint32 shl 7).masked(7 .. 7))
  x.setMask((PSIZE shl 8).masked(8 .. 9))
  x.setMask((MSIZE shl 10).masked(10 .. 11))
  x.setMask((PL shl 12).masked(12 .. 13))
  x.setMask((MEM2MEM.uint32 shl 14).masked(14 .. 14))
  reg.write x.DMA1_CFGR3_Fields

template modifyIt*(reg: DMA1_CFGR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CNTR3_Type): DMA1_CNTR3_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR3_Fields](reg.loc))

proc read*(reg: static DMA1_CNTR3_Type): DMA1_CNTR3_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR3_Fields](reg.loc))

proc write*(reg: DMA1_CNTR3_Type, val: DMA1_CNTR3_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CNTR3_Fields](reg.loc), val)

proc write*(reg: DMA1_CNTR3_Type, NDT: uint32 = 0) =
  var x: uint32
  x.setMask((NDT shl 0).masked(0 .. 15))
  reg.write x.DMA1_CNTR3_Fields

template modifyIt*(reg: DMA1_CNTR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_PADDR3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_PADDR3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_PADDR3_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_PADDR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_MADDR3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_MADDR3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_MADDR3_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_MADDR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CFGR4_Type): DMA1_CFGR4_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR4_Fields](reg.loc))

proc read*(reg: static DMA1_CFGR4_Type): DMA1_CFGR4_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR4_Fields](reg.loc))

proc write*(reg: DMA1_CFGR4_Type, val: DMA1_CFGR4_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CFGR4_Fields](reg.loc), val)

proc write*(reg: DMA1_CFGR4_Type, EN: bool = false, TCIE: bool = false, HTIE: bool = false, TEIE: bool = false, DIR: bool = false, CIRC: bool = false, PINC: bool = false, MINC: bool = false, PSIZE: uint32 = 0, MSIZE: uint32 = 0, PL: uint32 = 0, MEM2MEM: bool = false) =
  var x: uint32
  x.setMask((EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((TCIE.uint32 shl 1).masked(1 .. 1))
  x.setMask((HTIE.uint32 shl 2).masked(2 .. 2))
  x.setMask((TEIE.uint32 shl 3).masked(3 .. 3))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((CIRC.uint32 shl 5).masked(5 .. 5))
  x.setMask((PINC.uint32 shl 6).masked(6 .. 6))
  x.setMask((MINC.uint32 shl 7).masked(7 .. 7))
  x.setMask((PSIZE shl 8).masked(8 .. 9))
  x.setMask((MSIZE shl 10).masked(10 .. 11))
  x.setMask((PL shl 12).masked(12 .. 13))
  x.setMask((MEM2MEM.uint32 shl 14).masked(14 .. 14))
  reg.write x.DMA1_CFGR4_Fields

template modifyIt*(reg: DMA1_CFGR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CNTR4_Type): DMA1_CNTR4_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR4_Fields](reg.loc))

proc read*(reg: static DMA1_CNTR4_Type): DMA1_CNTR4_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR4_Fields](reg.loc))

proc write*(reg: DMA1_CNTR4_Type, val: DMA1_CNTR4_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CNTR4_Fields](reg.loc), val)

proc write*(reg: DMA1_CNTR4_Type, NDT: uint32 = 0) =
  var x: uint32
  x.setMask((NDT shl 0).masked(0 .. 15))
  reg.write x.DMA1_CNTR4_Fields

template modifyIt*(reg: DMA1_CNTR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_PADDR4_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_PADDR4_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_PADDR4_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_PADDR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_MADDR4_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_MADDR4_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_MADDR4_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_MADDR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CFGR5_Type): DMA1_CFGR5_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR5_Fields](reg.loc))

proc read*(reg: static DMA1_CFGR5_Type): DMA1_CFGR5_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR5_Fields](reg.loc))

proc write*(reg: DMA1_CFGR5_Type, val: DMA1_CFGR5_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CFGR5_Fields](reg.loc), val)

proc write*(reg: DMA1_CFGR5_Type, EN: bool = false, TCIE: bool = false, HTIE: bool = false, TEIE: bool = false, DIR: bool = false, CIRC: bool = false, PINC: bool = false, MINC: bool = false, PSIZE: uint32 = 0, MSIZE: uint32 = 0, PL: uint32 = 0, MEM2MEM: bool = false) =
  var x: uint32
  x.setMask((EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((TCIE.uint32 shl 1).masked(1 .. 1))
  x.setMask((HTIE.uint32 shl 2).masked(2 .. 2))
  x.setMask((TEIE.uint32 shl 3).masked(3 .. 3))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((CIRC.uint32 shl 5).masked(5 .. 5))
  x.setMask((PINC.uint32 shl 6).masked(6 .. 6))
  x.setMask((MINC.uint32 shl 7).masked(7 .. 7))
  x.setMask((PSIZE shl 8).masked(8 .. 9))
  x.setMask((MSIZE shl 10).masked(10 .. 11))
  x.setMask((PL shl 12).masked(12 .. 13))
  x.setMask((MEM2MEM.uint32 shl 14).masked(14 .. 14))
  reg.write x.DMA1_CFGR5_Fields

template modifyIt*(reg: DMA1_CFGR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CNTR5_Type): DMA1_CNTR5_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR5_Fields](reg.loc))

proc read*(reg: static DMA1_CNTR5_Type): DMA1_CNTR5_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR5_Fields](reg.loc))

proc write*(reg: DMA1_CNTR5_Type, val: DMA1_CNTR5_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CNTR5_Fields](reg.loc), val)

proc write*(reg: DMA1_CNTR5_Type, NDT: uint32 = 0) =
  var x: uint32
  x.setMask((NDT shl 0).masked(0 .. 15))
  reg.write x.DMA1_CNTR5_Fields

template modifyIt*(reg: DMA1_CNTR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_PADDR5_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_PADDR5_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_PADDR5_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_PADDR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_MADDR5_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_MADDR5_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_MADDR5_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_MADDR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CFGR6_Type): DMA1_CFGR6_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR6_Fields](reg.loc))

proc read*(reg: static DMA1_CFGR6_Type): DMA1_CFGR6_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR6_Fields](reg.loc))

proc write*(reg: DMA1_CFGR6_Type, val: DMA1_CFGR6_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CFGR6_Fields](reg.loc), val)

proc write*(reg: DMA1_CFGR6_Type, EN: bool = false, TCIE: bool = false, HTIE: bool = false, TEIE: bool = false, DIR: bool = false, CIRC: bool = false, PINC: bool = false, MINC: bool = false, PSIZE: uint32 = 0, MSIZE: uint32 = 0, PL: uint32 = 0, MEM2MEM: bool = false) =
  var x: uint32
  x.setMask((EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((TCIE.uint32 shl 1).masked(1 .. 1))
  x.setMask((HTIE.uint32 shl 2).masked(2 .. 2))
  x.setMask((TEIE.uint32 shl 3).masked(3 .. 3))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((CIRC.uint32 shl 5).masked(5 .. 5))
  x.setMask((PINC.uint32 shl 6).masked(6 .. 6))
  x.setMask((MINC.uint32 shl 7).masked(7 .. 7))
  x.setMask((PSIZE shl 8).masked(8 .. 9))
  x.setMask((MSIZE shl 10).masked(10 .. 11))
  x.setMask((PL shl 12).masked(12 .. 13))
  x.setMask((MEM2MEM.uint32 shl 14).masked(14 .. 14))
  reg.write x.DMA1_CFGR6_Fields

template modifyIt*(reg: DMA1_CFGR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CNTR6_Type): DMA1_CNTR6_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR6_Fields](reg.loc))

proc read*(reg: static DMA1_CNTR6_Type): DMA1_CNTR6_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR6_Fields](reg.loc))

proc write*(reg: DMA1_CNTR6_Type, val: DMA1_CNTR6_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CNTR6_Fields](reg.loc), val)

proc write*(reg: DMA1_CNTR6_Type, NDT: uint32 = 0) =
  var x: uint32
  x.setMask((NDT shl 0).masked(0 .. 15))
  reg.write x.DMA1_CNTR6_Fields

template modifyIt*(reg: DMA1_CNTR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_PADDR6_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_PADDR6_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_PADDR6_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_PADDR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_MADDR6_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_MADDR6_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_MADDR6_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_MADDR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CFGR7_Type): DMA1_CFGR7_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR7_Fields](reg.loc))

proc read*(reg: static DMA1_CFGR7_Type): DMA1_CFGR7_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CFGR7_Fields](reg.loc))

proc write*(reg: DMA1_CFGR7_Type, val: DMA1_CFGR7_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CFGR7_Fields](reg.loc), val)

proc write*(reg: DMA1_CFGR7_Type, EN: bool = false, TCIE: bool = false, HTIE: bool = false, TEIE: bool = false, DIR: bool = false, CIRC: bool = false, PINC: bool = false, MINC: bool = false, PSIZE: uint32 = 0, MSIZE: uint32 = 0, PL: uint32 = 0, MEM2MEM: bool = false) =
  var x: uint32
  x.setMask((EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((TCIE.uint32 shl 1).masked(1 .. 1))
  x.setMask((HTIE.uint32 shl 2).masked(2 .. 2))
  x.setMask((TEIE.uint32 shl 3).masked(3 .. 3))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((CIRC.uint32 shl 5).masked(5 .. 5))
  x.setMask((PINC.uint32 shl 6).masked(6 .. 6))
  x.setMask((MINC.uint32 shl 7).masked(7 .. 7))
  x.setMask((PSIZE shl 8).masked(8 .. 9))
  x.setMask((MSIZE shl 10).masked(10 .. 11))
  x.setMask((PL shl 12).masked(12 .. 13))
  x.setMask((MEM2MEM.uint32 shl 14).masked(14 .. 14))
  reg.write x.DMA1_CFGR7_Fields

template modifyIt*(reg: DMA1_CFGR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_CNTR7_Type): DMA1_CNTR7_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR7_Fields](reg.loc))

proc read*(reg: static DMA1_CNTR7_Type): DMA1_CNTR7_Fields {.inline.} =
  volatileLoad(cast[ptr DMA1_CNTR7_Fields](reg.loc))

proc write*(reg: DMA1_CNTR7_Type, val: DMA1_CNTR7_Fields) {.inline.} =
  volatileStore(cast[ptr DMA1_CNTR7_Fields](reg.loc), val)

proc write*(reg: DMA1_CNTR7_Type, NDT: uint32 = 0) =
  var x: uint32
  x.setMask((NDT shl 0).masked(0 .. 15))
  reg.write x.DMA1_CNTR7_Fields

template modifyIt*(reg: DMA1_CNTR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_PADDR7_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_PADDR7_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_PADDR7_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_PADDR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DMA1_MADDR7_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static DMA1_MADDR7_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: DMA1_MADDR7_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: DMA1_MADDR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func GIF1*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func TCIF1*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func HTIF1*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func TEIF1*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

func GIF2*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

func TCIF2*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

func HTIF2*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

func TEIF2*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

func GIF3*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

func TCIF3*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

func HTIF3*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

func TEIF3*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

func GIF4*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

func TCIF4*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(13 .. 13).bool

func HTIF4*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

func TEIF4*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

func GIF5*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(16 .. 16).bool

func TCIF5*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(17 .. 17).bool

func HTIF5*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(18 .. 18).bool

func TEIF5*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(19 .. 19).bool

func GIF6*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(20 .. 20).bool

func TCIF6*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(21 .. 21).bool

func HTIF6*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(22 .. 22).bool

func TEIF6*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(23 .. 23).bool

func GIF7*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(24 .. 24).bool

func TCIF7*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(25 .. 25).bool

func HTIF7*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(26 .. 26).bool

func TEIF7*(r: DMA1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(27 .. 27).bool

proc `CGIF1=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DMA1_INTFCR_Fields

proc `CGIF2=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DMA1_INTFCR_Fields

proc `CGIF3=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.DMA1_INTFCR_Fields

proc `CGIF4=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.DMA1_INTFCR_Fields

proc `CGIF5=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 16)
  tmp.setMask((val.uint32 shl 16).masked(16 .. 16))
  r = tmp.DMA1_INTFCR_Fields

proc `CGIF6=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 20)
  tmp.setMask((val.uint32 shl 20).masked(20 .. 20))
  r = tmp.DMA1_INTFCR_Fields

proc `CGIF7=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(24 .. 24)
  tmp.setMask((val.uint32 shl 24).masked(24 .. 24))
  r = tmp.DMA1_INTFCR_Fields

proc `CTCIF1=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DMA1_INTFCR_Fields

proc `CTCIF2=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DMA1_INTFCR_Fields

proc `CTCIF3=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.DMA1_INTFCR_Fields

proc `CTCIF4=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 13)
  tmp.setMask((val.uint32 shl 13).masked(13 .. 13))
  r = tmp.DMA1_INTFCR_Fields

proc `CTCIF5=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(17 .. 17)
  tmp.setMask((val.uint32 shl 17).masked(17 .. 17))
  r = tmp.DMA1_INTFCR_Fields

proc `CTCIF6=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(21 .. 21)
  tmp.setMask((val.uint32 shl 21).masked(21 .. 21))
  r = tmp.DMA1_INTFCR_Fields

proc `CTCIF7=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(25 .. 25)
  tmp.setMask((val.uint32 shl 25).masked(25 .. 25))
  r = tmp.DMA1_INTFCR_Fields

proc `CHTIF1=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DMA1_INTFCR_Fields

proc `CHTIF2=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.DMA1_INTFCR_Fields

proc `CHTIF3=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.DMA1_INTFCR_Fields

proc `CHTIF4=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.DMA1_INTFCR_Fields

proc `CHTIF5=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 18)
  tmp.setMask((val.uint32 shl 18).masked(18 .. 18))
  r = tmp.DMA1_INTFCR_Fields

proc `CHTIF6=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(22 .. 22)
  tmp.setMask((val.uint32 shl 22).masked(22 .. 22))
  r = tmp.DMA1_INTFCR_Fields

proc `CHTIF7=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(26 .. 26)
  tmp.setMask((val.uint32 shl 26).masked(26 .. 26))
  r = tmp.DMA1_INTFCR_Fields

proc `CTEIF1=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.DMA1_INTFCR_Fields

proc `CTEIF2=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.DMA1_INTFCR_Fields

proc `CTEIF3=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.DMA1_INTFCR_Fields

proc `CTEIF4=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.DMA1_INTFCR_Fields

proc `CTEIF5=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(19 .. 19)
  tmp.setMask((val.uint32 shl 19).masked(19 .. 19))
  r = tmp.DMA1_INTFCR_Fields

proc `CTEIF6=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(23 .. 23)
  tmp.setMask((val.uint32 shl 23).masked(23 .. 23))
  r = tmp.DMA1_INTFCR_Fields

proc `CTEIF7=`*(r: var DMA1_INTFCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(27 .. 27)
  tmp.setMask((val.uint32 shl 27).masked(27 .. 27))
  r = tmp.DMA1_INTFCR_Fields

func EN*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `EN=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DMA1_CFGR1_Fields

func TCIE*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TCIE=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DMA1_CFGR1_Fields

func HTIE*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `HTIE=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DMA1_CFGR1_Fields

func TEIE*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TEIE=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.DMA1_CFGR1_Fields

func DIR*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DMA1_CFGR1_Fields

func CIRC*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CIRC=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DMA1_CFGR1_Fields

func PINC*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `PINC=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.DMA1_CFGR1_Fields

func MINC*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MINC=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.DMA1_CFGR1_Fields

func PSIZE*(r: DMA1_CFGR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `PSIZE=`*(r: var DMA1_CFGR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.DMA1_CFGR1_Fields

func MSIZE*(r: DMA1_CFGR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `MSIZE=`*(r: var DMA1_CFGR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.DMA1_CFGR1_Fields

func PL*(r: DMA1_CFGR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `PL=`*(r: var DMA1_CFGR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.DMA1_CFGR1_Fields

func MEM2MEM*(r: DMA1_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `MEM2MEM=`*(r: var DMA1_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.DMA1_CFGR1_Fields

func NDT*(r: DMA1_CNTR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `NDT=`*(r: var DMA1_CNTR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.DMA1_CNTR1_Fields

func EN*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `EN=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DMA1_CFGR2_Fields

func TCIE*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TCIE=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DMA1_CFGR2_Fields

func HTIE*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `HTIE=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DMA1_CFGR2_Fields

func TEIE*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TEIE=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.DMA1_CFGR2_Fields

func DIR*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DMA1_CFGR2_Fields

func CIRC*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CIRC=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DMA1_CFGR2_Fields

func PINC*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `PINC=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.DMA1_CFGR2_Fields

func MINC*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MINC=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.DMA1_CFGR2_Fields

func PSIZE*(r: DMA1_CFGR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `PSIZE=`*(r: var DMA1_CFGR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.DMA1_CFGR2_Fields

func MSIZE*(r: DMA1_CFGR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `MSIZE=`*(r: var DMA1_CFGR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.DMA1_CFGR2_Fields

func PL*(r: DMA1_CFGR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `PL=`*(r: var DMA1_CFGR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.DMA1_CFGR2_Fields

func MEM2MEM*(r: DMA1_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `MEM2MEM=`*(r: var DMA1_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.DMA1_CFGR2_Fields

func NDT*(r: DMA1_CNTR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `NDT=`*(r: var DMA1_CNTR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.DMA1_CNTR2_Fields

func EN*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `EN=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DMA1_CFGR3_Fields

func TCIE*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TCIE=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DMA1_CFGR3_Fields

func HTIE*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `HTIE=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DMA1_CFGR3_Fields

func TEIE*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TEIE=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.DMA1_CFGR3_Fields

func DIR*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DMA1_CFGR3_Fields

func CIRC*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CIRC=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DMA1_CFGR3_Fields

func PINC*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `PINC=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.DMA1_CFGR3_Fields

func MINC*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MINC=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.DMA1_CFGR3_Fields

func PSIZE*(r: DMA1_CFGR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `PSIZE=`*(r: var DMA1_CFGR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.DMA1_CFGR3_Fields

func MSIZE*(r: DMA1_CFGR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `MSIZE=`*(r: var DMA1_CFGR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.DMA1_CFGR3_Fields

func PL*(r: DMA1_CFGR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `PL=`*(r: var DMA1_CFGR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.DMA1_CFGR3_Fields

func MEM2MEM*(r: DMA1_CFGR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `MEM2MEM=`*(r: var DMA1_CFGR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.DMA1_CFGR3_Fields

func NDT*(r: DMA1_CNTR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `NDT=`*(r: var DMA1_CNTR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.DMA1_CNTR3_Fields

func EN*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `EN=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DMA1_CFGR4_Fields

func TCIE*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TCIE=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DMA1_CFGR4_Fields

func HTIE*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `HTIE=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DMA1_CFGR4_Fields

func TEIE*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TEIE=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.DMA1_CFGR4_Fields

func DIR*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DMA1_CFGR4_Fields

func CIRC*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CIRC=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DMA1_CFGR4_Fields

func PINC*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `PINC=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.DMA1_CFGR4_Fields

func MINC*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MINC=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.DMA1_CFGR4_Fields

func PSIZE*(r: DMA1_CFGR4_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `PSIZE=`*(r: var DMA1_CFGR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.DMA1_CFGR4_Fields

func MSIZE*(r: DMA1_CFGR4_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `MSIZE=`*(r: var DMA1_CFGR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.DMA1_CFGR4_Fields

func PL*(r: DMA1_CFGR4_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `PL=`*(r: var DMA1_CFGR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.DMA1_CFGR4_Fields

func MEM2MEM*(r: DMA1_CFGR4_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `MEM2MEM=`*(r: var DMA1_CFGR4_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.DMA1_CFGR4_Fields

func NDT*(r: DMA1_CNTR4_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `NDT=`*(r: var DMA1_CNTR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.DMA1_CNTR4_Fields

func EN*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `EN=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DMA1_CFGR5_Fields

func TCIE*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TCIE=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DMA1_CFGR5_Fields

func HTIE*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `HTIE=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DMA1_CFGR5_Fields

func TEIE*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TEIE=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.DMA1_CFGR5_Fields

func DIR*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DMA1_CFGR5_Fields

func CIRC*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CIRC=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DMA1_CFGR5_Fields

func PINC*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `PINC=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.DMA1_CFGR5_Fields

func MINC*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MINC=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.DMA1_CFGR5_Fields

func PSIZE*(r: DMA1_CFGR5_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `PSIZE=`*(r: var DMA1_CFGR5_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.DMA1_CFGR5_Fields

func MSIZE*(r: DMA1_CFGR5_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `MSIZE=`*(r: var DMA1_CFGR5_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.DMA1_CFGR5_Fields

func PL*(r: DMA1_CFGR5_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `PL=`*(r: var DMA1_CFGR5_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.DMA1_CFGR5_Fields

func MEM2MEM*(r: DMA1_CFGR5_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `MEM2MEM=`*(r: var DMA1_CFGR5_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.DMA1_CFGR5_Fields

func NDT*(r: DMA1_CNTR5_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `NDT=`*(r: var DMA1_CNTR5_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.DMA1_CNTR5_Fields

func EN*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `EN=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DMA1_CFGR6_Fields

func TCIE*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TCIE=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DMA1_CFGR6_Fields

func HTIE*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `HTIE=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DMA1_CFGR6_Fields

func TEIE*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TEIE=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.DMA1_CFGR6_Fields

func DIR*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DMA1_CFGR6_Fields

func CIRC*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CIRC=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DMA1_CFGR6_Fields

func PINC*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `PINC=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.DMA1_CFGR6_Fields

func MINC*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MINC=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.DMA1_CFGR6_Fields

func PSIZE*(r: DMA1_CFGR6_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `PSIZE=`*(r: var DMA1_CFGR6_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.DMA1_CFGR6_Fields

func MSIZE*(r: DMA1_CFGR6_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `MSIZE=`*(r: var DMA1_CFGR6_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.DMA1_CFGR6_Fields

func PL*(r: DMA1_CFGR6_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `PL=`*(r: var DMA1_CFGR6_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.DMA1_CFGR6_Fields

func MEM2MEM*(r: DMA1_CFGR6_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `MEM2MEM=`*(r: var DMA1_CFGR6_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.DMA1_CFGR6_Fields

func NDT*(r: DMA1_CNTR6_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `NDT=`*(r: var DMA1_CNTR6_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.DMA1_CNTR6_Fields

func EN*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `EN=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DMA1_CFGR7_Fields

func TCIE*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TCIE=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DMA1_CFGR7_Fields

func HTIE*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `HTIE=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DMA1_CFGR7_Fields

func TEIE*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TEIE=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.DMA1_CFGR7_Fields

func DIR*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DMA1_CFGR7_Fields

func CIRC*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CIRC=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DMA1_CFGR7_Fields

func PINC*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `PINC=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.DMA1_CFGR7_Fields

func MINC*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MINC=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.DMA1_CFGR7_Fields

func PSIZE*(r: DMA1_CFGR7_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `PSIZE=`*(r: var DMA1_CFGR7_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.DMA1_CFGR7_Fields

func MSIZE*(r: DMA1_CFGR7_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `MSIZE=`*(r: var DMA1_CFGR7_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.DMA1_CFGR7_Fields

func PL*(r: DMA1_CFGR7_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `PL=`*(r: var DMA1_CFGR7_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.DMA1_CFGR7_Fields

func MEM2MEM*(r: DMA1_CFGR7_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `MEM2MEM=`*(r: var DMA1_CFGR7_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.DMA1_CFGR7_Fields

func NDT*(r: DMA1_CNTR7_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `NDT=`*(r: var DMA1_CNTR7_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.DMA1_CNTR7_Fields

type
  IWDG_CTLR_Fields* = distinct uint32
  IWDG_PSCR_Fields* = distinct uint32
  IWDG_RLDR_Fields* = distinct uint32
  IWDG_STATR_Fields* = distinct uint32

proc write*(reg: IWDG_CTLR_Type, val: IWDG_CTLR_Fields) {.inline.} =
  volatileStore(cast[ptr IWDG_CTLR_Fields](reg.loc), val)

proc write*(reg: IWDG_CTLR_Type, KEY: uint32 = 0) =
  var x: uint32
  x.setMask((KEY shl 0).masked(0 .. 15))
  reg.write x.IWDG_CTLR_Fields

proc read*(reg: IWDG_PSCR_Type): IWDG_PSCR_Fields {.inline.} =
  volatileLoad(cast[ptr IWDG_PSCR_Fields](reg.loc))

proc read*(reg: static IWDG_PSCR_Type): IWDG_PSCR_Fields {.inline.} =
  volatileLoad(cast[ptr IWDG_PSCR_Fields](reg.loc))

proc write*(reg: IWDG_PSCR_Type, val: IWDG_PSCR_Fields) {.inline.} =
  volatileStore(cast[ptr IWDG_PSCR_Fields](reg.loc), val)

proc write*(reg: IWDG_PSCR_Type, PR: uint32 = 0) =
  var x: uint32
  x.setMask((PR shl 0).masked(0 .. 2))
  reg.write x.IWDG_PSCR_Fields

template modifyIt*(reg: IWDG_PSCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: IWDG_RLDR_Type): IWDG_RLDR_Fields {.inline.} =
  volatileLoad(cast[ptr IWDG_RLDR_Fields](reg.loc))

proc read*(reg: static IWDG_RLDR_Type): IWDG_RLDR_Fields {.inline.} =
  volatileLoad(cast[ptr IWDG_RLDR_Fields](reg.loc))

proc write*(reg: IWDG_RLDR_Type, val: IWDG_RLDR_Fields) {.inline.} =
  volatileStore(cast[ptr IWDG_RLDR_Fields](reg.loc), val)

proc write*(reg: IWDG_RLDR_Type, RL: uint32 = 4095) =
  var x: uint32
  x.setMask((RL shl 0).masked(0 .. 11))
  reg.write x.IWDG_RLDR_Fields

template modifyIt*(reg: IWDG_RLDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: IWDG_STATR_Type): IWDG_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr IWDG_STATR_Fields](reg.loc))

proc read*(reg: static IWDG_STATR_Type): IWDG_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr IWDG_STATR_Fields](reg.loc))

proc `KEY=`*(r: var IWDG_CTLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.IWDG_CTLR_Fields

func PR*(r: IWDG_PSCR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 2)

proc `PR=`*(r: var IWDG_PSCR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 2)
  tmp.setMask((val shl 0).masked(0 .. 2))
  r = tmp.IWDG_PSCR_Fields

func RL*(r: IWDG_RLDR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 11)

proc `RL=`*(r: var IWDG_RLDR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 11)
  tmp.setMask((val shl 0).masked(0 .. 11))
  r = tmp.IWDG_RLDR_Fields

func PVU*(r: IWDG_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func RVU*(r: IWDG_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

type
  WWDG_CTLR_Fields* = distinct uint32
  WWDG_CFGR_Fields* = distinct uint32
  WWDG_STATR_Fields* = distinct uint32

proc read*(reg: WWDG_CTLR_Type): WWDG_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr WWDG_CTLR_Fields](reg.loc))

proc read*(reg: static WWDG_CTLR_Type): WWDG_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr WWDG_CTLR_Fields](reg.loc))

proc write*(reg: WWDG_CTLR_Type, val: WWDG_CTLR_Fields) {.inline.} =
  volatileStore(cast[ptr WWDG_CTLR_Fields](reg.loc), val)

proc write*(reg: WWDG_CTLR_Type, T: uint32 = 127, WDGA: bool = false) =
  var x: uint32
  x.setMask((T shl 0).masked(0 .. 6))
  x.setMask((WDGA.uint32 shl 7).masked(7 .. 7))
  reg.write x.WWDG_CTLR_Fields

template modifyIt*(reg: WWDG_CTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: WWDG_CFGR_Type): WWDG_CFGR_Fields {.inline.} =
  volatileLoad(cast[ptr WWDG_CFGR_Fields](reg.loc))

proc read*(reg: static WWDG_CFGR_Type): WWDG_CFGR_Fields {.inline.} =
  volatileLoad(cast[ptr WWDG_CFGR_Fields](reg.loc))

proc write*(reg: WWDG_CFGR_Type, val: WWDG_CFGR_Fields) {.inline.} =
  volatileStore(cast[ptr WWDG_CFGR_Fields](reg.loc), val)

proc write*(reg: WWDG_CFGR_Type, W: uint32 = 127, WDGTB: uint32 = 0, EWI: bool = false) =
  var x: uint32
  x.setMask((W shl 0).masked(0 .. 6))
  x.setMask((WDGTB shl 7).masked(7 .. 8))
  x.setMask((EWI.uint32 shl 9).masked(9 .. 9))
  reg.write x.WWDG_CFGR_Fields

template modifyIt*(reg: WWDG_CFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: WWDG_STATR_Type): WWDG_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr WWDG_STATR_Fields](reg.loc))

proc read*(reg: static WWDG_STATR_Type): WWDG_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr WWDG_STATR_Fields](reg.loc))

proc write*(reg: WWDG_STATR_Type, val: WWDG_STATR_Fields) {.inline.} =
  volatileStore(cast[ptr WWDG_STATR_Fields](reg.loc), val)

proc write*(reg: WWDG_STATR_Type, WEIF: bool = false) =
  var x: uint32
  x.setMask((WEIF.uint32 shl 0).masked(0 .. 0))
  reg.write x.WWDG_STATR_Fields

template modifyIt*(reg: WWDG_STATR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func T*(r: WWDG_CTLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 6)

proc `T=`*(r: var WWDG_CTLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 6)
  tmp.setMask((val shl 0).masked(0 .. 6))
  r = tmp.WWDG_CTLR_Fields

func WDGA*(r: WWDG_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `WDGA=`*(r: var WWDG_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.WWDG_CTLR_Fields

func W*(r: WWDG_CFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 6)

proc `W=`*(r: var WWDG_CFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 6)
  tmp.setMask((val shl 0).masked(0 .. 6))
  r = tmp.WWDG_CFGR_Fields

func WDGTB*(r: WWDG_CFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(7 .. 8)

proc `WDGTB=`*(r: var WWDG_CFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 8)
  tmp.setMask((val shl 7).masked(7 .. 8))
  r = tmp.WWDG_CFGR_Fields

func EWI*(r: WWDG_CFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `EWI=`*(r: var WWDG_CFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.WWDG_CFGR_Fields

func WEIF*(r: WWDG_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `WEIF=`*(r: var WWDG_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.WWDG_STATR_Fields

type
  TIM1_CTLR1_Fields* = distinct uint32
  TIM1_CTLR2_Fields* = distinct uint32
  TIM1_SMCFGR_Fields* = distinct uint32
  TIM1_DMAINTENR_Fields* = distinct uint32
  TIM1_INTFR_Fields* = distinct uint32
  TIM1_SWEVGR_Fields* = distinct uint32
  TIM1_CHCTLR1_Output_Fields* = distinct uint32
  TIM1_CHCTLR1_Input_Fields* = distinct uint32
  TIM1_CHCTLR2_Output_Fields* = distinct uint32
  TIM1_CHCTLR2_Input_Fields* = distinct uint32
  TIM1_CCER_Fields* = distinct uint32
  TIM1_CNT_Fields* = distinct uint32
  TIM1_PSC_Fields* = distinct uint32
  TIM1_ATRLR_Fields* = distinct uint32
  TIM1_RPTCR_Fields* = distinct uint32
  TIM1_CH1CVR_Fields* = distinct uint32
  TIM1_CH2CVR_Fields* = distinct uint32
  TIM1_CH3CVR_Fields* = distinct uint32
  TIM1_CH4CVR_Fields* = distinct uint32
  TIM1_BDTR_Fields* = distinct uint32
  TIM1_DMACFGR_Fields* = distinct uint32
  TIM1_DMAADR_Fields* = distinct uint32

proc read*(reg: TIM1_CTLR1_Type): TIM1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CTLR1_Fields](reg.loc))

proc read*(reg: static TIM1_CTLR1_Type): TIM1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CTLR1_Fields](reg.loc))

proc write*(reg: TIM1_CTLR1_Type, val: TIM1_CTLR1_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CTLR1_Fields](reg.loc), val)

proc write*(reg: TIM1_CTLR1_Type, TMR_CAP_LVL_EN: bool = false, TMR_CAP_OV_EN: bool = false, CKD: uint32 = 0, ARPE: bool = false, CMS: uint32 = 0, DIR: bool = false, OPM: bool = false, URS: bool = false, UDIS: bool = false, CEN: bool = false) =
  var x: uint32
  x.setMask((TMR_CAP_LVL_EN.uint32 shl 15).masked(15 .. 15))
  x.setMask((TMR_CAP_OV_EN.uint32 shl 14).masked(14 .. 14))
  x.setMask((CKD shl 8).masked(8 .. 9))
  x.setMask((ARPE.uint32 shl 7).masked(7 .. 7))
  x.setMask((CMS shl 5).masked(5 .. 6))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((OPM.uint32 shl 3).masked(3 .. 3))
  x.setMask((URS.uint32 shl 2).masked(2 .. 2))
  x.setMask((UDIS.uint32 shl 1).masked(1 .. 1))
  x.setMask((CEN.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM1_CTLR1_Fields

template modifyIt*(reg: TIM1_CTLR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CTLR2_Type): TIM1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CTLR2_Fields](reg.loc))

proc read*(reg: static TIM1_CTLR2_Type): TIM1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CTLR2_Fields](reg.loc))

proc write*(reg: TIM1_CTLR2_Type, val: TIM1_CTLR2_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CTLR2_Fields](reg.loc), val)

proc write*(reg: TIM1_CTLR2_Type, OIS4: bool = false, OIS3N: bool = false, OIS3: bool = false, OIS2N: bool = false, OIS2: bool = false, OIS1N: bool = false, OIS1: bool = false, TI1S: bool = false, MMS: uint32 = 0, CCDS: bool = false, CCUS: bool = false, CCPC: bool = false) =
  var x: uint32
  x.setMask((OIS4.uint32 shl 14).masked(14 .. 14))
  x.setMask((OIS3N.uint32 shl 13).masked(13 .. 13))
  x.setMask((OIS3.uint32 shl 12).masked(12 .. 12))
  x.setMask((OIS2N.uint32 shl 11).masked(11 .. 11))
  x.setMask((OIS2.uint32 shl 10).masked(10 .. 10))
  x.setMask((OIS1N.uint32 shl 9).masked(9 .. 9))
  x.setMask((OIS1.uint32 shl 8).masked(8 .. 8))
  x.setMask((TI1S.uint32 shl 7).masked(7 .. 7))
  x.setMask((MMS shl 4).masked(4 .. 6))
  x.setMask((CCDS.uint32 shl 3).masked(3 .. 3))
  x.setMask((CCUS.uint32 shl 2).masked(2 .. 2))
  x.setMask((CCPC.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM1_CTLR2_Fields

template modifyIt*(reg: TIM1_CTLR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_SMCFGR_Type): TIM1_SMCFGR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_SMCFGR_Fields](reg.loc))

proc read*(reg: static TIM1_SMCFGR_Type): TIM1_SMCFGR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_SMCFGR_Fields](reg.loc))

proc write*(reg: TIM1_SMCFGR_Type, val: TIM1_SMCFGR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_SMCFGR_Fields](reg.loc), val)

proc write*(reg: TIM1_SMCFGR_Type, ECE: bool = false, ETPS: uint32 = 0, ETF: uint32 = 0, MSM: bool = false, TS: uint32 = 0, SMS: uint32 = 0) =
  var x: uint32
  x.setMask((ECE.uint32 shl 14).masked(14 .. 14))
  x.setMask((ETPS shl 12).masked(12 .. 13))
  x.setMask((ETF shl 8).masked(8 .. 11))
  x.setMask((MSM.uint32 shl 7).masked(7 .. 7))
  x.setMask((TS shl 4).masked(4 .. 6))
  x.setMask((SMS shl 0).masked(0 .. 2))
  reg.write x.TIM1_SMCFGR_Fields

template modifyIt*(reg: TIM1_SMCFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_DMAINTENR_Type): TIM1_DMAINTENR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_DMAINTENR_Fields](reg.loc))

proc read*(reg: static TIM1_DMAINTENR_Type): TIM1_DMAINTENR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_DMAINTENR_Fields](reg.loc))

proc write*(reg: TIM1_DMAINTENR_Type, val: TIM1_DMAINTENR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_DMAINTENR_Fields](reg.loc), val)

proc write*(reg: TIM1_DMAINTENR_Type, TDE: bool = false, COMDE: bool = false, CC4DE: bool = false, CC3DE: bool = false, CC2DE: bool = false, CC1DE: bool = false, UDE: bool = false, BIE: bool = false, TIE: bool = false, COMIE: bool = false, CC4IE: bool = false, CC3IE: bool = false, CC2IE: bool = false, CC1IE: bool = false, UIE: bool = false) =
  var x: uint32
  x.setMask((TDE.uint32 shl 14).masked(14 .. 14))
  x.setMask((COMDE.uint32 shl 13).masked(13 .. 13))
  x.setMask((CC4DE.uint32 shl 12).masked(12 .. 12))
  x.setMask((CC3DE.uint32 shl 11).masked(11 .. 11))
  x.setMask((CC2DE.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC1DE.uint32 shl 9).masked(9 .. 9))
  x.setMask((UDE.uint32 shl 8).masked(8 .. 8))
  x.setMask((BIE.uint32 shl 7).masked(7 .. 7))
  x.setMask((TIE.uint32 shl 6).masked(6 .. 6))
  x.setMask((COMIE.uint32 shl 5).masked(5 .. 5))
  x.setMask((CC4IE.uint32 shl 4).masked(4 .. 4))
  x.setMask((CC3IE.uint32 shl 3).masked(3 .. 3))
  x.setMask((CC2IE.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1IE.uint32 shl 1).masked(1 .. 1))
  x.setMask((UIE.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM1_DMAINTENR_Fields

template modifyIt*(reg: TIM1_DMAINTENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_INTFR_Type): TIM1_INTFR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_INTFR_Fields](reg.loc))

proc read*(reg: static TIM1_INTFR_Type): TIM1_INTFR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_INTFR_Fields](reg.loc))

proc write*(reg: TIM1_INTFR_Type, val: TIM1_INTFR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_INTFR_Fields](reg.loc), val)

proc write*(reg: TIM1_INTFR_Type, CC4OF: bool = false, CC3OF: bool = false, CC2OF: bool = false, CC1OF: bool = false, BIF: bool = false, TIF: bool = false, COMIF: bool = false, CC4IF: bool = false, CC3IF: bool = false, CC2IF: bool = false, CC1IF: bool = false, UIF: bool = false) =
  var x: uint32
  x.setMask((CC4OF.uint32 shl 12).masked(12 .. 12))
  x.setMask((CC3OF.uint32 shl 11).masked(11 .. 11))
  x.setMask((CC2OF.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC1OF.uint32 shl 9).masked(9 .. 9))
  x.setMask((BIF.uint32 shl 7).masked(7 .. 7))
  x.setMask((TIF.uint32 shl 6).masked(6 .. 6))
  x.setMask((COMIF.uint32 shl 5).masked(5 .. 5))
  x.setMask((CC4IF.uint32 shl 4).masked(4 .. 4))
  x.setMask((CC3IF.uint32 shl 3).masked(3 .. 3))
  x.setMask((CC2IF.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1IF.uint32 shl 1).masked(1 .. 1))
  x.setMask((UIF.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM1_INTFR_Fields

template modifyIt*(reg: TIM1_INTFR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc write*(reg: TIM1_SWEVGR_Type, val: TIM1_SWEVGR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_SWEVGR_Fields](reg.loc), val)

proc write*(reg: TIM1_SWEVGR_Type, BG: bool = false, TG: bool = false, COMG: bool = false, CC4G: bool = false, CC3G: bool = false, CC2G: bool = false, CC1G: bool = false, UG: bool = false) =
  var x: uint32
  x.setMask((BG.uint32 shl 7).masked(7 .. 7))
  x.setMask((TG.uint32 shl 6).masked(6 .. 6))
  x.setMask((COMG.uint32 shl 5).masked(5 .. 5))
  x.setMask((CC4G.uint32 shl 4).masked(4 .. 4))
  x.setMask((CC3G.uint32 shl 3).masked(3 .. 3))
  x.setMask((CC2G.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1G.uint32 shl 1).masked(1 .. 1))
  x.setMask((UG.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM1_SWEVGR_Fields

proc read*(reg: TIM1_CHCTLR1_Output_Type): TIM1_CHCTLR1_Output_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CHCTLR1_Output_Fields](reg.loc))

proc read*(reg: static TIM1_CHCTLR1_Output_Type): TIM1_CHCTLR1_Output_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CHCTLR1_Output_Fields](reg.loc))

proc write*(reg: TIM1_CHCTLR1_Output_Type, val: TIM1_CHCTLR1_Output_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CHCTLR1_Output_Fields](reg.loc), val)

proc write*(reg: TIM1_CHCTLR1_Output_Type, OC2CE: bool = false, OC2M: uint32 = 0, OC2PE: bool = false, OC2FE: bool = false, CC2S: uint32 = 0, OC1CE: bool = false, OC1M: uint32 = 0, OC1PE: bool = false, OC1FE: bool = false, CC1S: uint32 = 0) =
  var x: uint32
  x.setMask((OC2CE.uint32 shl 15).masked(15 .. 15))
  x.setMask((OC2M shl 12).masked(12 .. 14))
  x.setMask((OC2PE.uint32 shl 11).masked(11 .. 11))
  x.setMask((OC2FE.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC2S shl 8).masked(8 .. 9))
  x.setMask((OC1CE.uint32 shl 7).masked(7 .. 7))
  x.setMask((OC1M shl 4).masked(4 .. 6))
  x.setMask((OC1PE.uint32 shl 3).masked(3 .. 3))
  x.setMask((OC1FE.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1S shl 0).masked(0 .. 1))
  reg.write x.TIM1_CHCTLR1_Output_Fields

template modifyIt*(reg: TIM1_CHCTLR1_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CHCTLR1_Input_Type): TIM1_CHCTLR1_Input_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CHCTLR1_Input_Fields](reg.loc))

proc read*(reg: static TIM1_CHCTLR1_Input_Type): TIM1_CHCTLR1_Input_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CHCTLR1_Input_Fields](reg.loc))

proc write*(reg: TIM1_CHCTLR1_Input_Type, val: TIM1_CHCTLR1_Input_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CHCTLR1_Input_Fields](reg.loc), val)

proc write*(reg: TIM1_CHCTLR1_Input_Type, IC2F: uint32 = 0, IC2PCS: uint32 = 0, CC2S: uint32 = 0, IC1F: uint32 = 0, IC1PSC: uint32 = 0, CC1S: uint32 = 0) =
  var x: uint32
  x.setMask((IC2F shl 12).masked(12 .. 15))
  x.setMask((IC2PCS shl 10).masked(10 .. 11))
  x.setMask((CC2S shl 8).masked(8 .. 9))
  x.setMask((IC1F shl 4).masked(4 .. 7))
  x.setMask((IC1PSC shl 2).masked(2 .. 3))
  x.setMask((CC1S shl 0).masked(0 .. 1))
  reg.write x.TIM1_CHCTLR1_Input_Fields

template modifyIt*(reg: TIM1_CHCTLR1_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CHCTLR2_Output_Type): TIM1_CHCTLR2_Output_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CHCTLR2_Output_Fields](reg.loc))

proc read*(reg: static TIM1_CHCTLR2_Output_Type): TIM1_CHCTLR2_Output_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CHCTLR2_Output_Fields](reg.loc))

proc write*(reg: TIM1_CHCTLR2_Output_Type, val: TIM1_CHCTLR2_Output_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CHCTLR2_Output_Fields](reg.loc), val)

proc write*(reg: TIM1_CHCTLR2_Output_Type, OC4CE: bool = false, OC4M: uint32 = 0, OC4PE: bool = false, OC4FE: bool = false, CC4S: uint32 = 0, OC3CE: bool = false, OC3M: uint32 = 0, OC3PE: bool = false, OC3FE: bool = false, CC3S: uint32 = 0) =
  var x: uint32
  x.setMask((OC4CE.uint32 shl 15).masked(15 .. 15))
  x.setMask((OC4M shl 12).masked(12 .. 14))
  x.setMask((OC4PE.uint32 shl 11).masked(11 .. 11))
  x.setMask((OC4FE.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC4S shl 8).masked(8 .. 9))
  x.setMask((OC3CE.uint32 shl 7).masked(7 .. 7))
  x.setMask((OC3M shl 4).masked(4 .. 6))
  x.setMask((OC3PE.uint32 shl 3).masked(3 .. 3))
  x.setMask((OC3FE.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC3S shl 0).masked(0 .. 1))
  reg.write x.TIM1_CHCTLR2_Output_Fields

template modifyIt*(reg: TIM1_CHCTLR2_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CHCTLR2_Input_Type): TIM1_CHCTLR2_Input_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CHCTLR2_Input_Fields](reg.loc))

proc read*(reg: static TIM1_CHCTLR2_Input_Type): TIM1_CHCTLR2_Input_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CHCTLR2_Input_Fields](reg.loc))

proc write*(reg: TIM1_CHCTLR2_Input_Type, val: TIM1_CHCTLR2_Input_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CHCTLR2_Input_Fields](reg.loc), val)

proc write*(reg: TIM1_CHCTLR2_Input_Type, IC4F: uint32 = 0, IC4PSC: uint32 = 0, CC4S: uint32 = 0, IC3F: uint32 = 0, IC3PSC: uint32 = 0, CC3S: uint32 = 0) =
  var x: uint32
  x.setMask((IC4F shl 12).masked(12 .. 15))
  x.setMask((IC4PSC shl 10).masked(10 .. 11))
  x.setMask((CC4S shl 8).masked(8 .. 9))
  x.setMask((IC3F shl 4).masked(4 .. 7))
  x.setMask((IC3PSC shl 2).masked(2 .. 3))
  x.setMask((CC3S shl 0).masked(0 .. 1))
  reg.write x.TIM1_CHCTLR2_Input_Fields

template modifyIt*(reg: TIM1_CHCTLR2_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CCER_Type): TIM1_CCER_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CCER_Fields](reg.loc))

proc read*(reg: static TIM1_CCER_Type): TIM1_CCER_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CCER_Fields](reg.loc))

proc write*(reg: TIM1_CCER_Type, val: TIM1_CCER_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CCER_Fields](reg.loc), val)

proc write*(reg: TIM1_CCER_Type, CC4P: bool = false, CC4E: bool = false, CC3NP: bool = false, CC3NE: bool = false, CC3P: bool = false, CC3E: bool = false, CC2NP: bool = false, CC2NE: bool = false, CC2P: bool = false, CC2E: bool = false, CC1NP: bool = false, CC1NE: bool = false, CC1P: bool = false, CC1E: bool = false) =
  var x: uint32
  x.setMask((CC4P.uint32 shl 13).masked(13 .. 13))
  x.setMask((CC4E.uint32 shl 12).masked(12 .. 12))
  x.setMask((CC3NP.uint32 shl 11).masked(11 .. 11))
  x.setMask((CC3NE.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC3P.uint32 shl 9).masked(9 .. 9))
  x.setMask((CC3E.uint32 shl 8).masked(8 .. 8))
  x.setMask((CC2NP.uint32 shl 7).masked(7 .. 7))
  x.setMask((CC2NE.uint32 shl 6).masked(6 .. 6))
  x.setMask((CC2P.uint32 shl 5).masked(5 .. 5))
  x.setMask((CC2E.uint32 shl 4).masked(4 .. 4))
  x.setMask((CC1NP.uint32 shl 3).masked(3 .. 3))
  x.setMask((CC1NE.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1P.uint32 shl 1).masked(1 .. 1))
  x.setMask((CC1E.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM1_CCER_Fields

template modifyIt*(reg: TIM1_CCER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CNT_Type): TIM1_CNT_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CNT_Fields](reg.loc))

proc read*(reg: static TIM1_CNT_Type): TIM1_CNT_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CNT_Fields](reg.loc))

proc write*(reg: TIM1_CNT_Type, val: TIM1_CNT_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CNT_Fields](reg.loc), val)

proc write*(reg: TIM1_CNT_Type, CNT: uint32 = 0) =
  var x: uint32
  x.setMask((CNT shl 0).masked(0 .. 15))
  reg.write x.TIM1_CNT_Fields

template modifyIt*(reg: TIM1_CNT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_PSC_Type): TIM1_PSC_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_PSC_Fields](reg.loc))

proc read*(reg: static TIM1_PSC_Type): TIM1_PSC_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_PSC_Fields](reg.loc))

proc write*(reg: TIM1_PSC_Type, val: TIM1_PSC_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_PSC_Fields](reg.loc), val)

proc write*(reg: TIM1_PSC_Type, PSC: uint32 = 0) =
  var x: uint32
  x.setMask((PSC shl 0).masked(0 .. 15))
  reg.write x.TIM1_PSC_Fields

template modifyIt*(reg: TIM1_PSC_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_ATRLR_Type): TIM1_ATRLR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_ATRLR_Fields](reg.loc))

proc read*(reg: static TIM1_ATRLR_Type): TIM1_ATRLR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_ATRLR_Fields](reg.loc))

proc write*(reg: TIM1_ATRLR_Type, val: TIM1_ATRLR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_ATRLR_Fields](reg.loc), val)

proc write*(reg: TIM1_ATRLR_Type, ATRLR: uint32 = 0) =
  var x: uint32
  x.setMask((ATRLR shl 0).masked(0 .. 15))
  reg.write x.TIM1_ATRLR_Fields

template modifyIt*(reg: TIM1_ATRLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_RPTCR_Type): TIM1_RPTCR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_RPTCR_Fields](reg.loc))

proc read*(reg: static TIM1_RPTCR_Type): TIM1_RPTCR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_RPTCR_Fields](reg.loc))

proc write*(reg: TIM1_RPTCR_Type, val: TIM1_RPTCR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_RPTCR_Fields](reg.loc), val)

proc write*(reg: TIM1_RPTCR_Type, RPTCR: uint32 = 0) =
  var x: uint32
  x.setMask((RPTCR shl 0).masked(0 .. 7))
  reg.write x.TIM1_RPTCR_Fields

template modifyIt*(reg: TIM1_RPTCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CH1CVR_Type): TIM1_CH1CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CH1CVR_Fields](reg.loc))

proc read*(reg: static TIM1_CH1CVR_Type): TIM1_CH1CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CH1CVR_Fields](reg.loc))

proc write*(reg: TIM1_CH1CVR_Type, val: TIM1_CH1CVR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CH1CVR_Fields](reg.loc), val)

proc write*(reg: TIM1_CH1CVR_Type, CH1CVR: uint32 = 0) =
  var x: uint32
  x.setMask((CH1CVR shl 0).masked(0 .. 15))
  reg.write x.TIM1_CH1CVR_Fields

template modifyIt*(reg: TIM1_CH1CVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CH2CVR_Type): TIM1_CH2CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CH2CVR_Fields](reg.loc))

proc read*(reg: static TIM1_CH2CVR_Type): TIM1_CH2CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CH2CVR_Fields](reg.loc))

proc write*(reg: TIM1_CH2CVR_Type, val: TIM1_CH2CVR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CH2CVR_Fields](reg.loc), val)

proc write*(reg: TIM1_CH2CVR_Type, CH2CVR: uint32 = 0) =
  var x: uint32
  x.setMask((CH2CVR shl 0).masked(0 .. 15))
  reg.write x.TIM1_CH2CVR_Fields

template modifyIt*(reg: TIM1_CH2CVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CH3CVR_Type): TIM1_CH3CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CH3CVR_Fields](reg.loc))

proc read*(reg: static TIM1_CH3CVR_Type): TIM1_CH3CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CH3CVR_Fields](reg.loc))

proc write*(reg: TIM1_CH3CVR_Type, val: TIM1_CH3CVR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CH3CVR_Fields](reg.loc), val)

proc write*(reg: TIM1_CH3CVR_Type, CH3CVR: uint32 = 0) =
  var x: uint32
  x.setMask((CH3CVR shl 0).masked(0 .. 15))
  reg.write x.TIM1_CH3CVR_Fields

template modifyIt*(reg: TIM1_CH3CVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_CH4CVR_Type): TIM1_CH4CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CH4CVR_Fields](reg.loc))

proc read*(reg: static TIM1_CH4CVR_Type): TIM1_CH4CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_CH4CVR_Fields](reg.loc))

proc write*(reg: TIM1_CH4CVR_Type, val: TIM1_CH4CVR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_CH4CVR_Fields](reg.loc), val)

proc write*(reg: TIM1_CH4CVR_Type, CH4CVR: uint32 = 0) =
  var x: uint32
  x.setMask((CH4CVR shl 0).masked(0 .. 15))
  reg.write x.TIM1_CH4CVR_Fields

template modifyIt*(reg: TIM1_CH4CVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_BDTR_Type): TIM1_BDTR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_BDTR_Fields](reg.loc))

proc read*(reg: static TIM1_BDTR_Type): TIM1_BDTR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_BDTR_Fields](reg.loc))

proc write*(reg: TIM1_BDTR_Type, val: TIM1_BDTR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_BDTR_Fields](reg.loc), val)

proc write*(reg: TIM1_BDTR_Type, MOE: bool = false, AOE: bool = false, BKP: bool = false, BKE: bool = false, OSSR: bool = false, OSSI: bool = false, LOCK: uint32 = 0, DTG: uint32 = 0) =
  var x: uint32
  x.setMask((MOE.uint32 shl 15).masked(15 .. 15))
  x.setMask((AOE.uint32 shl 14).masked(14 .. 14))
  x.setMask((BKP.uint32 shl 13).masked(13 .. 13))
  x.setMask((BKE.uint32 shl 12).masked(12 .. 12))
  x.setMask((OSSR.uint32 shl 11).masked(11 .. 11))
  x.setMask((OSSI.uint32 shl 10).masked(10 .. 10))
  x.setMask((LOCK shl 8).masked(8 .. 9))
  x.setMask((DTG shl 0).masked(0 .. 7))
  reg.write x.TIM1_BDTR_Fields

template modifyIt*(reg: TIM1_BDTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_DMACFGR_Type): TIM1_DMACFGR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_DMACFGR_Fields](reg.loc))

proc read*(reg: static TIM1_DMACFGR_Type): TIM1_DMACFGR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_DMACFGR_Fields](reg.loc))

proc write*(reg: TIM1_DMACFGR_Type, val: TIM1_DMACFGR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_DMACFGR_Fields](reg.loc), val)

proc write*(reg: TIM1_DMACFGR_Type, DBL: uint32 = 0, DBA: uint32 = 0) =
  var x: uint32
  x.setMask((DBL shl 8).masked(8 .. 12))
  x.setMask((DBA shl 0).masked(0 .. 4))
  reg.write x.TIM1_DMACFGR_Fields

template modifyIt*(reg: TIM1_DMACFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM1_DMAADR_Type): TIM1_DMAADR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_DMAADR_Fields](reg.loc))

proc read*(reg: static TIM1_DMAADR_Type): TIM1_DMAADR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM1_DMAADR_Fields](reg.loc))

proc write*(reg: TIM1_DMAADR_Type, val: TIM1_DMAADR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM1_DMAADR_Fields](reg.loc), val)

proc write*(reg: TIM1_DMAADR_Type, DMAADR: uint32 = 0) =
  var x: uint32
  x.setMask((DMAADR shl 0).masked(0 .. 15))
  reg.write x.TIM1_DMAADR_Fields

template modifyIt*(reg: TIM1_DMAADR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func TMR_CAP_LVL_EN*(r: TIM1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `TMR_CAP_LVL_EN=`*(r: var TIM1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.TIM1_CTLR1_Fields

func TMR_CAP_OV_EN*(r: TIM1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `TMR_CAP_OV_EN=`*(r: var TIM1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.TIM1_CTLR1_Fields

func CKD*(r: TIM1_CTLR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CKD=`*(r: var TIM1_CTLR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM1_CTLR1_Fields

func ARPE*(r: TIM1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `ARPE=`*(r: var TIM1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_CTLR1_Fields

func CMS*(r: TIM1_CTLR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(5 .. 6)

proc `CMS=`*(r: var TIM1_CTLR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 6)
  tmp.setMask((val shl 5).masked(5 .. 6))
  r = tmp.TIM1_CTLR1_Fields

func DIR*(r: TIM1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var TIM1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM1_CTLR1_Fields

func OPM*(r: TIM1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `OPM=`*(r: var TIM1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM1_CTLR1_Fields

func URS*(r: TIM1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `URS=`*(r: var TIM1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM1_CTLR1_Fields

func UDIS*(r: TIM1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `UDIS=`*(r: var TIM1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM1_CTLR1_Fields

func CEN*(r: TIM1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `CEN=`*(r: var TIM1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM1_CTLR1_Fields

func OIS4*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `OIS4=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.TIM1_CTLR2_Fields

func OIS3N*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(13 .. 13).bool

proc `OIS3N=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 13)
  tmp.setMask((val.uint32 shl 13).masked(13 .. 13))
  r = tmp.TIM1_CTLR2_Fields

func OIS3*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `OIS3=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.TIM1_CTLR2_Fields

func OIS2N*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `OIS2N=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM1_CTLR2_Fields

func OIS2*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `OIS2=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM1_CTLR2_Fields

func OIS1N*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `OIS1N=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.TIM1_CTLR2_Fields

func OIS1*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `OIS1=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.TIM1_CTLR2_Fields

func TI1S*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `TI1S=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_CTLR2_Fields

func MMS*(r: TIM1_CTLR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 6)

proc `MMS=`*(r: var TIM1_CTLR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 6)
  tmp.setMask((val shl 4).masked(4 .. 6))
  r = tmp.TIM1_CTLR2_Fields

func CCDS*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `CCDS=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM1_CTLR2_Fields

func CCUS*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `CCUS=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM1_CTLR2_Fields

func CCPC*(r: TIM1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `CCPC=`*(r: var TIM1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM1_CTLR2_Fields

func ETP*(r: TIM1_SMCFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

func ECE*(r: TIM1_SMCFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `ECE=`*(r: var TIM1_SMCFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.TIM1_SMCFGR_Fields

func ETPS*(r: TIM1_SMCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `ETPS=`*(r: var TIM1_SMCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.TIM1_SMCFGR_Fields

func ETF*(r: TIM1_SMCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 11)

proc `ETF=`*(r: var TIM1_SMCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 11)
  tmp.setMask((val shl 8).masked(8 .. 11))
  r = tmp.TIM1_SMCFGR_Fields

func MSM*(r: TIM1_SMCFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MSM=`*(r: var TIM1_SMCFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_SMCFGR_Fields

func TS*(r: TIM1_SMCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 6)

proc `TS=`*(r: var TIM1_SMCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 6)
  tmp.setMask((val shl 4).masked(4 .. 6))
  r = tmp.TIM1_SMCFGR_Fields

func SMS*(r: TIM1_SMCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 2)

proc `SMS=`*(r: var TIM1_SMCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 2)
  tmp.setMask((val shl 0).masked(0 .. 2))
  r = tmp.TIM1_SMCFGR_Fields

func TDE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `TDE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.TIM1_DMAINTENR_Fields

func COMDE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(13 .. 13).bool

proc `COMDE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 13)
  tmp.setMask((val.uint32 shl 13).masked(13 .. 13))
  r = tmp.TIM1_DMAINTENR_Fields

func CC4DE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `CC4DE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.TIM1_DMAINTENR_Fields

func CC3DE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `CC3DE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM1_DMAINTENR_Fields

func CC2DE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `CC2DE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM1_DMAINTENR_Fields

func CC1DE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CC1DE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.TIM1_DMAINTENR_Fields

func UDE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `UDE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.TIM1_DMAINTENR_Fields

func BIE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `BIE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_DMAINTENR_Fields

func TIE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `TIE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.TIM1_DMAINTENR_Fields

func COMIE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `COMIE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.TIM1_DMAINTENR_Fields

func CC4IE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `CC4IE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM1_DMAINTENR_Fields

func CC3IE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `CC3IE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM1_DMAINTENR_Fields

func CC2IE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `CC2IE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM1_DMAINTENR_Fields

func CC1IE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `CC1IE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM1_DMAINTENR_Fields

func UIE*(r: TIM1_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `UIE=`*(r: var TIM1_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM1_DMAINTENR_Fields

func CC4OF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `CC4OF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.TIM1_INTFR_Fields

func CC3OF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `CC3OF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM1_INTFR_Fields

func CC2OF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `CC2OF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM1_INTFR_Fields

func CC1OF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CC1OF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.TIM1_INTFR_Fields

func BIF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `BIF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_INTFR_Fields

func TIF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `TIF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.TIM1_INTFR_Fields

func COMIF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `COMIF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.TIM1_INTFR_Fields

func CC4IF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `CC4IF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM1_INTFR_Fields

func CC3IF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `CC3IF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM1_INTFR_Fields

func CC2IF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `CC2IF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM1_INTFR_Fields

func CC1IF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `CC1IF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM1_INTFR_Fields

func UIF*(r: TIM1_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `UIF=`*(r: var TIM1_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM1_INTFR_Fields

proc `BG=`*(r: var TIM1_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_SWEVGR_Fields

proc `TG=`*(r: var TIM1_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.TIM1_SWEVGR_Fields

proc `COMG=`*(r: var TIM1_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.TIM1_SWEVGR_Fields

proc `CC4G=`*(r: var TIM1_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM1_SWEVGR_Fields

proc `CC3G=`*(r: var TIM1_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM1_SWEVGR_Fields

proc `CC2G=`*(r: var TIM1_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM1_SWEVGR_Fields

proc `CC1G=`*(r: var TIM1_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM1_SWEVGR_Fields

proc `UG=`*(r: var TIM1_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM1_SWEVGR_Fields

func OC2CE*(r: TIM1_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `OC2CE=`*(r: var TIM1_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func OC2M*(r: TIM1_CHCTLR1_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 14)

proc `OC2M=`*(r: var TIM1_CHCTLR1_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 14)
  tmp.setMask((val shl 12).masked(12 .. 14))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func OC2PE*(r: TIM1_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `OC2PE=`*(r: var TIM1_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func OC2FE*(r: TIM1_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `OC2FE=`*(r: var TIM1_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func CC2S*(r: TIM1_CHCTLR1_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CC2S=`*(r: var TIM1_CHCTLR1_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func OC1CE*(r: TIM1_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `OC1CE=`*(r: var TIM1_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func OC1M*(r: TIM1_CHCTLR1_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 6)

proc `OC1M=`*(r: var TIM1_CHCTLR1_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 6)
  tmp.setMask((val shl 4).masked(4 .. 6))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func OC1PE*(r: TIM1_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `OC1PE=`*(r: var TIM1_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func OC1FE*(r: TIM1_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `OC1FE=`*(r: var TIM1_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func CC1S*(r: TIM1_CHCTLR1_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `CC1S=`*(r: var TIM1_CHCTLR1_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.TIM1_CHCTLR1_Output_Fields

func IC2F*(r: TIM1_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 15)

proc `IC2F=`*(r: var TIM1_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 15)
  tmp.setMask((val shl 12).masked(12 .. 15))
  r = tmp.TIM1_CHCTLR1_Input_Fields

func IC2PCS*(r: TIM1_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `IC2PCS=`*(r: var TIM1_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.TIM1_CHCTLR1_Input_Fields

func CC2S*(r: TIM1_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CC2S=`*(r: var TIM1_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM1_CHCTLR1_Input_Fields

func IC1F*(r: TIM1_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 7)

proc `IC1F=`*(r: var TIM1_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 7)
  tmp.setMask((val shl 4).masked(4 .. 7))
  r = tmp.TIM1_CHCTLR1_Input_Fields

func IC1PSC*(r: TIM1_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

proc `IC1PSC=`*(r: var TIM1_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.TIM1_CHCTLR1_Input_Fields

func CC1S*(r: TIM1_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `CC1S=`*(r: var TIM1_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.TIM1_CHCTLR1_Input_Fields

func OC4CE*(r: TIM1_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `OC4CE=`*(r: var TIM1_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func OC4M*(r: TIM1_CHCTLR2_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 14)

proc `OC4M=`*(r: var TIM1_CHCTLR2_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 14)
  tmp.setMask((val shl 12).masked(12 .. 14))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func OC4PE*(r: TIM1_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `OC4PE=`*(r: var TIM1_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func OC4FE*(r: TIM1_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `OC4FE=`*(r: var TIM1_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func CC4S*(r: TIM1_CHCTLR2_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CC4S=`*(r: var TIM1_CHCTLR2_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func OC3CE*(r: TIM1_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `OC3CE=`*(r: var TIM1_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func OC3M*(r: TIM1_CHCTLR2_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 6)

proc `OC3M=`*(r: var TIM1_CHCTLR2_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 6)
  tmp.setMask((val shl 4).masked(4 .. 6))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func OC3PE*(r: TIM1_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `OC3PE=`*(r: var TIM1_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func OC3FE*(r: TIM1_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `OC3FE=`*(r: var TIM1_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func CC3S*(r: TIM1_CHCTLR2_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `CC3S=`*(r: var TIM1_CHCTLR2_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.TIM1_CHCTLR2_Output_Fields

func IC4F*(r: TIM1_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 15)

proc `IC4F=`*(r: var TIM1_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 15)
  tmp.setMask((val shl 12).masked(12 .. 15))
  r = tmp.TIM1_CHCTLR2_Input_Fields

func IC4PSC*(r: TIM1_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `IC4PSC=`*(r: var TIM1_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.TIM1_CHCTLR2_Input_Fields

func CC4S*(r: TIM1_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CC4S=`*(r: var TIM1_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM1_CHCTLR2_Input_Fields

func IC3F*(r: TIM1_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 7)

proc `IC3F=`*(r: var TIM1_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 7)
  tmp.setMask((val shl 4).masked(4 .. 7))
  r = tmp.TIM1_CHCTLR2_Input_Fields

func IC3PSC*(r: TIM1_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

proc `IC3PSC=`*(r: var TIM1_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.TIM1_CHCTLR2_Input_Fields

func CC3S*(r: TIM1_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `CC3S=`*(r: var TIM1_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.TIM1_CHCTLR2_Input_Fields

func CC4P*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(13 .. 13).bool

proc `CC4P=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 13)
  tmp.setMask((val.uint32 shl 13).masked(13 .. 13))
  r = tmp.TIM1_CCER_Fields

func CC4E*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `CC4E=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.TIM1_CCER_Fields

func CC3NP*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `CC3NP=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM1_CCER_Fields

func CC3NE*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `CC3NE=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM1_CCER_Fields

func CC3P*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CC3P=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.TIM1_CCER_Fields

func CC3E*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `CC3E=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.TIM1_CCER_Fields

func CC2NP*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `CC2NP=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM1_CCER_Fields

func CC2NE*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `CC2NE=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.TIM1_CCER_Fields

func CC2P*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CC2P=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.TIM1_CCER_Fields

func CC2E*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `CC2E=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM1_CCER_Fields

func CC1NP*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `CC1NP=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM1_CCER_Fields

func CC1NE*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `CC1NE=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM1_CCER_Fields

func CC1P*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `CC1P=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM1_CCER_Fields

func CC1E*(r: TIM1_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `CC1E=`*(r: var TIM1_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM1_CCER_Fields

func CNT*(r: TIM1_CNT_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CNT=`*(r: var TIM1_CNT_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM1_CNT_Fields

func PSC*(r: TIM1_PSC_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `PSC=`*(r: var TIM1_PSC_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM1_PSC_Fields

func ATRLR*(r: TIM1_ATRLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `ATRLR=`*(r: var TIM1_ATRLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM1_ATRLR_Fields

func RPTCR*(r: TIM1_RPTCR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

proc `RPTCR=`*(r: var TIM1_RPTCR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.TIM1_RPTCR_Fields

func CH1CVR*(r: TIM1_CH1CVR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CH1CVR=`*(r: var TIM1_CH1CVR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM1_CH1CVR_Fields

func CH2CVR*(r: TIM1_CH2CVR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CH2CVR=`*(r: var TIM1_CH2CVR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM1_CH2CVR_Fields

func CH3CVR*(r: TIM1_CH3CVR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CH3CVR=`*(r: var TIM1_CH3CVR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM1_CH3CVR_Fields

func CH4CVR*(r: TIM1_CH4CVR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CH4CVR=`*(r: var TIM1_CH4CVR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM1_CH4CVR_Fields

func MOE*(r: TIM1_BDTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `MOE=`*(r: var TIM1_BDTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.TIM1_BDTR_Fields

func AOE*(r: TIM1_BDTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `AOE=`*(r: var TIM1_BDTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.TIM1_BDTR_Fields

func BKP*(r: TIM1_BDTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(13 .. 13).bool

proc `BKP=`*(r: var TIM1_BDTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 13)
  tmp.setMask((val.uint32 shl 13).masked(13 .. 13))
  r = tmp.TIM1_BDTR_Fields

func BKE*(r: TIM1_BDTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `BKE=`*(r: var TIM1_BDTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.TIM1_BDTR_Fields

func OSSR*(r: TIM1_BDTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `OSSR=`*(r: var TIM1_BDTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM1_BDTR_Fields

func OSSI*(r: TIM1_BDTR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `OSSI=`*(r: var TIM1_BDTR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM1_BDTR_Fields

func LOCK*(r: TIM1_BDTR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `LOCK=`*(r: var TIM1_BDTR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM1_BDTR_Fields

func DTG*(r: TIM1_BDTR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

proc `DTG=`*(r: var TIM1_BDTR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.TIM1_BDTR_Fields

func DBL*(r: TIM1_DMACFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 12)

proc `DBL=`*(r: var TIM1_DMACFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 12)
  tmp.setMask((val shl 8).masked(8 .. 12))
  r = tmp.TIM1_DMACFGR_Fields

func DBA*(r: TIM1_DMACFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 4)

proc `DBA=`*(r: var TIM1_DMACFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 4)
  tmp.setMask((val shl 0).masked(0 .. 4))
  r = tmp.TIM1_DMACFGR_Fields

func DMAADR*(r: TIM1_DMAADR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `DMAADR=`*(r: var TIM1_DMAADR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM1_DMAADR_Fields

type
  TIM2_CTLR1_Fields* = distinct uint32
  TIM2_CTLR2_Fields* = distinct uint32
  TIM2_SMCFGR_Fields* = distinct uint32
  TIM2_DMAINTENR_Fields* = distinct uint32
  TIM2_INTFR_Fields* = distinct uint32
  TIM2_SWEVGR_Fields* = distinct uint32
  TIM2_CHCTLR1_Output_Fields* = distinct uint32
  TIM2_CHCTLR1_Input_Fields* = distinct uint32
  TIM2_CHCTLR2_Output_Fields* = distinct uint32
  TIM2_CHCTLR2_Input_Fields* = distinct uint32
  TIM2_CCER_Fields* = distinct uint32
  TIM2_CNT_Fields* = distinct uint32
  TIM2_PSC_Fields* = distinct uint32
  TIM2_ATRLR_Fields* = distinct uint32
  TIM2_CH1CVR_Fields* = distinct uint32
  TIM2_CH2CVR_Fields* = distinct uint32
  TIM2_CH3CVR_Fields* = distinct uint32
  TIM2_CH4CVR_Fields* = distinct uint32
  TIM2_DMACFGR_Fields* = distinct uint32
  TIM2_DMAADR_Fields* = distinct uint32

proc read*(reg: TIM2_CTLR1_Type): TIM2_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CTLR1_Fields](reg.loc))

proc read*(reg: static TIM2_CTLR1_Type): TIM2_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CTLR1_Fields](reg.loc))

proc write*(reg: TIM2_CTLR1_Type, val: TIM2_CTLR1_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CTLR1_Fields](reg.loc), val)

proc write*(reg: TIM2_CTLR1_Type, TMR_CAP_LVL_EN: bool = false, TMR_CAP_OV_EN: bool = false, CKD: uint32 = 0, ARPE: bool = false, CMS: uint32 = 0, DIR: bool = false, OPM: bool = false, URS: bool = false, UDIS: bool = false, CEN: bool = false) =
  var x: uint32
  x.setMask((TMR_CAP_LVL_EN.uint32 shl 15).masked(15 .. 15))
  x.setMask((TMR_CAP_OV_EN.uint32 shl 14).masked(14 .. 14))
  x.setMask((CKD shl 8).masked(8 .. 9))
  x.setMask((ARPE.uint32 shl 7).masked(7 .. 7))
  x.setMask((CMS shl 5).masked(5 .. 6))
  x.setMask((DIR.uint32 shl 4).masked(4 .. 4))
  x.setMask((OPM.uint32 shl 3).masked(3 .. 3))
  x.setMask((URS.uint32 shl 2).masked(2 .. 2))
  x.setMask((UDIS.uint32 shl 1).masked(1 .. 1))
  x.setMask((CEN.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM2_CTLR1_Fields

template modifyIt*(reg: TIM2_CTLR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CTLR2_Type): TIM2_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CTLR2_Fields](reg.loc))

proc read*(reg: static TIM2_CTLR2_Type): TIM2_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CTLR2_Fields](reg.loc))

proc write*(reg: TIM2_CTLR2_Type, val: TIM2_CTLR2_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CTLR2_Fields](reg.loc), val)

proc write*(reg: TIM2_CTLR2_Type, TI1S: bool = false, MMS: uint32 = 0, CCDS: bool = false) =
  var x: uint32
  x.setMask((TI1S.uint32 shl 7).masked(7 .. 7))
  x.setMask((MMS shl 4).masked(4 .. 6))
  x.setMask((CCDS.uint32 shl 3).masked(3 .. 3))
  reg.write x.TIM2_CTLR2_Fields

template modifyIt*(reg: TIM2_CTLR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_SMCFGR_Type): TIM2_SMCFGR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_SMCFGR_Fields](reg.loc))

proc read*(reg: static TIM2_SMCFGR_Type): TIM2_SMCFGR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_SMCFGR_Fields](reg.loc))

proc write*(reg: TIM2_SMCFGR_Type, val: TIM2_SMCFGR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_SMCFGR_Fields](reg.loc), val)

proc write*(reg: TIM2_SMCFGR_Type, ETP: bool = false, ECE: bool = false, ETPS: uint32 = 0, ETF: uint32 = 0, MSM: bool = false, TS: uint32 = 0, SMS: uint32 = 0) =
  var x: uint32
  x.setMask((ETP.uint32 shl 15).masked(15 .. 15))
  x.setMask((ECE.uint32 shl 14).masked(14 .. 14))
  x.setMask((ETPS shl 12).masked(12 .. 13))
  x.setMask((ETF shl 8).masked(8 .. 11))
  x.setMask((MSM.uint32 shl 7).masked(7 .. 7))
  x.setMask((TS shl 4).masked(4 .. 6))
  x.setMask((SMS shl 0).masked(0 .. 2))
  reg.write x.TIM2_SMCFGR_Fields

template modifyIt*(reg: TIM2_SMCFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_DMAINTENR_Type): TIM2_DMAINTENR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_DMAINTENR_Fields](reg.loc))

proc read*(reg: static TIM2_DMAINTENR_Type): TIM2_DMAINTENR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_DMAINTENR_Fields](reg.loc))

proc write*(reg: TIM2_DMAINTENR_Type, val: TIM2_DMAINTENR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_DMAINTENR_Fields](reg.loc), val)

proc write*(reg: TIM2_DMAINTENR_Type, TDE: bool = false, CC4DE: bool = false, CC3DE: bool = false, CC2DE: bool = false, CC1DE: bool = false, UDE: bool = false, TIE: bool = false, CC4IE: bool = false, CC3IE: bool = false, CC2IE: bool = false, CC1IE: bool = false, UIE: bool = false) =
  var x: uint32
  x.setMask((TDE.uint32 shl 14).masked(14 .. 14))
  x.setMask((CC4DE.uint32 shl 12).masked(12 .. 12))
  x.setMask((CC3DE.uint32 shl 11).masked(11 .. 11))
  x.setMask((CC2DE.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC1DE.uint32 shl 9).masked(9 .. 9))
  x.setMask((UDE.uint32 shl 8).masked(8 .. 8))
  x.setMask((TIE.uint32 shl 6).masked(6 .. 6))
  x.setMask((CC4IE.uint32 shl 4).masked(4 .. 4))
  x.setMask((CC3IE.uint32 shl 3).masked(3 .. 3))
  x.setMask((CC2IE.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1IE.uint32 shl 1).masked(1 .. 1))
  x.setMask((UIE.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM2_DMAINTENR_Fields

template modifyIt*(reg: TIM2_DMAINTENR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_INTFR_Type): TIM2_INTFR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_INTFR_Fields](reg.loc))

proc read*(reg: static TIM2_INTFR_Type): TIM2_INTFR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_INTFR_Fields](reg.loc))

proc write*(reg: TIM2_INTFR_Type, val: TIM2_INTFR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_INTFR_Fields](reg.loc), val)

proc write*(reg: TIM2_INTFR_Type, CC4OF: bool = false, CC3OF: bool = false, CC2OF: bool = false, CC1OF: bool = false, TIF: bool = false, CC4IF: bool = false, CC3IF: bool = false, CC2IF: bool = false, CC1IF: bool = false, UIF: bool = false) =
  var x: uint32
  x.setMask((CC4OF.uint32 shl 12).masked(12 .. 12))
  x.setMask((CC3OF.uint32 shl 11).masked(11 .. 11))
  x.setMask((CC2OF.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC1OF.uint32 shl 9).masked(9 .. 9))
  x.setMask((TIF.uint32 shl 6).masked(6 .. 6))
  x.setMask((CC4IF.uint32 shl 4).masked(4 .. 4))
  x.setMask((CC3IF.uint32 shl 3).masked(3 .. 3))
  x.setMask((CC2IF.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1IF.uint32 shl 1).masked(1 .. 1))
  x.setMask((UIF.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM2_INTFR_Fields

template modifyIt*(reg: TIM2_INTFR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc write*(reg: TIM2_SWEVGR_Type, val: TIM2_SWEVGR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_SWEVGR_Fields](reg.loc), val)

proc write*(reg: TIM2_SWEVGR_Type, TG: bool = false, CC4G: bool = false, CC3G: bool = false, CC2G: bool = false, CC1G: bool = false, UG: bool = false) =
  var x: uint32
  x.setMask((TG.uint32 shl 6).masked(6 .. 6))
  x.setMask((CC4G.uint32 shl 4).masked(4 .. 4))
  x.setMask((CC3G.uint32 shl 3).masked(3 .. 3))
  x.setMask((CC2G.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1G.uint32 shl 1).masked(1 .. 1))
  x.setMask((UG.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM2_SWEVGR_Fields

proc read*(reg: TIM2_CHCTLR1_Output_Type): TIM2_CHCTLR1_Output_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CHCTLR1_Output_Fields](reg.loc))

proc read*(reg: static TIM2_CHCTLR1_Output_Type): TIM2_CHCTLR1_Output_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CHCTLR1_Output_Fields](reg.loc))

proc write*(reg: TIM2_CHCTLR1_Output_Type, val: TIM2_CHCTLR1_Output_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CHCTLR1_Output_Fields](reg.loc), val)

proc write*(reg: TIM2_CHCTLR1_Output_Type, OC2CE: bool = false, OC2M: uint32 = 0, OC2PE: bool = false, OC2FE: bool = false, CC2S: uint32 = 0, OC1CE: bool = false, OC1M: uint32 = 0, OC1PE: bool = false, OC1FE: bool = false, CC1S: uint32 = 0) =
  var x: uint32
  x.setMask((OC2CE.uint32 shl 15).masked(15 .. 15))
  x.setMask((OC2M shl 12).masked(12 .. 14))
  x.setMask((OC2PE.uint32 shl 11).masked(11 .. 11))
  x.setMask((OC2FE.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC2S shl 8).masked(8 .. 9))
  x.setMask((OC1CE.uint32 shl 7).masked(7 .. 7))
  x.setMask((OC1M shl 4).masked(4 .. 6))
  x.setMask((OC1PE.uint32 shl 3).masked(3 .. 3))
  x.setMask((OC1FE.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC1S shl 0).masked(0 .. 1))
  reg.write x.TIM2_CHCTLR1_Output_Fields

template modifyIt*(reg: TIM2_CHCTLR1_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CHCTLR1_Input_Type): TIM2_CHCTLR1_Input_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CHCTLR1_Input_Fields](reg.loc))

proc read*(reg: static TIM2_CHCTLR1_Input_Type): TIM2_CHCTLR1_Input_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CHCTLR1_Input_Fields](reg.loc))

proc write*(reg: TIM2_CHCTLR1_Input_Type, val: TIM2_CHCTLR1_Input_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CHCTLR1_Input_Fields](reg.loc), val)

proc write*(reg: TIM2_CHCTLR1_Input_Type, IC2F: uint32 = 0, IC2PSC: uint32 = 0, CC2S: uint32 = 0, IC1F: uint32 = 0, IC1PSC: uint32 = 0, CC1S: uint32 = 0) =
  var x: uint32
  x.setMask((IC2F shl 12).masked(12 .. 15))
  x.setMask((IC2PSC shl 10).masked(10 .. 11))
  x.setMask((CC2S shl 8).masked(8 .. 9))
  x.setMask((IC1F shl 4).masked(4 .. 7))
  x.setMask((IC1PSC shl 2).masked(2 .. 3))
  x.setMask((CC1S shl 0).masked(0 .. 1))
  reg.write x.TIM2_CHCTLR1_Input_Fields

template modifyIt*(reg: TIM2_CHCTLR1_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CHCTLR2_Output_Type): TIM2_CHCTLR2_Output_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CHCTLR2_Output_Fields](reg.loc))

proc read*(reg: static TIM2_CHCTLR2_Output_Type): TIM2_CHCTLR2_Output_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CHCTLR2_Output_Fields](reg.loc))

proc write*(reg: TIM2_CHCTLR2_Output_Type, val: TIM2_CHCTLR2_Output_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CHCTLR2_Output_Fields](reg.loc), val)

proc write*(reg: TIM2_CHCTLR2_Output_Type, OC4CE: bool = false, OC4M: uint32 = 0, OC4PE: bool = false, OC4FE: bool = false, CC4S: uint32 = 0, OC3CE: bool = false, OC3M: uint32 = 0, OC3PE: bool = false, OC3FE: bool = false, CC3S: uint32 = 0) =
  var x: uint32
  x.setMask((OC4CE.uint32 shl 15).masked(15 .. 15))
  x.setMask((OC4M shl 12).masked(12 .. 14))
  x.setMask((OC4PE.uint32 shl 11).masked(11 .. 11))
  x.setMask((OC4FE.uint32 shl 10).masked(10 .. 10))
  x.setMask((CC4S shl 8).masked(8 .. 9))
  x.setMask((OC3CE.uint32 shl 7).masked(7 .. 7))
  x.setMask((OC3M shl 4).masked(4 .. 6))
  x.setMask((OC3PE.uint32 shl 3).masked(3 .. 3))
  x.setMask((OC3FE.uint32 shl 2).masked(2 .. 2))
  x.setMask((CC3S shl 0).masked(0 .. 1))
  reg.write x.TIM2_CHCTLR2_Output_Fields

template modifyIt*(reg: TIM2_CHCTLR2_Output_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CHCTLR2_Input_Type): TIM2_CHCTLR2_Input_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CHCTLR2_Input_Fields](reg.loc))

proc read*(reg: static TIM2_CHCTLR2_Input_Type): TIM2_CHCTLR2_Input_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CHCTLR2_Input_Fields](reg.loc))

proc write*(reg: TIM2_CHCTLR2_Input_Type, val: TIM2_CHCTLR2_Input_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CHCTLR2_Input_Fields](reg.loc), val)

proc write*(reg: TIM2_CHCTLR2_Input_Type, IC4F: uint32 = 0, IC4PSC: uint32 = 0, CC4S: uint32 = 0, IC3F: uint32 = 0, IC3PSC: uint32 = 0, CC3S: uint32 = 0) =
  var x: uint32
  x.setMask((IC4F shl 12).masked(12 .. 15))
  x.setMask((IC4PSC shl 10).masked(10 .. 11))
  x.setMask((CC4S shl 8).masked(8 .. 9))
  x.setMask((IC3F shl 4).masked(4 .. 7))
  x.setMask((IC3PSC shl 2).masked(2 .. 3))
  x.setMask((CC3S shl 0).masked(0 .. 1))
  reg.write x.TIM2_CHCTLR2_Input_Fields

template modifyIt*(reg: TIM2_CHCTLR2_Input_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CCER_Type): TIM2_CCER_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CCER_Fields](reg.loc))

proc read*(reg: static TIM2_CCER_Type): TIM2_CCER_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CCER_Fields](reg.loc))

proc write*(reg: TIM2_CCER_Type, val: TIM2_CCER_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CCER_Fields](reg.loc), val)

proc write*(reg: TIM2_CCER_Type, CC4P: bool = false, CC4E: bool = false, CC3P: bool = false, CC3E: bool = false, CC2P: bool = false, CC2E: bool = false, CC1P: bool = false, CC1E: bool = false) =
  var x: uint32
  x.setMask((CC4P.uint32 shl 13).masked(13 .. 13))
  x.setMask((CC4E.uint32 shl 12).masked(12 .. 12))
  x.setMask((CC3P.uint32 shl 9).masked(9 .. 9))
  x.setMask((CC3E.uint32 shl 8).masked(8 .. 8))
  x.setMask((CC2P.uint32 shl 5).masked(5 .. 5))
  x.setMask((CC2E.uint32 shl 4).masked(4 .. 4))
  x.setMask((CC1P.uint32 shl 1).masked(1 .. 1))
  x.setMask((CC1E.uint32 shl 0).masked(0 .. 0))
  reg.write x.TIM2_CCER_Fields

template modifyIt*(reg: TIM2_CCER_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CNT_Type): TIM2_CNT_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CNT_Fields](reg.loc))

proc read*(reg: static TIM2_CNT_Type): TIM2_CNT_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CNT_Fields](reg.loc))

proc write*(reg: TIM2_CNT_Type, val: TIM2_CNT_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CNT_Fields](reg.loc), val)

proc write*(reg: TIM2_CNT_Type, CNT: uint32 = 0) =
  var x: uint32
  x.setMask((CNT shl 0).masked(0 .. 15))
  reg.write x.TIM2_CNT_Fields

template modifyIt*(reg: TIM2_CNT_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_PSC_Type): TIM2_PSC_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_PSC_Fields](reg.loc))

proc read*(reg: static TIM2_PSC_Type): TIM2_PSC_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_PSC_Fields](reg.loc))

proc write*(reg: TIM2_PSC_Type, val: TIM2_PSC_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_PSC_Fields](reg.loc), val)

proc write*(reg: TIM2_PSC_Type, PSC: uint32 = 0) =
  var x: uint32
  x.setMask((PSC shl 0).masked(0 .. 15))
  reg.write x.TIM2_PSC_Fields

template modifyIt*(reg: TIM2_PSC_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_ATRLR_Type): TIM2_ATRLR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_ATRLR_Fields](reg.loc))

proc read*(reg: static TIM2_ATRLR_Type): TIM2_ATRLR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_ATRLR_Fields](reg.loc))

proc write*(reg: TIM2_ATRLR_Type, val: TIM2_ATRLR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_ATRLR_Fields](reg.loc), val)

proc write*(reg: TIM2_ATRLR_Type, ATRLR: uint32 = 0) =
  var x: uint32
  x.setMask((ATRLR shl 0).masked(0 .. 15))
  reg.write x.TIM2_ATRLR_Fields

template modifyIt*(reg: TIM2_ATRLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CH1CVR_Type): TIM2_CH1CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CH1CVR_Fields](reg.loc))

proc read*(reg: static TIM2_CH1CVR_Type): TIM2_CH1CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CH1CVR_Fields](reg.loc))

proc write*(reg: TIM2_CH1CVR_Type, val: TIM2_CH1CVR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CH1CVR_Fields](reg.loc), val)

proc write*(reg: TIM2_CH1CVR_Type, CH1CVR: uint32 = 0) =
  var x: uint32
  x.setMask((CH1CVR shl 0).masked(0 .. 15))
  reg.write x.TIM2_CH1CVR_Fields

template modifyIt*(reg: TIM2_CH1CVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CH2CVR_Type): TIM2_CH2CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CH2CVR_Fields](reg.loc))

proc read*(reg: static TIM2_CH2CVR_Type): TIM2_CH2CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CH2CVR_Fields](reg.loc))

proc write*(reg: TIM2_CH2CVR_Type, val: TIM2_CH2CVR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CH2CVR_Fields](reg.loc), val)

proc write*(reg: TIM2_CH2CVR_Type, CH2CVR: uint32 = 0) =
  var x: uint32
  x.setMask((CH2CVR shl 0).masked(0 .. 15))
  reg.write x.TIM2_CH2CVR_Fields

template modifyIt*(reg: TIM2_CH2CVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CH3CVR_Type): TIM2_CH3CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CH3CVR_Fields](reg.loc))

proc read*(reg: static TIM2_CH3CVR_Type): TIM2_CH3CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CH3CVR_Fields](reg.loc))

proc write*(reg: TIM2_CH3CVR_Type, val: TIM2_CH3CVR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CH3CVR_Fields](reg.loc), val)

proc write*(reg: TIM2_CH3CVR_Type, CH3CVR: uint32 = 0) =
  var x: uint32
  x.setMask((CH3CVR shl 0).masked(0 .. 15))
  reg.write x.TIM2_CH3CVR_Fields

template modifyIt*(reg: TIM2_CH3CVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_CH4CVR_Type): TIM2_CH4CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CH4CVR_Fields](reg.loc))

proc read*(reg: static TIM2_CH4CVR_Type): TIM2_CH4CVR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_CH4CVR_Fields](reg.loc))

proc write*(reg: TIM2_CH4CVR_Type, val: TIM2_CH4CVR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_CH4CVR_Fields](reg.loc), val)

proc write*(reg: TIM2_CH4CVR_Type, CH4CVR: uint32 = 0) =
  var x: uint32
  x.setMask((CH4CVR shl 0).masked(0 .. 15))
  reg.write x.TIM2_CH4CVR_Fields

template modifyIt*(reg: TIM2_CH4CVR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_DMACFGR_Type): TIM2_DMACFGR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_DMACFGR_Fields](reg.loc))

proc read*(reg: static TIM2_DMACFGR_Type): TIM2_DMACFGR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_DMACFGR_Fields](reg.loc))

proc write*(reg: TIM2_DMACFGR_Type, val: TIM2_DMACFGR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_DMACFGR_Fields](reg.loc), val)

proc write*(reg: TIM2_DMACFGR_Type, DBL: uint32 = 0, DBA: uint32 = 0) =
  var x: uint32
  x.setMask((DBL shl 8).masked(8 .. 12))
  x.setMask((DBA shl 0).masked(0 .. 4))
  reg.write x.TIM2_DMACFGR_Fields

template modifyIt*(reg: TIM2_DMACFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: TIM2_DMAADR_Type): TIM2_DMAADR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_DMAADR_Fields](reg.loc))

proc read*(reg: static TIM2_DMAADR_Type): TIM2_DMAADR_Fields {.inline.} =
  volatileLoad(cast[ptr TIM2_DMAADR_Fields](reg.loc))

proc write*(reg: TIM2_DMAADR_Type, val: TIM2_DMAADR_Fields) {.inline.} =
  volatileStore(cast[ptr TIM2_DMAADR_Fields](reg.loc), val)

proc write*(reg: TIM2_DMAADR_Type, DMAADR: uint32 = 0) =
  var x: uint32
  x.setMask((DMAADR shl 0).masked(0 .. 15))
  reg.write x.TIM2_DMAADR_Fields

template modifyIt*(reg: TIM2_DMAADR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func TMR_CAP_LVL_EN*(r: TIM2_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `TMR_CAP_LVL_EN=`*(r: var TIM2_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.TIM2_CTLR1_Fields

func TMR_CAP_OV_EN*(r: TIM2_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `TMR_CAP_OV_EN=`*(r: var TIM2_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.TIM2_CTLR1_Fields

func CKD*(r: TIM2_CTLR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CKD=`*(r: var TIM2_CTLR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM2_CTLR1_Fields

func ARPE*(r: TIM2_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `ARPE=`*(r: var TIM2_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM2_CTLR1_Fields

func CMS*(r: TIM2_CTLR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(5 .. 6)

proc `CMS=`*(r: var TIM2_CTLR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 6)
  tmp.setMask((val shl 5).masked(5 .. 6))
  r = tmp.TIM2_CTLR1_Fields

func DIR*(r: TIM2_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DIR=`*(r: var TIM2_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM2_CTLR1_Fields

func OPM*(r: TIM2_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `OPM=`*(r: var TIM2_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM2_CTLR1_Fields

func URS*(r: TIM2_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `URS=`*(r: var TIM2_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM2_CTLR1_Fields

func UDIS*(r: TIM2_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `UDIS=`*(r: var TIM2_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM2_CTLR1_Fields

func CEN*(r: TIM2_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `CEN=`*(r: var TIM2_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM2_CTLR1_Fields

func TI1S*(r: TIM2_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `TI1S=`*(r: var TIM2_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM2_CTLR2_Fields

func MMS*(r: TIM2_CTLR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 6)

proc `MMS=`*(r: var TIM2_CTLR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 6)
  tmp.setMask((val shl 4).masked(4 .. 6))
  r = tmp.TIM2_CTLR2_Fields

func CCDS*(r: TIM2_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `CCDS=`*(r: var TIM2_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM2_CTLR2_Fields

func ETP*(r: TIM2_SMCFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `ETP=`*(r: var TIM2_SMCFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.TIM2_SMCFGR_Fields

func ECE*(r: TIM2_SMCFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `ECE=`*(r: var TIM2_SMCFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.TIM2_SMCFGR_Fields

func ETPS*(r: TIM2_SMCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `ETPS=`*(r: var TIM2_SMCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.TIM2_SMCFGR_Fields

func ETF*(r: TIM2_SMCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 11)

proc `ETF=`*(r: var TIM2_SMCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 11)
  tmp.setMask((val shl 8).masked(8 .. 11))
  r = tmp.TIM2_SMCFGR_Fields

func MSM*(r: TIM2_SMCFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `MSM=`*(r: var TIM2_SMCFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM2_SMCFGR_Fields

func TS*(r: TIM2_SMCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 6)

proc `TS=`*(r: var TIM2_SMCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 6)
  tmp.setMask((val shl 4).masked(4 .. 6))
  r = tmp.TIM2_SMCFGR_Fields

func SMS*(r: TIM2_SMCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 2)

proc `SMS=`*(r: var TIM2_SMCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 2)
  tmp.setMask((val shl 0).masked(0 .. 2))
  r = tmp.TIM2_SMCFGR_Fields

func TDE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `TDE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.TIM2_DMAINTENR_Fields

func CC4DE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `CC4DE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.TIM2_DMAINTENR_Fields

func CC3DE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `CC3DE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM2_DMAINTENR_Fields

func CC2DE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `CC2DE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM2_DMAINTENR_Fields

func CC1DE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CC1DE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.TIM2_DMAINTENR_Fields

func UDE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `UDE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.TIM2_DMAINTENR_Fields

func TIE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `TIE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.TIM2_DMAINTENR_Fields

func CC4IE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `CC4IE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM2_DMAINTENR_Fields

func CC3IE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `CC3IE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM2_DMAINTENR_Fields

func CC2IE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `CC2IE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM2_DMAINTENR_Fields

func CC1IE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `CC1IE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM2_DMAINTENR_Fields

func UIE*(r: TIM2_DMAINTENR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `UIE=`*(r: var TIM2_DMAINTENR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM2_DMAINTENR_Fields

func CC4OF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `CC4OF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.TIM2_INTFR_Fields

func CC3OF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `CC3OF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM2_INTFR_Fields

func CC2OF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `CC2OF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM2_INTFR_Fields

func CC1OF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CC1OF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.TIM2_INTFR_Fields

func TIF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `TIF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.TIM2_INTFR_Fields

func CC4IF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `CC4IF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM2_INTFR_Fields

func CC3IF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `CC3IF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM2_INTFR_Fields

func CC2IF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `CC2IF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM2_INTFR_Fields

func CC1IF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `CC1IF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM2_INTFR_Fields

func UIF*(r: TIM2_INTFR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `UIF=`*(r: var TIM2_INTFR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM2_INTFR_Fields

proc `TG=`*(r: var TIM2_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.TIM2_SWEVGR_Fields

proc `CC4G=`*(r: var TIM2_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM2_SWEVGR_Fields

proc `CC3G=`*(r: var TIM2_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM2_SWEVGR_Fields

proc `CC2G=`*(r: var TIM2_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM2_SWEVGR_Fields

proc `CC1G=`*(r: var TIM2_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM2_SWEVGR_Fields

proc `UG=`*(r: var TIM2_SWEVGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM2_SWEVGR_Fields

func OC2CE*(r: TIM2_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `OC2CE=`*(r: var TIM2_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func OC2M*(r: TIM2_CHCTLR1_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 14)

proc `OC2M=`*(r: var TIM2_CHCTLR1_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 14)
  tmp.setMask((val shl 12).masked(12 .. 14))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func OC2PE*(r: TIM2_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `OC2PE=`*(r: var TIM2_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func OC2FE*(r: TIM2_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `OC2FE=`*(r: var TIM2_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func CC2S*(r: TIM2_CHCTLR1_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CC2S=`*(r: var TIM2_CHCTLR1_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func OC1CE*(r: TIM2_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `OC1CE=`*(r: var TIM2_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func OC1M*(r: TIM2_CHCTLR1_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 6)

proc `OC1M=`*(r: var TIM2_CHCTLR1_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 6)
  tmp.setMask((val shl 4).masked(4 .. 6))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func OC1PE*(r: TIM2_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `OC1PE=`*(r: var TIM2_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func OC1FE*(r: TIM2_CHCTLR1_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `OC1FE=`*(r: var TIM2_CHCTLR1_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func CC1S*(r: TIM2_CHCTLR1_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `CC1S=`*(r: var TIM2_CHCTLR1_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.TIM2_CHCTLR1_Output_Fields

func IC2F*(r: TIM2_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 15)

proc `IC2F=`*(r: var TIM2_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 15)
  tmp.setMask((val shl 12).masked(12 .. 15))
  r = tmp.TIM2_CHCTLR1_Input_Fields

func IC2PSC*(r: TIM2_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `IC2PSC=`*(r: var TIM2_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.TIM2_CHCTLR1_Input_Fields

func CC2S*(r: TIM2_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CC2S=`*(r: var TIM2_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM2_CHCTLR1_Input_Fields

func IC1F*(r: TIM2_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 7)

proc `IC1F=`*(r: var TIM2_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 7)
  tmp.setMask((val shl 4).masked(4 .. 7))
  r = tmp.TIM2_CHCTLR1_Input_Fields

func IC1PSC*(r: TIM2_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

proc `IC1PSC=`*(r: var TIM2_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.TIM2_CHCTLR1_Input_Fields

func CC1S*(r: TIM2_CHCTLR1_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `CC1S=`*(r: var TIM2_CHCTLR1_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.TIM2_CHCTLR1_Input_Fields

func OC4CE*(r: TIM2_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `OC4CE=`*(r: var TIM2_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func OC4M*(r: TIM2_CHCTLR2_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 14)

proc `OC4M=`*(r: var TIM2_CHCTLR2_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 14)
  tmp.setMask((val shl 12).masked(12 .. 14))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func OC4PE*(r: TIM2_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `OC4PE=`*(r: var TIM2_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func OC4FE*(r: TIM2_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `OC4FE=`*(r: var TIM2_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func CC4S*(r: TIM2_CHCTLR2_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CC4S=`*(r: var TIM2_CHCTLR2_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func OC3CE*(r: TIM2_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `OC3CE=`*(r: var TIM2_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func OC3M*(r: TIM2_CHCTLR2_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 6)

proc `OC3M=`*(r: var TIM2_CHCTLR2_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 6)
  tmp.setMask((val shl 4).masked(4 .. 6))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func OC3PE*(r: TIM2_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `OC3PE=`*(r: var TIM2_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func OC3FE*(r: TIM2_CHCTLR2_Output_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `OC3FE=`*(r: var TIM2_CHCTLR2_Output_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func CC3S*(r: TIM2_CHCTLR2_Output_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `CC3S=`*(r: var TIM2_CHCTLR2_Output_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.TIM2_CHCTLR2_Output_Fields

func IC4F*(r: TIM2_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 15)

proc `IC4F=`*(r: var TIM2_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 15)
  tmp.setMask((val shl 12).masked(12 .. 15))
  r = tmp.TIM2_CHCTLR2_Input_Fields

func IC4PSC*(r: TIM2_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 11)

proc `IC4PSC=`*(r: var TIM2_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 11)
  tmp.setMask((val shl 10).masked(10 .. 11))
  r = tmp.TIM2_CHCTLR2_Input_Fields

func CC4S*(r: TIM2_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `CC4S=`*(r: var TIM2_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.TIM2_CHCTLR2_Input_Fields

func IC3F*(r: TIM2_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 7)

proc `IC3F=`*(r: var TIM2_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 7)
  tmp.setMask((val shl 4).masked(4 .. 7))
  r = tmp.TIM2_CHCTLR2_Input_Fields

func IC3PSC*(r: TIM2_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

proc `IC3PSC=`*(r: var TIM2_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.TIM2_CHCTLR2_Input_Fields

func CC3S*(r: TIM2_CHCTLR2_Input_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 1)

proc `CC3S=`*(r: var TIM2_CHCTLR2_Input_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 1)
  tmp.setMask((val shl 0).masked(0 .. 1))
  r = tmp.TIM2_CHCTLR2_Input_Fields

func CC4P*(r: TIM2_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(13 .. 13).bool

proc `CC4P=`*(r: var TIM2_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 13)
  tmp.setMask((val.uint32 shl 13).masked(13 .. 13))
  r = tmp.TIM2_CCER_Fields

func CC4E*(r: TIM2_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `CC4E=`*(r: var TIM2_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.TIM2_CCER_Fields

func CC3P*(r: TIM2_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CC3P=`*(r: var TIM2_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.TIM2_CCER_Fields

func CC3E*(r: TIM2_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `CC3E=`*(r: var TIM2_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.TIM2_CCER_Fields

func CC2P*(r: TIM2_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `CC2P=`*(r: var TIM2_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.TIM2_CCER_Fields

func CC2E*(r: TIM2_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `CC2E=`*(r: var TIM2_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.TIM2_CCER_Fields

func CC1P*(r: TIM2_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `CC1P=`*(r: var TIM2_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.TIM2_CCER_Fields

func CC1E*(r: TIM2_CCER_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `CC1E=`*(r: var TIM2_CCER_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.TIM2_CCER_Fields

func CNT*(r: TIM2_CNT_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CNT=`*(r: var TIM2_CNT_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM2_CNT_Fields

func PSC*(r: TIM2_PSC_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `PSC=`*(r: var TIM2_PSC_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM2_PSC_Fields

func ATRLR*(r: TIM2_ATRLR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `ATRLR=`*(r: var TIM2_ATRLR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM2_ATRLR_Fields

func CH1CVR*(r: TIM2_CH1CVR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CH1CVR=`*(r: var TIM2_CH1CVR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM2_CH1CVR_Fields

func CH2CVR*(r: TIM2_CH2CVR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CH2CVR=`*(r: var TIM2_CH2CVR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM2_CH2CVR_Fields

func CH3CVR*(r: TIM2_CH3CVR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CH3CVR=`*(r: var TIM2_CH3CVR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM2_CH3CVR_Fields

func CH4CVR*(r: TIM2_CH4CVR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CH4CVR=`*(r: var TIM2_CH4CVR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM2_CH4CVR_Fields

func DBL*(r: TIM2_DMACFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 12)

proc `DBL=`*(r: var TIM2_DMACFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 12)
  tmp.setMask((val shl 8).masked(8 .. 12))
  r = tmp.TIM2_DMACFGR_Fields

func DBA*(r: TIM2_DMACFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 4)

proc `DBA=`*(r: var TIM2_DMACFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 4)
  tmp.setMask((val shl 0).masked(0 .. 4))
  r = tmp.TIM2_DMACFGR_Fields

func DMAADR*(r: TIM2_DMAADR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `DMAADR=`*(r: var TIM2_DMAADR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.TIM2_DMAADR_Fields

type
  I2C1_CTLR1_Fields* = distinct uint32
  I2C1_CTLR2_Fields* = distinct uint32
  I2C1_OADDR1_Fields* = distinct uint32
  I2C1_OADDR2_Fields* = distinct uint32
  I2C1_DATAR_Fields* = distinct uint32
  I2C1_STAR1_Fields* = distinct uint32
  I2C1_STAR2_Fields* = distinct uint32
  I2C1_CKCFGR_Fields* = distinct uint32

proc read*(reg: I2C1_CTLR1_Type): I2C1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_CTLR1_Fields](reg.loc))

proc read*(reg: static I2C1_CTLR1_Type): I2C1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_CTLR1_Fields](reg.loc))

proc write*(reg: I2C1_CTLR1_Type, val: I2C1_CTLR1_Fields) {.inline.} =
  volatileStore(cast[ptr I2C1_CTLR1_Fields](reg.loc), val)

proc write*(reg: I2C1_CTLR1_Type, SWRST: bool = false, PEC: bool = false, POS: bool = false, ACK: bool = false, STOP: bool = false, START: bool = false, NOSTRETCH: bool = false, ENGC: bool = false, ENPEC: bool = false, ENARP: bool = false, PE: bool = false) =
  var x: uint32
  x.setMask((SWRST.uint32 shl 15).masked(15 .. 15))
  x.setMask((PEC.uint32 shl 12).masked(12 .. 12))
  x.setMask((POS.uint32 shl 11).masked(11 .. 11))
  x.setMask((ACK.uint32 shl 10).masked(10 .. 10))
  x.setMask((STOP.uint32 shl 9).masked(9 .. 9))
  x.setMask((START.uint32 shl 8).masked(8 .. 8))
  x.setMask((NOSTRETCH.uint32 shl 7).masked(7 .. 7))
  x.setMask((ENGC.uint32 shl 6).masked(6 .. 6))
  x.setMask((ENPEC.uint32 shl 5).masked(5 .. 5))
  x.setMask((ENARP.uint32 shl 4).masked(4 .. 4))
  x.setMask((PE.uint32 shl 0).masked(0 .. 0))
  reg.write x.I2C1_CTLR1_Fields

template modifyIt*(reg: I2C1_CTLR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: I2C1_CTLR2_Type): I2C1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_CTLR2_Fields](reg.loc))

proc read*(reg: static I2C1_CTLR2_Type): I2C1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_CTLR2_Fields](reg.loc))

proc write*(reg: I2C1_CTLR2_Type, val: I2C1_CTLR2_Fields) {.inline.} =
  volatileStore(cast[ptr I2C1_CTLR2_Fields](reg.loc), val)

proc write*(reg: I2C1_CTLR2_Type, LAST: bool = false, DMAEN: bool = false, ITBUFEN: bool = false, ITEVTEN: bool = false, ITERREN: bool = false, FREQ: uint32 = 0) =
  var x: uint32
  x.setMask((LAST.uint32 shl 12).masked(12 .. 12))
  x.setMask((DMAEN.uint32 shl 11).masked(11 .. 11))
  x.setMask((ITBUFEN.uint32 shl 10).masked(10 .. 10))
  x.setMask((ITEVTEN.uint32 shl 9).masked(9 .. 9))
  x.setMask((ITERREN.uint32 shl 8).masked(8 .. 8))
  x.setMask((FREQ shl 0).masked(0 .. 5))
  reg.write x.I2C1_CTLR2_Fields

template modifyIt*(reg: I2C1_CTLR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: I2C1_OADDR1_Type): I2C1_OADDR1_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_OADDR1_Fields](reg.loc))

proc read*(reg: static I2C1_OADDR1_Type): I2C1_OADDR1_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_OADDR1_Fields](reg.loc))

proc write*(reg: I2C1_OADDR1_Type, val: I2C1_OADDR1_Fields) {.inline.} =
  volatileStore(cast[ptr I2C1_OADDR1_Fields](reg.loc), val)

proc write*(reg: I2C1_OADDR1_Type, ADDMODE: bool = false, ADD9_8: uint32 = 0, ADD7_1: uint32 = 0, ADD0: bool = false) =
  var x: uint32
  x.setMask((ADDMODE.uint32 shl 15).masked(15 .. 15))
  x.setMask((ADD9_8 shl 8).masked(8 .. 9))
  x.setMask((ADD7_1 shl 1).masked(1 .. 7))
  x.setMask((ADD0.uint32 shl 0).masked(0 .. 0))
  reg.write x.I2C1_OADDR1_Fields

template modifyIt*(reg: I2C1_OADDR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: I2C1_OADDR2_Type): I2C1_OADDR2_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_OADDR2_Fields](reg.loc))

proc read*(reg: static I2C1_OADDR2_Type): I2C1_OADDR2_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_OADDR2_Fields](reg.loc))

proc write*(reg: I2C1_OADDR2_Type, val: I2C1_OADDR2_Fields) {.inline.} =
  volatileStore(cast[ptr I2C1_OADDR2_Fields](reg.loc), val)

proc write*(reg: I2C1_OADDR2_Type, ADD2: uint32 = 0, ENDUAL: bool = false) =
  var x: uint32
  x.setMask((ADD2 shl 1).masked(1 .. 7))
  x.setMask((ENDUAL.uint32 shl 0).masked(0 .. 0))
  reg.write x.I2C1_OADDR2_Fields

template modifyIt*(reg: I2C1_OADDR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: I2C1_DATAR_Type): I2C1_DATAR_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_DATAR_Fields](reg.loc))

proc read*(reg: static I2C1_DATAR_Type): I2C1_DATAR_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_DATAR_Fields](reg.loc))

proc write*(reg: I2C1_DATAR_Type, val: I2C1_DATAR_Fields) {.inline.} =
  volatileStore(cast[ptr I2C1_DATAR_Fields](reg.loc), val)

proc write*(reg: I2C1_DATAR_Type, DATAR: uint32 = 0) =
  var x: uint32
  x.setMask((DATAR shl 0).masked(0 .. 7))
  reg.write x.I2C1_DATAR_Fields

template modifyIt*(reg: I2C1_DATAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: I2C1_STAR1_Type): I2C1_STAR1_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_STAR1_Fields](reg.loc))

proc read*(reg: static I2C1_STAR1_Type): I2C1_STAR1_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_STAR1_Fields](reg.loc))

proc write*(reg: I2C1_STAR1_Type, val: I2C1_STAR1_Fields) {.inline.} =
  volatileStore(cast[ptr I2C1_STAR1_Fields](reg.loc), val)

proc write*(reg: I2C1_STAR1_Type, PECERR: bool = false, OVR: bool = false, AF: bool = false, ARLO: bool = false, BERR: bool = false) =
  var x: uint32
  x.setMask((PECERR.uint32 shl 12).masked(12 .. 12))
  x.setMask((OVR.uint32 shl 11).masked(11 .. 11))
  x.setMask((AF.uint32 shl 10).masked(10 .. 10))
  x.setMask((ARLO.uint32 shl 9).masked(9 .. 9))
  x.setMask((BERR.uint32 shl 8).masked(8 .. 8))
  reg.write x.I2C1_STAR1_Fields

template modifyIt*(reg: I2C1_STAR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: I2C1_STAR2_Type): I2C1_STAR2_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_STAR2_Fields](reg.loc))

proc read*(reg: static I2C1_STAR2_Type): I2C1_STAR2_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_STAR2_Fields](reg.loc))

proc read*(reg: I2C1_CKCFGR_Type): I2C1_CKCFGR_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_CKCFGR_Fields](reg.loc))

proc read*(reg: static I2C1_CKCFGR_Type): I2C1_CKCFGR_Fields {.inline.} =
  volatileLoad(cast[ptr I2C1_CKCFGR_Fields](reg.loc))

proc write*(reg: I2C1_CKCFGR_Type, val: I2C1_CKCFGR_Fields) {.inline.} =
  volatileStore(cast[ptr I2C1_CKCFGR_Fields](reg.loc), val)

proc write*(reg: I2C1_CKCFGR_Type, F_S: bool = false, DUTY: bool = false, CCR: uint32 = 0) =
  var x: uint32
  x.setMask((F_S.uint32 shl 15).masked(15 .. 15))
  x.setMask((DUTY.uint32 shl 14).masked(14 .. 14))
  x.setMask((CCR shl 0).masked(0 .. 11))
  reg.write x.I2C1_CKCFGR_Fields

template modifyIt*(reg: I2C1_CKCFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func SWRST*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `SWRST=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.I2C1_CTLR1_Fields

func PEC*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `PEC=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.I2C1_CTLR1_Fields

func POS*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `POS=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.I2C1_CTLR1_Fields

func ACK*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `ACK=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.I2C1_CTLR1_Fields

func STOP*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `STOP=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.I2C1_CTLR1_Fields

func START*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `START=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.I2C1_CTLR1_Fields

func NOSTRETCH*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `NOSTRETCH=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.I2C1_CTLR1_Fields

func ENGC*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `ENGC=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.I2C1_CTLR1_Fields

func ENPEC*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `ENPEC=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.I2C1_CTLR1_Fields

func ENARP*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `ENARP=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.I2C1_CTLR1_Fields

func PE*(r: I2C1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `PE=`*(r: var I2C1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.I2C1_CTLR1_Fields

func LAST*(r: I2C1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `LAST=`*(r: var I2C1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.I2C1_CTLR2_Fields

func DMAEN*(r: I2C1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `DMAEN=`*(r: var I2C1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.I2C1_CTLR2_Fields

func ITBUFEN*(r: I2C1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `ITBUFEN=`*(r: var I2C1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.I2C1_CTLR2_Fields

func ITEVTEN*(r: I2C1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `ITEVTEN=`*(r: var I2C1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.I2C1_CTLR2_Fields

func ITERREN*(r: I2C1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `ITERREN=`*(r: var I2C1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.I2C1_CTLR2_Fields

func FREQ*(r: I2C1_CTLR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 5)

proc `FREQ=`*(r: var I2C1_CTLR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 5)
  tmp.setMask((val shl 0).masked(0 .. 5))
  r = tmp.I2C1_CTLR2_Fields

func ADDMODE*(r: I2C1_OADDR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `ADDMODE=`*(r: var I2C1_OADDR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.I2C1_OADDR1_Fields

func ADD9_8*(r: I2C1_OADDR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 9)

proc `ADD9_8=`*(r: var I2C1_OADDR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 9)
  tmp.setMask((val shl 8).masked(8 .. 9))
  r = tmp.I2C1_OADDR1_Fields

func ADD7_1*(r: I2C1_OADDR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(1 .. 7)

proc `ADD7_1=`*(r: var I2C1_OADDR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 7)
  tmp.setMask((val shl 1).masked(1 .. 7))
  r = tmp.I2C1_OADDR1_Fields

func ADD0*(r: I2C1_OADDR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `ADD0=`*(r: var I2C1_OADDR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.I2C1_OADDR1_Fields

func ADD2*(r: I2C1_OADDR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(1 .. 7)

proc `ADD2=`*(r: var I2C1_OADDR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 7)
  tmp.setMask((val shl 1).masked(1 .. 7))
  r = tmp.I2C1_OADDR2_Fields

func ENDUAL*(r: I2C1_OADDR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `ENDUAL=`*(r: var I2C1_OADDR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.I2C1_OADDR2_Fields

func DATAR*(r: I2C1_DATAR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

proc `DATAR=`*(r: var I2C1_DATAR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.I2C1_DATAR_Fields

func PECERR*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `PECERR=`*(r: var I2C1_STAR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.I2C1_STAR1_Fields

func OVR*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `OVR=`*(r: var I2C1_STAR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.I2C1_STAR1_Fields

func AF*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `AF=`*(r: var I2C1_STAR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.I2C1_STAR1_Fields

func ARLO*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `ARLO=`*(r: var I2C1_STAR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.I2C1_STAR1_Fields

func BERR*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `BERR=`*(r: var I2C1_STAR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.I2C1_STAR1_Fields

func TxE*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

func RxNE*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

func STOPF*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

func ADD10*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

func BTF*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func ADDRx*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func SB*(r: I2C1_STAR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func PEC*(r: I2C1_STAR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 15)

func DUALF*(r: I2C1_STAR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

func GENCALL*(r: I2C1_STAR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

func TRA*(r: I2C1_STAR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func BUSY*(r: I2C1_STAR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func MSL*(r: I2C1_STAR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func F_S*(r: I2C1_CKCFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `F_S=`*(r: var I2C1_CKCFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.I2C1_CKCFGR_Fields

func DUTY*(r: I2C1_CKCFGR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `DUTY=`*(r: var I2C1_CKCFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.I2C1_CKCFGR_Fields

func CCR*(r: I2C1_CKCFGR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 11)

proc `CCR=`*(r: var I2C1_CKCFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 11)
  tmp.setMask((val shl 0).masked(0 .. 11))
  r = tmp.I2C1_CKCFGR_Fields

type
  SPI1_CTLR1_Fields* = distinct uint32
  SPI1_CTLR2_Fields* = distinct uint32
  SPI1_STATR_Fields* = distinct uint32
  SPI1_DATAR_Fields* = distinct uint32
  SPI1_CRCR_Fields* = distinct uint32
  SPI1_RCRCR_Fields* = distinct uint32
  SPI1_TCRCR_Fields* = distinct uint32
  SPI1_HSCR_Fields* = distinct uint32

proc read*(reg: SPI1_CTLR1_Type): SPI1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_CTLR1_Fields](reg.loc))

proc read*(reg: static SPI1_CTLR1_Type): SPI1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_CTLR1_Fields](reg.loc))

proc write*(reg: SPI1_CTLR1_Type, val: SPI1_CTLR1_Fields) {.inline.} =
  volatileStore(cast[ptr SPI1_CTLR1_Fields](reg.loc), val)

proc write*(reg: SPI1_CTLR1_Type, BIDIMODE: bool = false, BIDIOE: bool = false, CRCEN: bool = false, CRCNEXT: bool = false, DFF: bool = false, RXONLY: bool = false, SSM: bool = false, SSI: bool = false, LSBFIRST: bool = false, SPE: bool = false, BR: uint32 = 0, MSTR: bool = false, CPOL: bool = false, CPHA: bool = false) =
  var x: uint32
  x.setMask((BIDIMODE.uint32 shl 15).masked(15 .. 15))
  x.setMask((BIDIOE.uint32 shl 14).masked(14 .. 14))
  x.setMask((CRCEN.uint32 shl 13).masked(13 .. 13))
  x.setMask((CRCNEXT.uint32 shl 12).masked(12 .. 12))
  x.setMask((DFF.uint32 shl 11).masked(11 .. 11))
  x.setMask((RXONLY.uint32 shl 10).masked(10 .. 10))
  x.setMask((SSM.uint32 shl 9).masked(9 .. 9))
  x.setMask((SSI.uint32 shl 8).masked(8 .. 8))
  x.setMask((LSBFIRST.uint32 shl 7).masked(7 .. 7))
  x.setMask((SPE.uint32 shl 6).masked(6 .. 6))
  x.setMask((BR shl 3).masked(3 .. 5))
  x.setMask((MSTR.uint32 shl 2).masked(2 .. 2))
  x.setMask((CPOL.uint32 shl 1).masked(1 .. 1))
  x.setMask((CPHA.uint32 shl 0).masked(0 .. 0))
  reg.write x.SPI1_CTLR1_Fields

template modifyIt*(reg: SPI1_CTLR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: SPI1_CTLR2_Type): SPI1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_CTLR2_Fields](reg.loc))

proc read*(reg: static SPI1_CTLR2_Type): SPI1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_CTLR2_Fields](reg.loc))

proc write*(reg: SPI1_CTLR2_Type, val: SPI1_CTLR2_Fields) {.inline.} =
  volatileStore(cast[ptr SPI1_CTLR2_Fields](reg.loc), val)

proc write*(reg: SPI1_CTLR2_Type, TXEIE: bool = false, RXNEIE: bool = false, ERRIE: bool = false, SSOE: bool = false, TXDMAEN: bool = false, RXDMAEN: bool = false) =
  var x: uint32
  x.setMask((TXEIE.uint32 shl 7).masked(7 .. 7))
  x.setMask((RXNEIE.uint32 shl 6).masked(6 .. 6))
  x.setMask((ERRIE.uint32 shl 5).masked(5 .. 5))
  x.setMask((SSOE.uint32 shl 2).masked(2 .. 2))
  x.setMask((TXDMAEN.uint32 shl 1).masked(1 .. 1))
  x.setMask((RXDMAEN.uint32 shl 0).masked(0 .. 0))
  reg.write x.SPI1_CTLR2_Fields

template modifyIt*(reg: SPI1_CTLR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: SPI1_STATR_Type): SPI1_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_STATR_Fields](reg.loc))

proc read*(reg: static SPI1_STATR_Type): SPI1_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_STATR_Fields](reg.loc))

proc write*(reg: SPI1_STATR_Type, val: SPI1_STATR_Fields) {.inline.} =
  volatileStore(cast[ptr SPI1_STATR_Fields](reg.loc), val)

proc write*(reg: SPI1_STATR_Type, OVR: bool = false, CRCERR: bool = false) =
  var x: uint32
  x.setMask((OVR.uint32 shl 6).masked(6 .. 6))
  x.setMask((CRCERR.uint32 shl 4).masked(4 .. 4))
  reg.write x.SPI1_STATR_Fields

template modifyIt*(reg: SPI1_STATR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: SPI1_DATAR_Type): SPI1_DATAR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_DATAR_Fields](reg.loc))

proc read*(reg: static SPI1_DATAR_Type): SPI1_DATAR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_DATAR_Fields](reg.loc))

proc write*(reg: SPI1_DATAR_Type, val: SPI1_DATAR_Fields) {.inline.} =
  volatileStore(cast[ptr SPI1_DATAR_Fields](reg.loc), val)

proc write*(reg: SPI1_DATAR_Type, DATAR: uint32 = 0) =
  var x: uint32
  x.setMask((DATAR shl 0).masked(0 .. 15))
  reg.write x.SPI1_DATAR_Fields

template modifyIt*(reg: SPI1_DATAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: SPI1_CRCR_Type): SPI1_CRCR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_CRCR_Fields](reg.loc))

proc read*(reg: static SPI1_CRCR_Type): SPI1_CRCR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_CRCR_Fields](reg.loc))

proc write*(reg: SPI1_CRCR_Type, val: SPI1_CRCR_Fields) {.inline.} =
  volatileStore(cast[ptr SPI1_CRCR_Fields](reg.loc), val)

proc write*(reg: SPI1_CRCR_Type, CRCPOLY: uint32 = 7) =
  var x: uint32
  x.setMask((CRCPOLY shl 0).masked(0 .. 15))
  reg.write x.SPI1_CRCR_Fields

template modifyIt*(reg: SPI1_CRCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: SPI1_RCRCR_Type): SPI1_RCRCR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_RCRCR_Fields](reg.loc))

proc read*(reg: static SPI1_RCRCR_Type): SPI1_RCRCR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_RCRCR_Fields](reg.loc))

proc read*(reg: SPI1_TCRCR_Type): SPI1_TCRCR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_TCRCR_Fields](reg.loc))

proc read*(reg: static SPI1_TCRCR_Type): SPI1_TCRCR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_TCRCR_Fields](reg.loc))

proc read*(reg: SPI1_HSCR_Type): SPI1_HSCR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_HSCR_Fields](reg.loc))

proc read*(reg: static SPI1_HSCR_Type): SPI1_HSCR_Fields {.inline.} =
  volatileLoad(cast[ptr SPI1_HSCR_Fields](reg.loc))

proc write*(reg: SPI1_HSCR_Type, val: SPI1_HSCR_Fields) {.inline.} =
  volatileStore(cast[ptr SPI1_HSCR_Fields](reg.loc), val)

proc write*(reg: SPI1_HSCR_Type, HSRXEN: bool = false) =
  var x: uint32
  x.setMask((HSRXEN.uint32 shl 0).masked(0 .. 0))
  reg.write x.SPI1_HSCR_Fields

template modifyIt*(reg: SPI1_HSCR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func BIDIMODE*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `BIDIMODE=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.SPI1_CTLR1_Fields

func BIDIOE*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `BIDIOE=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.SPI1_CTLR1_Fields

func CRCEN*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(13 .. 13).bool

proc `CRCEN=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 13)
  tmp.setMask((val.uint32 shl 13).masked(13 .. 13))
  r = tmp.SPI1_CTLR1_Fields

func CRCNEXT*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `CRCNEXT=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.SPI1_CTLR1_Fields

func DFF*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `DFF=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.SPI1_CTLR1_Fields

func RXONLY*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `RXONLY=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.SPI1_CTLR1_Fields

func SSM*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `SSM=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.SPI1_CTLR1_Fields

func SSI*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `SSI=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.SPI1_CTLR1_Fields

func LSBFIRST*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `LSBFIRST=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.SPI1_CTLR1_Fields

func SPE*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `SPE=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.SPI1_CTLR1_Fields

func BR*(r: SPI1_CTLR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(3 .. 5)

proc `BR=`*(r: var SPI1_CTLR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 5)
  tmp.setMask((val shl 3).masked(3 .. 5))
  r = tmp.SPI1_CTLR1_Fields

func MSTR*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `MSTR=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.SPI1_CTLR1_Fields

func CPOL*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `CPOL=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.SPI1_CTLR1_Fields

func CPHA*(r: SPI1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `CPHA=`*(r: var SPI1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.SPI1_CTLR1_Fields

func TXEIE*(r: SPI1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `TXEIE=`*(r: var SPI1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.SPI1_CTLR2_Fields

func RXNEIE*(r: SPI1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `RXNEIE=`*(r: var SPI1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.SPI1_CTLR2_Fields

func ERRIE*(r: SPI1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `ERRIE=`*(r: var SPI1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.SPI1_CTLR2_Fields

func SSOE*(r: SPI1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `SSOE=`*(r: var SPI1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.SPI1_CTLR2_Fields

func TXDMAEN*(r: SPI1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `TXDMAEN=`*(r: var SPI1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.SPI1_CTLR2_Fields

func RXDMAEN*(r: SPI1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `RXDMAEN=`*(r: var SPI1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.SPI1_CTLR2_Fields

func BSY*(r: SPI1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

func OVR*(r: SPI1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `OVR=`*(r: var SPI1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.SPI1_STATR_Fields

func MODF*(r: SPI1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

func CRCERR*(r: SPI1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `CRCERR=`*(r: var SPI1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.SPI1_STATR_Fields

func CHSID*(r: SPI1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func UDR*(r: SPI1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

func TXE*(r: SPI1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func RXNE*(r: SPI1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func DATAR*(r: SPI1_DATAR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `DATAR=`*(r: var SPI1_DATAR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.SPI1_DATAR_Fields

func CRCPOLY*(r: SPI1_CRCR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `CRCPOLY=`*(r: var SPI1_CRCR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 15)
  tmp.setMask((val shl 0).masked(0 .. 15))
  r = tmp.SPI1_CRCR_Fields

func RXCRC*(r: SPI1_RCRCR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

func TXCRC*(r: SPI1_TCRCR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

proc `HSRXEN=`*(r: var SPI1_HSCR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.SPI1_HSCR_Fields

type
  USART1_STATR_Fields* = distinct uint32
  USART1_DATAR_Fields* = distinct uint32
  USART1_BRR_Fields* = distinct uint32
  USART1_CTLR1_Fields* = distinct uint32
  USART1_CTLR2_Fields* = distinct uint32
  USART1_CTLR3_Fields* = distinct uint32
  USART1_GPR_Fields* = distinct uint32

proc read*(reg: USART1_STATR_Type): USART1_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_STATR_Fields](reg.loc))

proc read*(reg: static USART1_STATR_Type): USART1_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_STATR_Fields](reg.loc))

proc write*(reg: USART1_STATR_Type, val: USART1_STATR_Fields) {.inline.} =
  volatileStore(cast[ptr USART1_STATR_Fields](reg.loc), val)

proc write*(reg: USART1_STATR_Type, CTS: bool = false, LBD: bool = false, TC: bool = true, RXNE: bool = false) =
  var x: uint32
  x.setMask((CTS.uint32 shl 9).masked(9 .. 9))
  x.setMask((LBD.uint32 shl 8).masked(8 .. 8))
  x.setMask((TC.uint32 shl 6).masked(6 .. 6))
  x.setMask((RXNE.uint32 shl 5).masked(5 .. 5))
  reg.write x.USART1_STATR_Fields

template modifyIt*(reg: USART1_STATR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: USART1_DATAR_Type): USART1_DATAR_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_DATAR_Fields](reg.loc))

proc read*(reg: static USART1_DATAR_Type): USART1_DATAR_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_DATAR_Fields](reg.loc))

proc write*(reg: USART1_DATAR_Type, val: USART1_DATAR_Fields) {.inline.} =
  volatileStore(cast[ptr USART1_DATAR_Fields](reg.loc), val)

proc write*(reg: USART1_DATAR_Type, DR: uint32 = 0) =
  var x: uint32
  x.setMask((DR shl 0).masked(0 .. 8))
  reg.write x.USART1_DATAR_Fields

template modifyIt*(reg: USART1_DATAR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: USART1_BRR_Type): USART1_BRR_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_BRR_Fields](reg.loc))

proc read*(reg: static USART1_BRR_Type): USART1_BRR_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_BRR_Fields](reg.loc))

proc write*(reg: USART1_BRR_Type, val: USART1_BRR_Fields) {.inline.} =
  volatileStore(cast[ptr USART1_BRR_Fields](reg.loc), val)

proc write*(reg: USART1_BRR_Type, DIV_Mantissa: uint32 = 0, DIV_Fraction: uint32 = 0) =
  var x: uint32
  x.setMask((DIV_Mantissa shl 4).masked(4 .. 15))
  x.setMask((DIV_Fraction shl 0).masked(0 .. 3))
  reg.write x.USART1_BRR_Fields

template modifyIt*(reg: USART1_BRR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: USART1_CTLR1_Type): USART1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_CTLR1_Fields](reg.loc))

proc read*(reg: static USART1_CTLR1_Type): USART1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_CTLR1_Fields](reg.loc))

proc write*(reg: USART1_CTLR1_Type, val: USART1_CTLR1_Fields) {.inline.} =
  volatileStore(cast[ptr USART1_CTLR1_Fields](reg.loc), val)

proc write*(reg: USART1_CTLR1_Type, UE: bool = false, M: bool = false, WAKE: bool = false, PCE: bool = false, PS: bool = false, PEIE: bool = false, TXEIE: bool = false, TCIE: bool = false, RXNEIE: bool = false, IDLEIE: bool = false, TE: bool = false, RE: bool = false, RWU: bool = false, SBK: bool = false) =
  var x: uint32
  x.setMask((UE.uint32 shl 13).masked(13 .. 13))
  x.setMask((M.uint32 shl 12).masked(12 .. 12))
  x.setMask((WAKE.uint32 shl 11).masked(11 .. 11))
  x.setMask((PCE.uint32 shl 10).masked(10 .. 10))
  x.setMask((PS.uint32 shl 9).masked(9 .. 9))
  x.setMask((PEIE.uint32 shl 8).masked(8 .. 8))
  x.setMask((TXEIE.uint32 shl 7).masked(7 .. 7))
  x.setMask((TCIE.uint32 shl 6).masked(6 .. 6))
  x.setMask((RXNEIE.uint32 shl 5).masked(5 .. 5))
  x.setMask((IDLEIE.uint32 shl 4).masked(4 .. 4))
  x.setMask((TE.uint32 shl 3).masked(3 .. 3))
  x.setMask((RE.uint32 shl 2).masked(2 .. 2))
  x.setMask((RWU.uint32 shl 1).masked(1 .. 1))
  x.setMask((SBK.uint32 shl 0).masked(0 .. 0))
  reg.write x.USART1_CTLR1_Fields

template modifyIt*(reg: USART1_CTLR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: USART1_CTLR2_Type): USART1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_CTLR2_Fields](reg.loc))

proc read*(reg: static USART1_CTLR2_Type): USART1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_CTLR2_Fields](reg.loc))

proc write*(reg: USART1_CTLR2_Type, val: USART1_CTLR2_Fields) {.inline.} =
  volatileStore(cast[ptr USART1_CTLR2_Fields](reg.loc), val)

proc write*(reg: USART1_CTLR2_Type, LINEN: bool = false, STOP: uint32 = 0, CLKEN: bool = false, CPOL: bool = false, CPHA: bool = false, LBCL: bool = false, LBDIE: bool = false, LBDL: bool = false, ADD: uint32 = 0) =
  var x: uint32
  x.setMask((LINEN.uint32 shl 14).masked(14 .. 14))
  x.setMask((STOP shl 12).masked(12 .. 13))
  x.setMask((CLKEN.uint32 shl 11).masked(11 .. 11))
  x.setMask((CPOL.uint32 shl 10).masked(10 .. 10))
  x.setMask((CPHA.uint32 shl 9).masked(9 .. 9))
  x.setMask((LBCL.uint32 shl 8).masked(8 .. 8))
  x.setMask((LBDIE.uint32 shl 6).masked(6 .. 6))
  x.setMask((LBDL.uint32 shl 5).masked(5 .. 5))
  x.setMask((ADD shl 0).masked(0 .. 3))
  reg.write x.USART1_CTLR2_Fields

template modifyIt*(reg: USART1_CTLR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: USART1_CTLR3_Type): USART1_CTLR3_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_CTLR3_Fields](reg.loc))

proc read*(reg: static USART1_CTLR3_Type): USART1_CTLR3_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_CTLR3_Fields](reg.loc))

proc write*(reg: USART1_CTLR3_Type, val: USART1_CTLR3_Fields) {.inline.} =
  volatileStore(cast[ptr USART1_CTLR3_Fields](reg.loc), val)

proc write*(reg: USART1_CTLR3_Type, CTSIE: bool = false, CTSE: bool = false, RTSE: bool = false, DMAT: bool = false, DMAR: bool = false, SCEN: bool = false, NACK: bool = false, HDSEL: bool = false, IRLP: bool = false, IREN: bool = false, EIE: bool = false) =
  var x: uint32
  x.setMask((CTSIE.uint32 shl 10).masked(10 .. 10))
  x.setMask((CTSE.uint32 shl 9).masked(9 .. 9))
  x.setMask((RTSE.uint32 shl 8).masked(8 .. 8))
  x.setMask((DMAT.uint32 shl 7).masked(7 .. 7))
  x.setMask((DMAR.uint32 shl 6).masked(6 .. 6))
  x.setMask((SCEN.uint32 shl 5).masked(5 .. 5))
  x.setMask((NACK.uint32 shl 4).masked(4 .. 4))
  x.setMask((HDSEL.uint32 shl 3).masked(3 .. 3))
  x.setMask((IRLP.uint32 shl 2).masked(2 .. 2))
  x.setMask((IREN.uint32 shl 1).masked(1 .. 1))
  x.setMask((EIE.uint32 shl 0).masked(0 .. 0))
  reg.write x.USART1_CTLR3_Fields

template modifyIt*(reg: USART1_CTLR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: USART1_GPR_Type): USART1_GPR_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_GPR_Fields](reg.loc))

proc read*(reg: static USART1_GPR_Type): USART1_GPR_Fields {.inline.} =
  volatileLoad(cast[ptr USART1_GPR_Fields](reg.loc))

proc write*(reg: USART1_GPR_Type, val: USART1_GPR_Fields) {.inline.} =
  volatileStore(cast[ptr USART1_GPR_Fields](reg.loc), val)

proc write*(reg: USART1_GPR_Type, GT: uint32 = 0, PSC: uint32 = 0) =
  var x: uint32
  x.setMask((GT shl 8).masked(8 .. 15))
  x.setMask((PSC shl 0).masked(0 .. 7))
  reg.write x.USART1_GPR_Fields

template modifyIt*(reg: USART1_GPR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func CTS*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CTS=`*(r: var USART1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.USART1_STATR_Fields

func LBD*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `LBD=`*(r: var USART1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.USART1_STATR_Fields

func TXE*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

func TC*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `TC=`*(r: var USART1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.USART1_STATR_Fields

func RXNE*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `RXNE=`*(r: var USART1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.USART1_STATR_Fields

func IDLE*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

func ORE*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

func NE*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func FE*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func PE*(r: USART1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func DR*(r: USART1_DATAR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 8)

proc `DR=`*(r: var USART1_DATAR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 8)
  tmp.setMask((val shl 0).masked(0 .. 8))
  r = tmp.USART1_DATAR_Fields

func DIV_Mantissa*(r: USART1_BRR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(4 .. 15)

proc `DIV_Mantissa=`*(r: var USART1_BRR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 15)
  tmp.setMask((val shl 4).masked(4 .. 15))
  r = tmp.USART1_BRR_Fields

func DIV_Fraction*(r: USART1_BRR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 3)

proc `DIV_Fraction=`*(r: var USART1_BRR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 3)
  tmp.setMask((val shl 0).masked(0 .. 3))
  r = tmp.USART1_BRR_Fields

func UE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(13 .. 13).bool

proc `UE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 13)
  tmp.setMask((val.uint32 shl 13).masked(13 .. 13))
  r = tmp.USART1_CTLR1_Fields

func M*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `M=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.USART1_CTLR1_Fields

func WAKE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `WAKE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.USART1_CTLR1_Fields

func PCE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `PCE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.USART1_CTLR1_Fields

func PS*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `PS=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.USART1_CTLR1_Fields

func PEIE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `PEIE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.USART1_CTLR1_Fields

func TXEIE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `TXEIE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.USART1_CTLR1_Fields

func TCIE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `TCIE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.USART1_CTLR1_Fields

func RXNEIE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `RXNEIE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.USART1_CTLR1_Fields

func IDLEIE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `IDLEIE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.USART1_CTLR1_Fields

func TE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `TE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.USART1_CTLR1_Fields

func RE*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `RE=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.USART1_CTLR1_Fields

func RWU*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `RWU=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.USART1_CTLR1_Fields

func SBK*(r: USART1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `SBK=`*(r: var USART1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.USART1_CTLR1_Fields

func LINEN*(r: USART1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `LINEN=`*(r: var USART1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.USART1_CTLR2_Fields

func STOP*(r: USART1_CTLR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 13)

proc `STOP=`*(r: var USART1_CTLR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 13)
  tmp.setMask((val shl 12).masked(12 .. 13))
  r = tmp.USART1_CTLR2_Fields

func CLKEN*(r: USART1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `CLKEN=`*(r: var USART1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.USART1_CTLR2_Fields

func CPOL*(r: USART1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `CPOL=`*(r: var USART1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.USART1_CTLR2_Fields

func CPHA*(r: USART1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CPHA=`*(r: var USART1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.USART1_CTLR2_Fields

func LBCL*(r: USART1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `LBCL=`*(r: var USART1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.USART1_CTLR2_Fields

func LBDIE*(r: USART1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `LBDIE=`*(r: var USART1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.USART1_CTLR2_Fields

func LBDL*(r: USART1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `LBDL=`*(r: var USART1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.USART1_CTLR2_Fields

func ADD*(r: USART1_CTLR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 3)

proc `ADD=`*(r: var USART1_CTLR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 3)
  tmp.setMask((val shl 0).masked(0 .. 3))
  r = tmp.USART1_CTLR2_Fields

func CTSIE*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `CTSIE=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.USART1_CTLR3_Fields

func CTSE*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `CTSE=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.USART1_CTLR3_Fields

func RTSE*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `RTSE=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.USART1_CTLR3_Fields

func DMAT*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `DMAT=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.USART1_CTLR3_Fields

func DMAR*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `DMAR=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.USART1_CTLR3_Fields

func SCEN*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `SCEN=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.USART1_CTLR3_Fields

func NACK*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `NACK=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.USART1_CTLR3_Fields

func HDSEL*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `HDSEL=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.USART1_CTLR3_Fields

func IRLP*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `IRLP=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.USART1_CTLR3_Fields

func IREN*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `IREN=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.USART1_CTLR3_Fields

func EIE*(r: USART1_CTLR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `EIE=`*(r: var USART1_CTLR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.USART1_CTLR3_Fields

func GT*(r: USART1_GPR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 15)

proc `GT=`*(r: var USART1_GPR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 15)
  tmp.setMask((val shl 8).masked(8 .. 15))
  r = tmp.USART1_GPR_Fields

func PSC*(r: USART1_GPR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

proc `PSC=`*(r: var USART1_GPR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.USART1_GPR_Fields

type
  ADC1_STATR_Fields* = distinct uint32
  ADC1_CTLR1_Fields* = distinct uint32
  ADC1_CTLR2_Fields* = distinct uint32
  ADC1_SAMPTR1_CHARGE1_Fields* = distinct uint32
  ADC1_SAMPTR2_CHARGE2_Fields* = distinct uint32
  ADC1_IOFR1_Fields* = distinct uint32
  ADC1_IOFR2_Fields* = distinct uint32
  ADC1_IOFR3_Fields* = distinct uint32
  ADC1_IOFR4_Fields* = distinct uint32
  ADC1_WDHTR_Fields* = distinct uint32
  ADC1_WDLTR_Fields* = distinct uint32
  ADC1_RSQR1_Fields* = distinct uint32
  ADC1_RSQR2_Fields* = distinct uint32
  ADC1_RSQR3_Fields* = distinct uint32
  ADC1_ISQR_Fields* = distinct uint32
  ADC1_IDATAR1_Fields* = distinct uint32
  ADC1_IDATAR2_Fields* = distinct uint32
  ADC1_IDATAR3_Fields* = distinct uint32
  ADC1_IDATAR4_Fields* = distinct uint32
  ADC1_DLYR_Fields* = distinct uint32

proc read*(reg: ADC1_STATR_Type): ADC1_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_STATR_Fields](reg.loc))

proc read*(reg: static ADC1_STATR_Type): ADC1_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_STATR_Fields](reg.loc))

proc write*(reg: ADC1_STATR_Type, val: ADC1_STATR_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_STATR_Fields](reg.loc), val)

proc write*(reg: ADC1_STATR_Type, STRT: bool = false, JSTRT: bool = false, JEOC: bool = false, EOC: bool = false, AWD: bool = false) =
  var x: uint32
  x.setMask((STRT.uint32 shl 4).masked(4 .. 4))
  x.setMask((JSTRT.uint32 shl 3).masked(3 .. 3))
  x.setMask((JEOC.uint32 shl 2).masked(2 .. 2))
  x.setMask((EOC.uint32 shl 1).masked(1 .. 1))
  x.setMask((AWD.uint32 shl 0).masked(0 .. 0))
  reg.write x.ADC1_STATR_Fields

template modifyIt*(reg: ADC1_STATR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_CTLR1_Type): ADC1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_CTLR1_Fields](reg.loc))

proc read*(reg: static ADC1_CTLR1_Type): ADC1_CTLR1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_CTLR1_Fields](reg.loc))

proc write*(reg: ADC1_CTLR1_Type, val: ADC1_CTLR1_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_CTLR1_Fields](reg.loc), val)

proc write*(reg: ADC1_CTLR1_Type, ADC_CAL_VOL: uint32 = 0, AWDEN: bool = false, JAWDEN: bool = false, DISCNUM: uint32 = 0, JDISCEN: bool = false, DISCEN: bool = false, JAUTO: bool = false, AWDSGL: bool = false, SCAN: bool = false, JEOCIE: bool = false, AWDIE: bool = false, EOCIE: bool = false, AWDCH: uint32 = 0) =
  var x: uint32
  x.setMask((ADC_CAL_VOL shl 25).masked(25 .. 26))
  x.setMask((AWDEN.uint32 shl 23).masked(23 .. 23))
  x.setMask((JAWDEN.uint32 shl 22).masked(22 .. 22))
  x.setMask((DISCNUM shl 13).masked(13 .. 15))
  x.setMask((JDISCEN.uint32 shl 12).masked(12 .. 12))
  x.setMask((DISCEN.uint32 shl 11).masked(11 .. 11))
  x.setMask((JAUTO.uint32 shl 10).masked(10 .. 10))
  x.setMask((AWDSGL.uint32 shl 9).masked(9 .. 9))
  x.setMask((SCAN.uint32 shl 8).masked(8 .. 8))
  x.setMask((JEOCIE.uint32 shl 7).masked(7 .. 7))
  x.setMask((AWDIE.uint32 shl 6).masked(6 .. 6))
  x.setMask((EOCIE.uint32 shl 5).masked(5 .. 5))
  x.setMask((AWDCH shl 0).masked(0 .. 4))
  reg.write x.ADC1_CTLR1_Fields

template modifyIt*(reg: ADC1_CTLR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_CTLR2_Type): ADC1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_CTLR2_Fields](reg.loc))

proc read*(reg: static ADC1_CTLR2_Type): ADC1_CTLR2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_CTLR2_Fields](reg.loc))

proc write*(reg: ADC1_CTLR2_Type, val: ADC1_CTLR2_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_CTLR2_Fields](reg.loc), val)

proc write*(reg: ADC1_CTLR2_Type, SWSTART: bool = false, JSWSTART: bool = false, EXTTRIG: bool = false, EXTSEL: uint32 = 0, JEXTTRIG: bool = false, JEXTSEL: uint32 = 0, ALIGN: bool = false, DMA: bool = false, RSTCAL: bool = false, CAL: bool = false, CONT: bool = false, ADON: bool = false) =
  var x: uint32
  x.setMask((SWSTART.uint32 shl 22).masked(22 .. 22))
  x.setMask((JSWSTART.uint32 shl 21).masked(21 .. 21))
  x.setMask((EXTTRIG.uint32 shl 20).masked(20 .. 20))
  x.setMask((EXTSEL shl 17).masked(17 .. 19))
  x.setMask((JEXTTRIG.uint32 shl 15).masked(15 .. 15))
  x.setMask((JEXTSEL shl 12).masked(12 .. 14))
  x.setMask((ALIGN.uint32 shl 11).masked(11 .. 11))
  x.setMask((DMA.uint32 shl 8).masked(8 .. 8))
  x.setMask((RSTCAL.uint32 shl 3).masked(3 .. 3))
  x.setMask((CAL.uint32 shl 2).masked(2 .. 2))
  x.setMask((CONT.uint32 shl 1).masked(1 .. 1))
  x.setMask((ADON.uint32 shl 0).masked(0 .. 0))
  reg.write x.ADC1_CTLR2_Fields

template modifyIt*(reg: ADC1_CTLR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_SAMPTR1_CHARGE1_Type): ADC1_SAMPTR1_CHARGE1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_SAMPTR1_CHARGE1_Fields](reg.loc))

proc read*(reg: static ADC1_SAMPTR1_CHARGE1_Type): ADC1_SAMPTR1_CHARGE1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_SAMPTR1_CHARGE1_Fields](reg.loc))

proc write*(reg: ADC1_SAMPTR1_CHARGE1_Type, val: ADC1_SAMPTR1_CHARGE1_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_SAMPTR1_CHARGE1_Fields](reg.loc), val)

proc write*(reg: ADC1_SAMPTR1_CHARGE1_Type, SMP10_TKCG10: uint32 = 0, SMP11_TKCG11: uint32 = 0, SMP12_TKCG12: uint32 = 0, SMP13_TKCG13: uint32 = 0, SMP14_TKCG14: uint32 = 0, SMP15_TKCG15: uint32 = 0) =
  var x: uint32
  x.setMask((SMP10_TKCG10 shl 0).masked(0 .. 2))
  x.setMask((SMP11_TKCG11 shl 3).masked(3 .. 5))
  x.setMask((SMP12_TKCG12 shl 6).masked(6 .. 8))
  x.setMask((SMP13_TKCG13 shl 9).masked(9 .. 11))
  x.setMask((SMP14_TKCG14 shl 12).masked(12 .. 14))
  x.setMask((SMP15_TKCG15 shl 15).masked(15 .. 17))
  reg.write x.ADC1_SAMPTR1_CHARGE1_Fields

template modifyIt*(reg: ADC1_SAMPTR1_CHARGE1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_SAMPTR2_CHARGE2_Type): ADC1_SAMPTR2_CHARGE2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_SAMPTR2_CHARGE2_Fields](reg.loc))

proc read*(reg: static ADC1_SAMPTR2_CHARGE2_Type): ADC1_SAMPTR2_CHARGE2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_SAMPTR2_CHARGE2_Fields](reg.loc))

proc write*(reg: ADC1_SAMPTR2_CHARGE2_Type, val: ADC1_SAMPTR2_CHARGE2_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_SAMPTR2_CHARGE2_Fields](reg.loc), val)

proc write*(reg: ADC1_SAMPTR2_CHARGE2_Type, SMP0_TKCG0: uint32 = 0, SMP1_TKCG1: uint32 = 0, SMP2_TKCG2: uint32 = 0, SMP3_TKCG3: uint32 = 0, SMP4_TKCG4: uint32 = 0, SMP5_TKCG5: uint32 = 0, SMP6_TKCG6: uint32 = 0, SMP7_TKCG7: uint32 = 0, SMP8_TKCG8: uint32 = 0, SMP9_TKCG9: uint32 = 0) =
  var x: uint32
  x.setMask((SMP0_TKCG0 shl 0).masked(0 .. 2))
  x.setMask((SMP1_TKCG1 shl 3).masked(3 .. 5))
  x.setMask((SMP2_TKCG2 shl 6).masked(6 .. 8))
  x.setMask((SMP3_TKCG3 shl 9).masked(9 .. 11))
  x.setMask((SMP4_TKCG4 shl 12).masked(12 .. 14))
  x.setMask((SMP5_TKCG5 shl 15).masked(15 .. 17))
  x.setMask((SMP6_TKCG6 shl 18).masked(18 .. 20))
  x.setMask((SMP7_TKCG7 shl 21).masked(21 .. 23))
  x.setMask((SMP8_TKCG8 shl 24).masked(24 .. 26))
  x.setMask((SMP9_TKCG9 shl 27).masked(27 .. 29))
  reg.write x.ADC1_SAMPTR2_CHARGE2_Fields

template modifyIt*(reg: ADC1_SAMPTR2_CHARGE2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_IOFR1_Type): ADC1_IOFR1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IOFR1_Fields](reg.loc))

proc read*(reg: static ADC1_IOFR1_Type): ADC1_IOFR1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IOFR1_Fields](reg.loc))

proc write*(reg: ADC1_IOFR1_Type, val: ADC1_IOFR1_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_IOFR1_Fields](reg.loc), val)

proc write*(reg: ADC1_IOFR1_Type, JOFFSET1: uint32 = 0) =
  var x: uint32
  x.setMask((JOFFSET1 shl 0).masked(0 .. 9))
  reg.write x.ADC1_IOFR1_Fields

template modifyIt*(reg: ADC1_IOFR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_IOFR2_Type): ADC1_IOFR2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IOFR2_Fields](reg.loc))

proc read*(reg: static ADC1_IOFR2_Type): ADC1_IOFR2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IOFR2_Fields](reg.loc))

proc write*(reg: ADC1_IOFR2_Type, val: ADC1_IOFR2_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_IOFR2_Fields](reg.loc), val)

proc write*(reg: ADC1_IOFR2_Type, JOFFSET2: uint32 = 0) =
  var x: uint32
  x.setMask((JOFFSET2 shl 0).masked(0 .. 9))
  reg.write x.ADC1_IOFR2_Fields

template modifyIt*(reg: ADC1_IOFR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_IOFR3_Type): ADC1_IOFR3_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IOFR3_Fields](reg.loc))

proc read*(reg: static ADC1_IOFR3_Type): ADC1_IOFR3_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IOFR3_Fields](reg.loc))

proc write*(reg: ADC1_IOFR3_Type, val: ADC1_IOFR3_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_IOFR3_Fields](reg.loc), val)

proc write*(reg: ADC1_IOFR3_Type, JOFFSET3: uint32 = 0) =
  var x: uint32
  x.setMask((JOFFSET3 shl 0).masked(0 .. 9))
  reg.write x.ADC1_IOFR3_Fields

template modifyIt*(reg: ADC1_IOFR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_IOFR4_Type): ADC1_IOFR4_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IOFR4_Fields](reg.loc))

proc read*(reg: static ADC1_IOFR4_Type): ADC1_IOFR4_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IOFR4_Fields](reg.loc))

proc write*(reg: ADC1_IOFR4_Type, val: ADC1_IOFR4_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_IOFR4_Fields](reg.loc), val)

proc write*(reg: ADC1_IOFR4_Type, JOFFSET4: uint32 = 0) =
  var x: uint32
  x.setMask((JOFFSET4 shl 0).masked(0 .. 9))
  reg.write x.ADC1_IOFR4_Fields

template modifyIt*(reg: ADC1_IOFR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_WDHTR_Type): ADC1_WDHTR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_WDHTR_Fields](reg.loc))

proc read*(reg: static ADC1_WDHTR_Type): ADC1_WDHTR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_WDHTR_Fields](reg.loc))

proc write*(reg: ADC1_WDHTR_Type, val: ADC1_WDHTR_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_WDHTR_Fields](reg.loc), val)

proc write*(reg: ADC1_WDHTR_Type, HT: uint32 = 0) =
  var x: uint32
  x.setMask((HT shl 0).masked(0 .. 9))
  reg.write x.ADC1_WDHTR_Fields

template modifyIt*(reg: ADC1_WDHTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_WDLTR_Type): ADC1_WDLTR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_WDLTR_Fields](reg.loc))

proc read*(reg: static ADC1_WDLTR_Type): ADC1_WDLTR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_WDLTR_Fields](reg.loc))

proc write*(reg: ADC1_WDLTR_Type, val: ADC1_WDLTR_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_WDLTR_Fields](reg.loc), val)

proc write*(reg: ADC1_WDLTR_Type, LT: uint32 = 0) =
  var x: uint32
  x.setMask((LT shl 0).masked(0 .. 9))
  reg.write x.ADC1_WDLTR_Fields

template modifyIt*(reg: ADC1_WDLTR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_RSQR1_Type): ADC1_RSQR1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_RSQR1_Fields](reg.loc))

proc read*(reg: static ADC1_RSQR1_Type): ADC1_RSQR1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_RSQR1_Fields](reg.loc))

proc write*(reg: ADC1_RSQR1_Type, val: ADC1_RSQR1_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_RSQR1_Fields](reg.loc), val)

proc write*(reg: ADC1_RSQR1_Type, L: uint32 = 0, SQ16: uint32 = 0, SQ15: uint32 = 0, SQ14: uint32 = 0, SQ13: uint32 = 0) =
  var x: uint32
  x.setMask((L shl 20).masked(20 .. 23))
  x.setMask((SQ16 shl 15).masked(15 .. 19))
  x.setMask((SQ15 shl 10).masked(10 .. 14))
  x.setMask((SQ14 shl 5).masked(5 .. 9))
  x.setMask((SQ13 shl 0).masked(0 .. 4))
  reg.write x.ADC1_RSQR1_Fields

template modifyIt*(reg: ADC1_RSQR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_RSQR2_Type): ADC1_RSQR2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_RSQR2_Fields](reg.loc))

proc read*(reg: static ADC1_RSQR2_Type): ADC1_RSQR2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_RSQR2_Fields](reg.loc))

proc write*(reg: ADC1_RSQR2_Type, val: ADC1_RSQR2_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_RSQR2_Fields](reg.loc), val)

proc write*(reg: ADC1_RSQR2_Type, SQ12: uint32 = 0, SQ11: uint32 = 0, SQ10: uint32 = 0, SQ9: uint32 = 0, SQ8: uint32 = 0, SQ7: uint32 = 0) =
  var x: uint32
  x.setMask((SQ12 shl 25).masked(25 .. 29))
  x.setMask((SQ11 shl 20).masked(20 .. 24))
  x.setMask((SQ10 shl 15).masked(15 .. 19))
  x.setMask((SQ9 shl 10).masked(10 .. 14))
  x.setMask((SQ8 shl 5).masked(5 .. 9))
  x.setMask((SQ7 shl 0).masked(0 .. 4))
  reg.write x.ADC1_RSQR2_Fields

template modifyIt*(reg: ADC1_RSQR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_RSQR3_Type): ADC1_RSQR3_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_RSQR3_Fields](reg.loc))

proc read*(reg: static ADC1_RSQR3_Type): ADC1_RSQR3_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_RSQR3_Fields](reg.loc))

proc write*(reg: ADC1_RSQR3_Type, val: ADC1_RSQR3_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_RSQR3_Fields](reg.loc), val)

proc write*(reg: ADC1_RSQR3_Type, SQ6: uint32 = 0, SQ5: uint32 = 0, SQ4: uint32 = 0, SQ3: uint32 = 0, SQ2: uint32 = 0, SQ1: uint32 = 0) =
  var x: uint32
  x.setMask((SQ6 shl 25).masked(25 .. 29))
  x.setMask((SQ5 shl 20).masked(20 .. 24))
  x.setMask((SQ4 shl 15).masked(15 .. 19))
  x.setMask((SQ3 shl 10).masked(10 .. 14))
  x.setMask((SQ2 shl 5).masked(5 .. 9))
  x.setMask((SQ1 shl 0).masked(0 .. 4))
  reg.write x.ADC1_RSQR3_Fields

template modifyIt*(reg: ADC1_RSQR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_ISQR_Type): ADC1_ISQR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_ISQR_Fields](reg.loc))

proc read*(reg: static ADC1_ISQR_Type): ADC1_ISQR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_ISQR_Fields](reg.loc))

proc write*(reg: ADC1_ISQR_Type, val: ADC1_ISQR_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_ISQR_Fields](reg.loc), val)

proc write*(reg: ADC1_ISQR_Type, JL: uint32 = 0, JSQ4: uint32 = 0, JSQ3: uint32 = 0, JSQ2: uint32 = 0, JSQ1: uint32 = 0) =
  var x: uint32
  x.setMask((JL shl 20).masked(20 .. 21))
  x.setMask((JSQ4 shl 15).masked(15 .. 19))
  x.setMask((JSQ3 shl 10).masked(10 .. 14))
  x.setMask((JSQ2 shl 5).masked(5 .. 9))
  x.setMask((JSQ1 shl 0).masked(0 .. 4))
  reg.write x.ADC1_ISQR_Fields

template modifyIt*(reg: ADC1_ISQR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: ADC1_IDATAR1_Type): ADC1_IDATAR1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IDATAR1_Fields](reg.loc))

proc read*(reg: static ADC1_IDATAR1_Type): ADC1_IDATAR1_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IDATAR1_Fields](reg.loc))

proc read*(reg: ADC1_IDATAR2_Type): ADC1_IDATAR2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IDATAR2_Fields](reg.loc))

proc read*(reg: static ADC1_IDATAR2_Type): ADC1_IDATAR2_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IDATAR2_Fields](reg.loc))

proc read*(reg: ADC1_IDATAR3_Type): ADC1_IDATAR3_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IDATAR3_Fields](reg.loc))

proc read*(reg: static ADC1_IDATAR3_Type): ADC1_IDATAR3_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IDATAR3_Fields](reg.loc))

proc read*(reg: ADC1_IDATAR4_Type): ADC1_IDATAR4_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IDATAR4_Fields](reg.loc))

proc read*(reg: static ADC1_IDATAR4_Type): ADC1_IDATAR4_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_IDATAR4_Fields](reg.loc))

proc read*(reg: ADC1_RDATAR_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static ADC1_RDATAR_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: ADC1_DLYR_Type): ADC1_DLYR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_DLYR_Fields](reg.loc))

proc read*(reg: static ADC1_DLYR_Type): ADC1_DLYR_Fields {.inline.} =
  volatileLoad(cast[ptr ADC1_DLYR_Fields](reg.loc))

proc write*(reg: ADC1_DLYR_Type, val: ADC1_DLYR_Fields) {.inline.} =
  volatileStore(cast[ptr ADC1_DLYR_Fields](reg.loc), val)

proc write*(reg: ADC1_DLYR_Type, DLYVLU: uint32 = 0, DLYSRC: bool = false) =
  var x: uint32
  x.setMask((DLYVLU shl 0).masked(0 .. 8))
  x.setMask((DLYSRC.uint32 shl 9).masked(9 .. 9))
  reg.write x.ADC1_DLYR_Fields

template modifyIt*(reg: ADC1_DLYR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func STRT*(r: ADC1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `STRT=`*(r: var ADC1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.ADC1_STATR_Fields

func JSTRT*(r: ADC1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `JSTRT=`*(r: var ADC1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.ADC1_STATR_Fields

func JEOC*(r: ADC1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `JEOC=`*(r: var ADC1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.ADC1_STATR_Fields

func EOC*(r: ADC1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `EOC=`*(r: var ADC1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.ADC1_STATR_Fields

func AWD*(r: ADC1_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `AWD=`*(r: var ADC1_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.ADC1_STATR_Fields

func ADC_CAL_VOL*(r: ADC1_CTLR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(25 .. 26)

proc `ADC_CAL_VOL=`*(r: var ADC1_CTLR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(25 .. 26)
  tmp.setMask((val shl 25).masked(25 .. 26))
  r = tmp.ADC1_CTLR1_Fields

func AWDEN*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(23 .. 23).bool

proc `AWDEN=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(23 .. 23)
  tmp.setMask((val.uint32 shl 23).masked(23 .. 23))
  r = tmp.ADC1_CTLR1_Fields

func JAWDEN*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(22 .. 22).bool

proc `JAWDEN=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(22 .. 22)
  tmp.setMask((val.uint32 shl 22).masked(22 .. 22))
  r = tmp.ADC1_CTLR1_Fields

func DISCNUM*(r: ADC1_CTLR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(13 .. 15)

proc `DISCNUM=`*(r: var ADC1_CTLR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(13 .. 15)
  tmp.setMask((val shl 13).masked(13 .. 15))
  r = tmp.ADC1_CTLR1_Fields

func JDISCEN*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `JDISCEN=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.ADC1_CTLR1_Fields

func DISCEN*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `DISCEN=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.ADC1_CTLR1_Fields

func JAUTO*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `JAUTO=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.ADC1_CTLR1_Fields

func AWDSGL*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `AWDSGL=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.ADC1_CTLR1_Fields

func SCAN*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `SCAN=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.ADC1_CTLR1_Fields

func JEOCIE*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `JEOCIE=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.ADC1_CTLR1_Fields

func AWDIE*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `AWDIE=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.ADC1_CTLR1_Fields

func EOCIE*(r: ADC1_CTLR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `EOCIE=`*(r: var ADC1_CTLR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.ADC1_CTLR1_Fields

func AWDCH*(r: ADC1_CTLR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 4)

proc `AWDCH=`*(r: var ADC1_CTLR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 4)
  tmp.setMask((val shl 0).masked(0 .. 4))
  r = tmp.ADC1_CTLR1_Fields

func SWSTART*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(22 .. 22).bool

proc `SWSTART=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(22 .. 22)
  tmp.setMask((val.uint32 shl 22).masked(22 .. 22))
  r = tmp.ADC1_CTLR2_Fields

func JSWSTART*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(21 .. 21).bool

proc `JSWSTART=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(21 .. 21)
  tmp.setMask((val.uint32 shl 21).masked(21 .. 21))
  r = tmp.ADC1_CTLR2_Fields

func EXTTRIG*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(20 .. 20).bool

proc `EXTTRIG=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 20)
  tmp.setMask((val.uint32 shl 20).masked(20 .. 20))
  r = tmp.ADC1_CTLR2_Fields

func EXTSEL*(r: ADC1_CTLR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(17 .. 19)

proc `EXTSEL=`*(r: var ADC1_CTLR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(17 .. 19)
  tmp.setMask((val shl 17).masked(17 .. 19))
  r = tmp.ADC1_CTLR2_Fields

func JEXTTRIG*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `JEXTTRIG=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.ADC1_CTLR2_Fields

func JEXTSEL*(r: ADC1_CTLR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 14)

proc `JEXTSEL=`*(r: var ADC1_CTLR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 14)
  tmp.setMask((val shl 12).masked(12 .. 14))
  r = tmp.ADC1_CTLR2_Fields

func ALIGN*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(11 .. 11).bool

proc `ALIGN=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(11 .. 11)
  tmp.setMask((val.uint32 shl 11).masked(11 .. 11))
  r = tmp.ADC1_CTLR2_Fields

func DMA*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

proc `DMA=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 8)
  tmp.setMask((val.uint32 shl 8).masked(8 .. 8))
  r = tmp.ADC1_CTLR2_Fields

func RSTCAL*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `RSTCAL=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.ADC1_CTLR2_Fields

func CAL*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `CAL=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.ADC1_CTLR2_Fields

func CONT*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `CONT=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.ADC1_CTLR2_Fields

func ADON*(r: ADC1_CTLR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `ADON=`*(r: var ADC1_CTLR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.ADC1_CTLR2_Fields

func SMP10_TKCG10*(r: ADC1_SAMPTR1_CHARGE1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 2)

proc `SMP10_TKCG10=`*(r: var ADC1_SAMPTR1_CHARGE1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 2)
  tmp.setMask((val shl 0).masked(0 .. 2))
  r = tmp.ADC1_SAMPTR1_CHARGE1_Fields

func SMP11_TKCG11*(r: ADC1_SAMPTR1_CHARGE1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(3 .. 5)

proc `SMP11_TKCG11=`*(r: var ADC1_SAMPTR1_CHARGE1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 5)
  tmp.setMask((val shl 3).masked(3 .. 5))
  r = tmp.ADC1_SAMPTR1_CHARGE1_Fields

func SMP12_TKCG12*(r: ADC1_SAMPTR1_CHARGE1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(6 .. 8)

proc `SMP12_TKCG12=`*(r: var ADC1_SAMPTR1_CHARGE1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 8)
  tmp.setMask((val shl 6).masked(6 .. 8))
  r = tmp.ADC1_SAMPTR1_CHARGE1_Fields

func SMP13_TKCG13*(r: ADC1_SAMPTR1_CHARGE1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(9 .. 11)

proc `SMP13_TKCG13=`*(r: var ADC1_SAMPTR1_CHARGE1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 11)
  tmp.setMask((val shl 9).masked(9 .. 11))
  r = tmp.ADC1_SAMPTR1_CHARGE1_Fields

func SMP14_TKCG14*(r: ADC1_SAMPTR1_CHARGE1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 14)

proc `SMP14_TKCG14=`*(r: var ADC1_SAMPTR1_CHARGE1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 14)
  tmp.setMask((val shl 12).masked(12 .. 14))
  r = tmp.ADC1_SAMPTR1_CHARGE1_Fields

func SMP15_TKCG15*(r: ADC1_SAMPTR1_CHARGE1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(15 .. 17)

proc `SMP15_TKCG15=`*(r: var ADC1_SAMPTR1_CHARGE1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 17)
  tmp.setMask((val shl 15).masked(15 .. 17))
  r = tmp.ADC1_SAMPTR1_CHARGE1_Fields

func SMP0_TKCG0*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 2)

proc `SMP0_TKCG0=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 2)
  tmp.setMask((val shl 0).masked(0 .. 2))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP1_TKCG1*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(3 .. 5)

proc `SMP1_TKCG1=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 5)
  tmp.setMask((val shl 3).masked(3 .. 5))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP2_TKCG2*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(6 .. 8)

proc `SMP2_TKCG2=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 8)
  tmp.setMask((val shl 6).masked(6 .. 8))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP3_TKCG3*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(9 .. 11)

proc `SMP3_TKCG3=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 11)
  tmp.setMask((val shl 9).masked(9 .. 11))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP4_TKCG4*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 14)

proc `SMP4_TKCG4=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 14)
  tmp.setMask((val shl 12).masked(12 .. 14))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP5_TKCG5*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(15 .. 17)

proc `SMP5_TKCG5=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 17)
  tmp.setMask((val shl 15).masked(15 .. 17))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP6_TKCG6*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(18 .. 20)

proc `SMP6_TKCG6=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 20)
  tmp.setMask((val shl 18).masked(18 .. 20))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP7_TKCG7*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(21 .. 23)

proc `SMP7_TKCG7=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(21 .. 23)
  tmp.setMask((val shl 21).masked(21 .. 23))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP8_TKCG8*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(24 .. 26)

proc `SMP8_TKCG8=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(24 .. 26)
  tmp.setMask((val shl 24).masked(24 .. 26))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func SMP9_TKCG9*(r: ADC1_SAMPTR2_CHARGE2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(27 .. 29)

proc `SMP9_TKCG9=`*(r: var ADC1_SAMPTR2_CHARGE2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(27 .. 29)
  tmp.setMask((val shl 27).masked(27 .. 29))
  r = tmp.ADC1_SAMPTR2_CHARGE2_Fields

func JOFFSET1*(r: ADC1_IOFR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 9)

proc `JOFFSET1=`*(r: var ADC1_IOFR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 9)
  tmp.setMask((val shl 0).masked(0 .. 9))
  r = tmp.ADC1_IOFR1_Fields

func JOFFSET2*(r: ADC1_IOFR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 9)

proc `JOFFSET2=`*(r: var ADC1_IOFR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 9)
  tmp.setMask((val shl 0).masked(0 .. 9))
  r = tmp.ADC1_IOFR2_Fields

func JOFFSET3*(r: ADC1_IOFR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 9)

proc `JOFFSET3=`*(r: var ADC1_IOFR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 9)
  tmp.setMask((val shl 0).masked(0 .. 9))
  r = tmp.ADC1_IOFR3_Fields

func JOFFSET4*(r: ADC1_IOFR4_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 9)

proc `JOFFSET4=`*(r: var ADC1_IOFR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 9)
  tmp.setMask((val shl 0).masked(0 .. 9))
  r = tmp.ADC1_IOFR4_Fields

func HT*(r: ADC1_WDHTR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 9)

proc `HT=`*(r: var ADC1_WDHTR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 9)
  tmp.setMask((val shl 0).masked(0 .. 9))
  r = tmp.ADC1_WDHTR_Fields

func LT*(r: ADC1_WDLTR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 9)

proc `LT=`*(r: var ADC1_WDLTR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 9)
  tmp.setMask((val shl 0).masked(0 .. 9))
  r = tmp.ADC1_WDLTR_Fields

func L*(r: ADC1_RSQR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(20 .. 23)

proc `L=`*(r: var ADC1_RSQR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 23)
  tmp.setMask((val shl 20).masked(20 .. 23))
  r = tmp.ADC1_RSQR1_Fields

func SQ16*(r: ADC1_RSQR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(15 .. 19)

proc `SQ16=`*(r: var ADC1_RSQR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 19)
  tmp.setMask((val shl 15).masked(15 .. 19))
  r = tmp.ADC1_RSQR1_Fields

func SQ15*(r: ADC1_RSQR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 14)

proc `SQ15=`*(r: var ADC1_RSQR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 14)
  tmp.setMask((val shl 10).masked(10 .. 14))
  r = tmp.ADC1_RSQR1_Fields

func SQ14*(r: ADC1_RSQR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(5 .. 9)

proc `SQ14=`*(r: var ADC1_RSQR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 9)
  tmp.setMask((val shl 5).masked(5 .. 9))
  r = tmp.ADC1_RSQR1_Fields

func SQ13*(r: ADC1_RSQR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 4)

proc `SQ13=`*(r: var ADC1_RSQR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 4)
  tmp.setMask((val shl 0).masked(0 .. 4))
  r = tmp.ADC1_RSQR1_Fields

func SQ12*(r: ADC1_RSQR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(25 .. 29)

proc `SQ12=`*(r: var ADC1_RSQR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(25 .. 29)
  tmp.setMask((val shl 25).masked(25 .. 29))
  r = tmp.ADC1_RSQR2_Fields

func SQ11*(r: ADC1_RSQR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(20 .. 24)

proc `SQ11=`*(r: var ADC1_RSQR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 24)
  tmp.setMask((val shl 20).masked(20 .. 24))
  r = tmp.ADC1_RSQR2_Fields

func SQ10*(r: ADC1_RSQR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(15 .. 19)

proc `SQ10=`*(r: var ADC1_RSQR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 19)
  tmp.setMask((val shl 15).masked(15 .. 19))
  r = tmp.ADC1_RSQR2_Fields

func SQ9*(r: ADC1_RSQR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 14)

proc `SQ9=`*(r: var ADC1_RSQR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 14)
  tmp.setMask((val shl 10).masked(10 .. 14))
  r = tmp.ADC1_RSQR2_Fields

func SQ8*(r: ADC1_RSQR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(5 .. 9)

proc `SQ8=`*(r: var ADC1_RSQR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 9)
  tmp.setMask((val shl 5).masked(5 .. 9))
  r = tmp.ADC1_RSQR2_Fields

func SQ7*(r: ADC1_RSQR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 4)

proc `SQ7=`*(r: var ADC1_RSQR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 4)
  tmp.setMask((val shl 0).masked(0 .. 4))
  r = tmp.ADC1_RSQR2_Fields

func SQ6*(r: ADC1_RSQR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(25 .. 29)

proc `SQ6=`*(r: var ADC1_RSQR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(25 .. 29)
  tmp.setMask((val shl 25).masked(25 .. 29))
  r = tmp.ADC1_RSQR3_Fields

func SQ5*(r: ADC1_RSQR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(20 .. 24)

proc `SQ5=`*(r: var ADC1_RSQR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 24)
  tmp.setMask((val shl 20).masked(20 .. 24))
  r = tmp.ADC1_RSQR3_Fields

func SQ4*(r: ADC1_RSQR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(15 .. 19)

proc `SQ4=`*(r: var ADC1_RSQR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 19)
  tmp.setMask((val shl 15).masked(15 .. 19))
  r = tmp.ADC1_RSQR3_Fields

func SQ3*(r: ADC1_RSQR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 14)

proc `SQ3=`*(r: var ADC1_RSQR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 14)
  tmp.setMask((val shl 10).masked(10 .. 14))
  r = tmp.ADC1_RSQR3_Fields

func SQ2*(r: ADC1_RSQR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(5 .. 9)

proc `SQ2=`*(r: var ADC1_RSQR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 9)
  tmp.setMask((val shl 5).masked(5 .. 9))
  r = tmp.ADC1_RSQR3_Fields

func SQ1*(r: ADC1_RSQR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 4)

proc `SQ1=`*(r: var ADC1_RSQR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 4)
  tmp.setMask((val shl 0).masked(0 .. 4))
  r = tmp.ADC1_RSQR3_Fields

func JL*(r: ADC1_ISQR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(20 .. 21)

proc `JL=`*(r: var ADC1_ISQR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(20 .. 21)
  tmp.setMask((val shl 20).masked(20 .. 21))
  r = tmp.ADC1_ISQR_Fields

func JSQ4*(r: ADC1_ISQR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(15 .. 19)

proc `JSQ4=`*(r: var ADC1_ISQR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 19)
  tmp.setMask((val shl 15).masked(15 .. 19))
  r = tmp.ADC1_ISQR_Fields

func JSQ3*(r: ADC1_ISQR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 14)

proc `JSQ3=`*(r: var ADC1_ISQR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 14)
  tmp.setMask((val shl 10).masked(10 .. 14))
  r = tmp.ADC1_ISQR_Fields

func JSQ2*(r: ADC1_ISQR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(5 .. 9)

proc `JSQ2=`*(r: var ADC1_ISQR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 9)
  tmp.setMask((val shl 5).masked(5 .. 9))
  r = tmp.ADC1_ISQR_Fields

func JSQ1*(r: ADC1_ISQR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 4)

proc `JSQ1=`*(r: var ADC1_ISQR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 4)
  tmp.setMask((val shl 0).masked(0 .. 4))
  r = tmp.ADC1_ISQR_Fields

func IDATA*(r: ADC1_IDATAR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

func IDATA*(r: ADC1_IDATAR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

func IDATA*(r: ADC1_IDATAR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

func IDATA*(r: ADC1_IDATAR4_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 15)

func DLYVLU*(r: ADC1_DLYR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 8)

proc `DLYVLU=`*(r: var ADC1_DLYR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 8)
  tmp.setMask((val shl 0).masked(0 .. 8))
  r = tmp.ADC1_DLYR_Fields

func DLYSRC*(r: ADC1_DLYR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `DLYSRC=`*(r: var ADC1_DLYR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.ADC1_DLYR_Fields

type
  DBG_CFGR1_Fields* = distinct uint32
  DBG_CFGR2_Fields* = distinct uint32

proc read*(reg: DBG_CFGR1_Type): DBG_CFGR1_Fields {.inline.} =
  volatileLoad(cast[ptr DBG_CFGR1_Fields](reg.loc))

proc read*(reg: static DBG_CFGR1_Type): DBG_CFGR1_Fields {.inline.} =
  volatileLoad(cast[ptr DBG_CFGR1_Fields](reg.loc))

proc write*(reg: DBG_CFGR1_Type, val: DBG_CFGR1_Fields) {.inline.} =
  volatileStore(cast[ptr DBG_CFGR1_Fields](reg.loc), val)

proc write*(reg: DBG_CFGR1_Type, DEG_IWDG: bool = false, DEG_WWDG: bool = false, DEG_I2C1: bool = false, DEG_TIM1: bool = false, DEG_TIM2: bool = false) =
  var x: uint32
  x.setMask((DEG_IWDG.uint32 shl 0).masked(0 .. 0))
  x.setMask((DEG_WWDG.uint32 shl 1).masked(1 .. 1))
  x.setMask((DEG_I2C1.uint32 shl 2).masked(2 .. 2))
  x.setMask((DEG_TIM1.uint32 shl 4).masked(4 .. 4))
  x.setMask((DEG_TIM2.uint32 shl 5).masked(5 .. 5))
  reg.write x.DBG_CFGR1_Fields

template modifyIt*(reg: DBG_CFGR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: DBG_CFGR2_Type): DBG_CFGR2_Fields {.inline.} =
  volatileLoad(cast[ptr DBG_CFGR2_Fields](reg.loc))

proc read*(reg: static DBG_CFGR2_Type): DBG_CFGR2_Fields {.inline.} =
  volatileLoad(cast[ptr DBG_CFGR2_Fields](reg.loc))

proc write*(reg: DBG_CFGR2_Type, val: DBG_CFGR2_Fields) {.inline.} =
  volatileStore(cast[ptr DBG_CFGR2_Fields](reg.loc), val)

proc write*(reg: DBG_CFGR2_Type, DBG_SLEEP: bool = false, DBG_STOP: bool = false, DBG_STANDBY: bool = false) =
  var x: uint32
  x.setMask((DBG_SLEEP.uint32 shl 0).masked(0 .. 0))
  x.setMask((DBG_STOP.uint32 shl 1).masked(1 .. 1))
  x.setMask((DBG_STANDBY.uint32 shl 2).masked(2 .. 2))
  reg.write x.DBG_CFGR2_Fields

template modifyIt*(reg: DBG_CFGR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func DEG_IWDG*(r: DBG_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `DEG_IWDG=`*(r: var DBG_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DBG_CFGR1_Fields

func DEG_WWDG*(r: DBG_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `DEG_WWDG=`*(r: var DBG_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DBG_CFGR1_Fields

func DEG_I2C1*(r: DBG_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `DEG_I2C1=`*(r: var DBG_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DBG_CFGR1_Fields

func DEG_TIM1*(r: DBG_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `DEG_TIM1=`*(r: var DBG_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.DBG_CFGR1_Fields

func DEG_TIM2*(r: DBG_CFGR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `DEG_TIM2=`*(r: var DBG_CFGR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.DBG_CFGR1_Fields

func DBG_SLEEP*(r: DBG_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `DBG_SLEEP=`*(r: var DBG_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.DBG_CFGR2_Fields

func DBG_STOP*(r: DBG_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `DBG_STOP=`*(r: var DBG_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.DBG_CFGR2_Fields

func DBG_STANDBY*(r: DBG_CFGR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `DBG_STANDBY=`*(r: var DBG_CFGR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.DBG_CFGR2_Fields

proc read*(reg: ESIG_FLACAP_Type): uint16 {.inline.} =
  volatileLoad(cast[ptr uint16](reg.loc))

proc read*(reg: static ESIG_FLACAP_Type): uint16 {.inline.} =
  volatileLoad(cast[ptr uint16](reg.loc))

proc read*(reg: ESIG_UNIID1_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static ESIG_UNIID1_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: ESIG_UNIID2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static ESIG_UNIID2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: ESIG_UNIID3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static ESIG_UNIID3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

type
  FLASH_ACTLR_Fields* = distinct uint32
  FLASH_STATR_Fields* = distinct uint32
  FLASH_CTLR_Fields* = distinct uint32
  FLASH_OBR_Fields* = distinct uint32

proc read*(reg: FLASH_ACTLR_Type): FLASH_ACTLR_Fields {.inline.} =
  volatileLoad(cast[ptr FLASH_ACTLR_Fields](reg.loc))

proc read*(reg: static FLASH_ACTLR_Type): FLASH_ACTLR_Fields {.inline.} =
  volatileLoad(cast[ptr FLASH_ACTLR_Fields](reg.loc))

proc write*(reg: FLASH_ACTLR_Type, val: FLASH_ACTLR_Fields) {.inline.} =
  volatileStore(cast[ptr FLASH_ACTLR_Fields](reg.loc), val)

proc write*(reg: FLASH_ACTLR_Type, LATENCY: bool = false) =
  var x: uint32
  x.setMask((LATENCY.uint32 shl 0).masked(0 .. 0))
  reg.write x.FLASH_ACTLR_Fields

template modifyIt*(reg: FLASH_ACTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc write*(reg: FLASH_KEYR_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: FLASH_OBKEYR_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc read*(reg: FLASH_STATR_Type): FLASH_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr FLASH_STATR_Fields](reg.loc))

proc read*(reg: static FLASH_STATR_Type): FLASH_STATR_Fields {.inline.} =
  volatileLoad(cast[ptr FLASH_STATR_Fields](reg.loc))

proc write*(reg: FLASH_STATR_Type, val: FLASH_STATR_Fields) {.inline.} =
  volatileStore(cast[ptr FLASH_STATR_Fields](reg.loc), val)

proc write*(reg: FLASH_STATR_Type, BOOT_LOCK: bool = true, BOOT_MODE: bool = false, EOP: bool = false, WRPRTERR: bool = false) =
  var x: uint32
  x.setMask((BOOT_LOCK.uint32 shl 15).masked(15 .. 15))
  x.setMask((BOOT_MODE.uint32 shl 14).masked(14 .. 14))
  x.setMask((EOP.uint32 shl 5).masked(5 .. 5))
  x.setMask((WRPRTERR.uint32 shl 4).masked(4 .. 4))
  reg.write x.FLASH_STATR_Fields

template modifyIt*(reg: FLASH_STATR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: FLASH_CTLR_Type): FLASH_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr FLASH_CTLR_Fields](reg.loc))

proc read*(reg: static FLASH_CTLR_Type): FLASH_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr FLASH_CTLR_Fields](reg.loc))

proc write*(reg: FLASH_CTLR_Type, val: FLASH_CTLR_Fields) {.inline.} =
  volatileStore(cast[ptr FLASH_CTLR_Fields](reg.loc), val)

proc write*(reg: FLASH_CTLR_Type, PG: bool = false, PER: bool = false, MER: bool = false, OBPG: bool = false, OBER: bool = false, STRT: bool = false, LOCK: bool = true, OBWRE: bool = false, ERRIE: bool = false, EOPIE: bool = false, FLOCK: bool = true, PAGE_PG: bool = false, PAGE_ER: bool = false, BUFLOAD: bool = false, BUFRST: bool = false) =
  var x: uint32
  x.setMask((PG.uint32 shl 0).masked(0 .. 0))
  x.setMask((PER.uint32 shl 1).masked(1 .. 1))
  x.setMask((MER.uint32 shl 2).masked(2 .. 2))
  x.setMask((OBPG.uint32 shl 4).masked(4 .. 4))
  x.setMask((OBER.uint32 shl 5).masked(5 .. 5))
  x.setMask((STRT.uint32 shl 6).masked(6 .. 6))
  x.setMask((LOCK.uint32 shl 7).masked(7 .. 7))
  x.setMask((OBWRE.uint32 shl 9).masked(9 .. 9))
  x.setMask((ERRIE.uint32 shl 10).masked(10 .. 10))
  x.setMask((EOPIE.uint32 shl 12).masked(12 .. 12))
  x.setMask((FLOCK.uint32 shl 15).masked(15 .. 15))
  x.setMask((PAGE_PG.uint32 shl 16).masked(16 .. 16))
  x.setMask((PAGE_ER.uint32 shl 17).masked(17 .. 17))
  x.setMask((BUFLOAD.uint32 shl 18).masked(18 .. 18))
  x.setMask((BUFRST.uint32 shl 19).masked(19 .. 19))
  reg.write x.FLASH_CTLR_Fields

template modifyIt*(reg: FLASH_CTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc write*(reg: FLASH_ADDR_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc read*(reg: FLASH_OBR_Type): FLASH_OBR_Fields {.inline.} =
  volatileLoad(cast[ptr FLASH_OBR_Fields](reg.loc))

proc read*(reg: static FLASH_OBR_Type): FLASH_OBR_Fields {.inline.} =
  volatileLoad(cast[ptr FLASH_OBR_Fields](reg.loc))

proc read*(reg: FLASH_WPR_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static FLASH_WPR_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: FLASH_MODEKEYR_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: FLASH_BOOT_MODEKEYP_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

func LATENCY*(r: FLASH_ACTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `LATENCY=`*(r: var FLASH_ACTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.FLASH_ACTLR_Fields

func BOOT_LOCK*(r: FLASH_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `BOOT_LOCK=`*(r: var FLASH_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.FLASH_STATR_Fields

func BOOT_MODE*(r: FLASH_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(14 .. 14).bool

proc `BOOT_MODE=`*(r: var FLASH_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(14 .. 14)
  tmp.setMask((val.uint32 shl 14).masked(14 .. 14))
  r = tmp.FLASH_STATR_Fields

func EOP*(r: FLASH_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `EOP=`*(r: var FLASH_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.FLASH_STATR_Fields

func WRPRTERR*(r: FLASH_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `WRPRTERR=`*(r: var FLASH_STATR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.FLASH_STATR_Fields

func BSY*(r: FLASH_STATR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func PG*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `PG=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.FLASH_CTLR_Fields

func PER*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `PER=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.FLASH_CTLR_Fields

func MER*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `MER=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.FLASH_CTLR_Fields

func OBPG*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `OBPG=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.FLASH_CTLR_Fields

func OBER*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `OBER=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.FLASH_CTLR_Fields

func STRT*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(6 .. 6).bool

proc `STRT=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(6 .. 6)
  tmp.setMask((val.uint32 shl 6).masked(6 .. 6))
  r = tmp.FLASH_CTLR_Fields

func LOCK*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(7 .. 7).bool

proc `LOCK=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.FLASH_CTLR_Fields

func OBWRE*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

proc `OBWRE=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(9 .. 9)
  tmp.setMask((val.uint32 shl 9).masked(9 .. 9))
  r = tmp.FLASH_CTLR_Fields

func ERRIE*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(10 .. 10).bool

proc `ERRIE=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(10 .. 10)
  tmp.setMask((val.uint32 shl 10).masked(10 .. 10))
  r = tmp.FLASH_CTLR_Fields

func EOPIE*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(12 .. 12).bool

proc `EOPIE=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 12)
  tmp.setMask((val.uint32 shl 12).masked(12 .. 12))
  r = tmp.FLASH_CTLR_Fields

func FLOCK*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(15 .. 15).bool

proc `FLOCK=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(15 .. 15)
  tmp.setMask((val.uint32 shl 15).masked(15 .. 15))
  r = tmp.FLASH_CTLR_Fields

func PAGE_PG*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(16 .. 16).bool

proc `PAGE_PG=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 16)
  tmp.setMask((val.uint32 shl 16).masked(16 .. 16))
  r = tmp.FLASH_CTLR_Fields

func PAGE_ER*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(17 .. 17).bool

proc `PAGE_ER=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(17 .. 17)
  tmp.setMask((val.uint32 shl 17).masked(17 .. 17))
  r = tmp.FLASH_CTLR_Fields

func BUFLOAD*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(18 .. 18).bool

proc `BUFLOAD=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(18 .. 18)
  tmp.setMask((val.uint32 shl 18).masked(18 .. 18))
  r = tmp.FLASH_CTLR_Fields

func BUFRST*(r: FLASH_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(19 .. 19).bool

proc `BUFRST=`*(r: var FLASH_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(19 .. 19)
  tmp.setMask((val.uint32 shl 19).masked(19 .. 19))
  r = tmp.FLASH_CTLR_Fields

func OBERR*(r: FLASH_OBR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

func RDPRT*(r: FLASH_OBR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

func IWDG_SW*(r: FLASH_OBR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

func STOP_RST*(r: FLASH_OBR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

func STANDY_RST*(r: FLASH_OBR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

func CFG_RST_MODE*(r: FLASH_OBR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(5 .. 6)

func DATA0*(r: FLASH_OBR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(10 .. 17)

func DATA1*(r: FLASH_OBR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(18 .. 25)

type
  PFIC_ISR1_Fields* = distinct uint32
  PFIC_ISR4_Fields* = distinct uint32
  PFIC_IPR1_Fields* = distinct uint32
  PFIC_IPR4_Fields* = distinct uint32
  PFIC_ITHRESDR_Fields* = distinct uint32
  PFIC_CFGR_Fields* = distinct uint32
  PFIC_GISR_Fields* = distinct uint32
  PFIC_VTFIDR_Fields* = distinct uint32
  PFIC_VTFADDRR0_Fields* = distinct uint32
  PFIC_VTFADDRR1_Fields* = distinct uint32
  PFIC_VTFADDRR2_Fields* = distinct uint32
  PFIC_VTFADDRR3_Fields* = distinct uint32
  PFIC_IENR1_Fields* = distinct uint32
  PFIC_IENR4_Fields* = distinct uint32
  PFIC_IRER1_Fields* = distinct uint32
  PFIC_IRER4_Fields* = distinct uint32
  PFIC_IPSR1_Fields* = distinct uint32
  PFIC_IPSR4_Fields* = distinct uint32
  PFIC_IPRR1_Fields* = distinct uint32
  PFIC_IPRR4_Fields* = distinct uint32
  PFIC_IACTR1_Fields* = distinct uint32
  PFIC_IACTR4_Fields* = distinct uint32
  PFIC_SCTLR_Fields* = distinct uint32
  PFIC_STK_CTLR_Fields* = distinct uint32
  PFIC_STK_SR_Fields* = distinct uint32

proc read*(reg: PFIC_ISR1_Type): PFIC_ISR1_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_ISR1_Fields](reg.loc))

proc read*(reg: static PFIC_ISR1_Type): PFIC_ISR1_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_ISR1_Fields](reg.loc))

proc read*(reg: PFIC_ISR2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static PFIC_ISR2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: PFIC_ISR3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static PFIC_ISR3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: PFIC_ISR4_Type): PFIC_ISR4_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_ISR4_Fields](reg.loc))

proc read*(reg: static PFIC_ISR4_Type): PFIC_ISR4_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_ISR4_Fields](reg.loc))

proc read*(reg: PFIC_IPR1_Type): PFIC_IPR1_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_IPR1_Fields](reg.loc))

proc read*(reg: static PFIC_IPR1_Type): PFIC_IPR1_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_IPR1_Fields](reg.loc))

proc read*(reg: PFIC_IPR2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static PFIC_IPR2_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: PFIC_IPR3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static PFIC_IPR3_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: PFIC_IPR4_Type): PFIC_IPR4_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_IPR4_Fields](reg.loc))

proc read*(reg: static PFIC_IPR4_Type): PFIC_IPR4_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_IPR4_Fields](reg.loc))

proc read*(reg: PFIC_ITHRESDR_Type): PFIC_ITHRESDR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_ITHRESDR_Fields](reg.loc))

proc read*(reg: static PFIC_ITHRESDR_Type): PFIC_ITHRESDR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_ITHRESDR_Fields](reg.loc))

proc write*(reg: PFIC_ITHRESDR_Type, val: PFIC_ITHRESDR_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_ITHRESDR_Fields](reg.loc), val)

proc write*(reg: PFIC_ITHRESDR_Type, THRESHOLD: uint32 = 0) =
  var x: uint32
  x.setMask((THRESHOLD shl 0).masked(0 .. 7))
  reg.write x.PFIC_ITHRESDR_Fields

template modifyIt*(reg: PFIC_ITHRESDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_CFGR_Type): PFIC_CFGR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_CFGR_Fields](reg.loc))

proc read*(reg: static PFIC_CFGR_Type): PFIC_CFGR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_CFGR_Fields](reg.loc))

proc write*(reg: PFIC_CFGR_Type, val: PFIC_CFGR_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_CFGR_Fields](reg.loc), val)

proc write*(reg: PFIC_CFGR_Type, RESETSYS: bool = false, KEYCODE: uint32 = 0) =
  var x: uint32
  x.setMask((RESETSYS.uint32 shl 7).masked(7 .. 7))
  x.setMask((KEYCODE shl 16).masked(16 .. 31))
  reg.write x.PFIC_CFGR_Fields

template modifyIt*(reg: PFIC_CFGR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_GISR_Type): PFIC_GISR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_GISR_Fields](reg.loc))

proc read*(reg: static PFIC_GISR_Type): PFIC_GISR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_GISR_Fields](reg.loc))

proc read*(reg: PFIC_VTFIDR_Type): PFIC_VTFIDR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFIDR_Fields](reg.loc))

proc read*(reg: static PFIC_VTFIDR_Type): PFIC_VTFIDR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFIDR_Fields](reg.loc))

proc write*(reg: PFIC_VTFIDR_Type, val: PFIC_VTFIDR_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_VTFIDR_Fields](reg.loc), val)

proc write*(reg: PFIC_VTFIDR_Type, VTFID0: uint32 = 0, VTFID1: uint32 = 0, VTFID2: uint32 = 0, VTFID3: uint32 = 0) =
  var x: uint32
  x.setMask((VTFID0 shl 0).masked(0 .. 7))
  x.setMask((VTFID1 shl 8).masked(8 .. 15))
  x.setMask((VTFID2 shl 16).masked(16 .. 23))
  x.setMask((VTFID3 shl 24).masked(24 .. 31))
  reg.write x.PFIC_VTFIDR_Fields

template modifyIt*(reg: PFIC_VTFIDR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_VTFADDRR0_Type): PFIC_VTFADDRR0_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFADDRR0_Fields](reg.loc))

proc read*(reg: static PFIC_VTFADDRR0_Type): PFIC_VTFADDRR0_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFADDRR0_Fields](reg.loc))

proc write*(reg: PFIC_VTFADDRR0_Type, val: PFIC_VTFADDRR0_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_VTFADDRR0_Fields](reg.loc), val)

proc write*(reg: PFIC_VTFADDRR0_Type, VTF0EN: bool = false, ADDR0: uint32 = 0) =
  var x: uint32
  x.setMask((VTF0EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((ADDR0 shl 1).masked(1 .. 31))
  reg.write x.PFIC_VTFADDRR0_Fields

template modifyIt*(reg: PFIC_VTFADDRR0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_VTFADDRR1_Type): PFIC_VTFADDRR1_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFADDRR1_Fields](reg.loc))

proc read*(reg: static PFIC_VTFADDRR1_Type): PFIC_VTFADDRR1_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFADDRR1_Fields](reg.loc))

proc write*(reg: PFIC_VTFADDRR1_Type, val: PFIC_VTFADDRR1_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_VTFADDRR1_Fields](reg.loc), val)

proc write*(reg: PFIC_VTFADDRR1_Type, VTF1EN: bool = false, ADDR1: uint32 = 0) =
  var x: uint32
  x.setMask((VTF1EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((ADDR1 shl 1).masked(1 .. 31))
  reg.write x.PFIC_VTFADDRR1_Fields

template modifyIt*(reg: PFIC_VTFADDRR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_VTFADDRR2_Type): PFIC_VTFADDRR2_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFADDRR2_Fields](reg.loc))

proc read*(reg: static PFIC_VTFADDRR2_Type): PFIC_VTFADDRR2_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFADDRR2_Fields](reg.loc))

proc write*(reg: PFIC_VTFADDRR2_Type, val: PFIC_VTFADDRR2_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_VTFADDRR2_Fields](reg.loc), val)

proc write*(reg: PFIC_VTFADDRR2_Type, VTF2EN: bool = false, ADDR2: uint32 = 0) =
  var x: uint32
  x.setMask((VTF2EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((ADDR2 shl 1).masked(1 .. 31))
  reg.write x.PFIC_VTFADDRR2_Fields

template modifyIt*(reg: PFIC_VTFADDRR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_VTFADDRR3_Type): PFIC_VTFADDRR3_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFADDRR3_Fields](reg.loc))

proc read*(reg: static PFIC_VTFADDRR3_Type): PFIC_VTFADDRR3_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_VTFADDRR3_Fields](reg.loc))

proc write*(reg: PFIC_VTFADDRR3_Type, val: PFIC_VTFADDRR3_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_VTFADDRR3_Fields](reg.loc), val)

proc write*(reg: PFIC_VTFADDRR3_Type, VTF3EN: bool = false, ADDR3: uint32 = 0) =
  var x: uint32
  x.setMask((VTF3EN.uint32 shl 0).masked(0 .. 0))
  x.setMask((ADDR3 shl 1).masked(1 .. 31))
  reg.write x.PFIC_VTFADDRR3_Fields

template modifyIt*(reg: PFIC_VTFADDRR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc write*(reg: PFIC_IENR1_Type, val: PFIC_IENR1_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IENR1_Fields](reg.loc), val)

proc write*(reg: PFIC_IENR1_Type, INTEN: uint32 = 0) =
  var x: uint32
  x.setMask((INTEN shl 12).masked(12 .. 31))
  reg.write x.PFIC_IENR1_Fields

proc write*(reg: PFIC_IENR2_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IENR3_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IENR4_Type, val: PFIC_IENR4_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IENR4_Fields](reg.loc), val)

proc write*(reg: PFIC_IENR4_Type, INTEN: uint32 = 0) =
  var x: uint32
  x.setMask((INTEN shl 0).masked(0 .. 7))
  reg.write x.PFIC_IENR4_Fields

proc write*(reg: PFIC_IRER1_Type, val: PFIC_IRER1_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IRER1_Fields](reg.loc), val)

proc write*(reg: PFIC_IRER1_Type, INTRSET: uint32 = 0) =
  var x: uint32
  x.setMask((INTRSET shl 12).masked(12 .. 31))
  reg.write x.PFIC_IRER1_Fields

proc write*(reg: PFIC_IRER2_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IRER3_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IRER4_Type, val: PFIC_IRER4_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IRER4_Fields](reg.loc), val)

proc write*(reg: PFIC_IRER4_Type, INTRSET: uint32 = 0) =
  var x: uint32
  x.setMask((INTRSET shl 0).masked(0 .. 7))
  reg.write x.PFIC_IRER4_Fields

proc write*(reg: PFIC_IPSR1_Type, val: PFIC_IPSR1_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IPSR1_Fields](reg.loc), val)

proc write*(reg: PFIC_IPSR1_Type, PENDSET2_3: uint32 = 0, PENDSET12_31: uint32 = 0) =
  var x: uint32
  x.setMask((PENDSET2_3 shl 2).masked(2 .. 3))
  x.setMask((PENDSET12_31 shl 12).masked(12 .. 31))
  reg.write x.PFIC_IPSR1_Fields

proc write*(reg: PFIC_IPSR2_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IPSR3_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IPSR4_Type, val: PFIC_IPSR4_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IPSR4_Fields](reg.loc), val)

proc write*(reg: PFIC_IPSR4_Type, PENDSET: uint32 = 0) =
  var x: uint32
  x.setMask((PENDSET shl 0).masked(0 .. 7))
  reg.write x.PFIC_IPSR4_Fields

proc write*(reg: PFIC_IPRR1_Type, val: PFIC_IPRR1_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IPRR1_Fields](reg.loc), val)

proc write*(reg: PFIC_IPRR1_Type, PENDRESET2_3: uint32 = 0, PENDRESET12_31: uint32 = 0) =
  var x: uint32
  x.setMask((PENDRESET2_3 shl 2).masked(2 .. 3))
  x.setMask((PENDRESET12_31 shl 12).masked(12 .. 31))
  reg.write x.PFIC_IPRR1_Fields

proc write*(reg: PFIC_IPRR2_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IPRR3_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IPRR4_Type, val: PFIC_IPRR4_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IPRR4_Fields](reg.loc), val)

proc write*(reg: PFIC_IPRR4_Type, PENDRESET: uint32 = 0) =
  var x: uint32
  x.setMask((PENDRESET shl 0).masked(0 .. 7))
  reg.write x.PFIC_IPRR4_Fields

proc write*(reg: PFIC_IACTR1_Type, val: PFIC_IACTR1_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IACTR1_Fields](reg.loc), val)

proc write*(reg: PFIC_IACTR1_Type, IACTS2_3: uint32 = 0, IACTS12_31: uint32 = 0) =
  var x: uint32
  x.setMask((IACTS2_3 shl 2).masked(2 .. 3))
  x.setMask((IACTS12_31 shl 12).masked(12 .. 31))
  reg.write x.PFIC_IACTR1_Fields

proc write*(reg: PFIC_IACTR2_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IACTR3_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

proc write*(reg: PFIC_IACTR4_Type, val: PFIC_IACTR4_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_IACTR4_Fields](reg.loc), val)

proc write*(reg: PFIC_IACTR4_Type, IACTS: uint32 = 0) =
  var x: uint32
  x.setMask((IACTS shl 0).masked(0 .. 7))
  reg.write x.PFIC_IACTR4_Fields

proc read*(reg: PFIC_IPRIOR0_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR0_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR0_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR0_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR1_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR1_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR1_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR1_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR2_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR2_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR2_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR2_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR3_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR3_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR3_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR3_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR4_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR4_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR4_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR4_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR5_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR5_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR5_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR5_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR6_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR6_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR6_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR6_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR7_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR7_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR7_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR7_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR8_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR8_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR8_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR8_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR9_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR9_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR9_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR9_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR10_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR10_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR10_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR10_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR11_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR11_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR11_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR11_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR12_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR12_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR12_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR12_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR13_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR13_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR13_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR13_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR14_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR14_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR14_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR14_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR15_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR15_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR15_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR15_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR16_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR16_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR16_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR16_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR17_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR17_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR17_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR17_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR18_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR18_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR18_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR18_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR19_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR19_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR19_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR19_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR20_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR20_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR20_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR20_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR21_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR21_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR21_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR21_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR22_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR22_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR22_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR22_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR23_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR23_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR23_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR23_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR24_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR24_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR24_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR24_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR25_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR25_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR25_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR25_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR26_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR26_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR26_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR26_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR27_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR27_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR27_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR27_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR28_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR28_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR28_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR28_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR29_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR29_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR29_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR29_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR30_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR30_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR30_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR30_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR31_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR31_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR31_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR31_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR32_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR32_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR32_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR32_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR33_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR33_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR33_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR33_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR34_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR34_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR34_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR34_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR35_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR35_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR35_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR35_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR36_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR36_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR36_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR36_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR37_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR37_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR37_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR37_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR38_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR38_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR38_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR38_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR39_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR39_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR39_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR39_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR40_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR40_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR40_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR40_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR41_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR41_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR41_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR41_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR42_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR42_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR42_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR42_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR43_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR43_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR43_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR43_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR44_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR44_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR44_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR44_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR45_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR45_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR45_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR45_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR46_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR46_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR46_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR46_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR47_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR47_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR47_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR47_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR48_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR48_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR48_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR48_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR49_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR49_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR49_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR49_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR50_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR50_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR50_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR50_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR51_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR51_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR51_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR51_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR52_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR52_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR52_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR52_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR53_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR53_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR53_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR53_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR54_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR54_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR54_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR54_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR55_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR55_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR55_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR55_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR56_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR56_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR56_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR56_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR57_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR57_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR57_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR57_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR58_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR58_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR58_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR58_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR59_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR59_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR59_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR59_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR60_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR60_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR60_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR60_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR61_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR61_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR61_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR61_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR62_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR62_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR62_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR62_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_IPRIOR63_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc read*(reg: static PFIC_IPRIOR63_Type): uint8 {.inline.} =
  volatileLoad(cast[ptr uint8](reg.loc))

proc write*(reg: PFIC_IPRIOR63_Type, val: uint8) {.inline.} =
  volatileStore(cast[ptr uint8](reg.loc), val)

template modifyIt*(reg: PFIC_IPRIOR63_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_SCTLR_Type): PFIC_SCTLR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_SCTLR_Fields](reg.loc))

proc read*(reg: static PFIC_SCTLR_Type): PFIC_SCTLR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_SCTLR_Fields](reg.loc))

proc write*(reg: PFIC_SCTLR_Type, val: PFIC_SCTLR_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_SCTLR_Fields](reg.loc), val)

proc write*(reg: PFIC_SCTLR_Type, SLEEPONEXIT: bool = false, SLEEPDEEP: bool = false, WFITOWFE: bool = false, SEVONPEND: bool = false, SETEVENT: bool = false, SYSRESET: bool = false) =
  var x: uint32
  x.setMask((SLEEPONEXIT.uint32 shl 1).masked(1 .. 1))
  x.setMask((SLEEPDEEP.uint32 shl 2).masked(2 .. 2))
  x.setMask((WFITOWFE.uint32 shl 3).masked(3 .. 3))
  x.setMask((SEVONPEND.uint32 shl 4).masked(4 .. 4))
  x.setMask((SETEVENT.uint32 shl 5).masked(5 .. 5))
  x.setMask((SYSRESET.uint32 shl 31).masked(31 .. 31))
  reg.write x.PFIC_SCTLR_Fields

template modifyIt*(reg: PFIC_SCTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_STK_CTLR_Type): PFIC_STK_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_STK_CTLR_Fields](reg.loc))

proc read*(reg: static PFIC_STK_CTLR_Type): PFIC_STK_CTLR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_STK_CTLR_Fields](reg.loc))

proc write*(reg: PFIC_STK_CTLR_Type, val: PFIC_STK_CTLR_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_STK_CTLR_Fields](reg.loc), val)

proc write*(reg: PFIC_STK_CTLR_Type, STE: bool = false, STIE: bool = false, STCLK: bool = false, STRE: bool = false, MODE: bool = false, INIT: bool = false, SWIE: bool = false) =
  var x: uint32
  x.setMask((STE.uint32 shl 0).masked(0 .. 0))
  x.setMask((STIE.uint32 shl 1).masked(1 .. 1))
  x.setMask((STCLK.uint32 shl 2).masked(2 .. 2))
  x.setMask((STRE.uint32 shl 3).masked(3 .. 3))
  x.setMask((MODE.uint32 shl 4).masked(4 .. 4))
  x.setMask((INIT.uint32 shl 5).masked(5 .. 5))
  x.setMask((SWIE.uint32 shl 31).masked(31 .. 31))
  reg.write x.PFIC_STK_CTLR_Fields

template modifyIt*(reg: PFIC_STK_CTLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_STK_SR_Type): PFIC_STK_SR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_STK_SR_Fields](reg.loc))

proc read*(reg: static PFIC_STK_SR_Type): PFIC_STK_SR_Fields {.inline.} =
  volatileLoad(cast[ptr PFIC_STK_SR_Fields](reg.loc))

proc write*(reg: PFIC_STK_SR_Type, val: PFIC_STK_SR_Fields) {.inline.} =
  volatileStore(cast[ptr PFIC_STK_SR_Fields](reg.loc), val)

proc write*(reg: PFIC_STK_SR_Type, CNTIF: bool = false) =
  var x: uint32
  x.setMask((CNTIF.uint32 shl 0).masked(0 .. 0))
  reg.write x.PFIC_STK_SR_Fields

template modifyIt*(reg: PFIC_STK_SR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_STK_CNTL_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static PFIC_STK_CNTL_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: PFIC_STK_CNTL_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: PFIC_STK_CNTL_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

proc read*(reg: PFIC_STK_CMPLR_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc read*(reg: static PFIC_STK_CMPLR_Type): uint32 {.inline.} =
  volatileLoad(cast[ptr uint32](reg.loc))

proc write*(reg: PFIC_STK_CMPLR_Type, val: uint32) {.inline.} =
  volatileStore(cast[ptr uint32](reg.loc), val)

template modifyIt*(reg: PFIC_STK_CMPLR_Type, op: untyped): untyped =
  block:
    var it {.inject.} = reg.read()
    op
    reg.write(it)

func INTENSTA2_3*(r: PFIC_ISR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

func INTENSTA12_31*(r: PFIC_ISR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 31)

func INTENSTA*(r: PFIC_ISR4_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

func PENDSTA2_3*(r: PFIC_IPR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(2 .. 3)

func PENDSTA12_31*(r: PFIC_IPR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(12 .. 31)

func PENDSTA*(r: PFIC_IPR4_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

func THRESHOLD*(r: PFIC_ITHRESDR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

proc `THRESHOLD=`*(r: var PFIC_ITHRESDR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.PFIC_ITHRESDR_Fields

proc `RESETSYS=`*(r: var PFIC_CFGR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(7 .. 7)
  tmp.setMask((val.uint32 shl 7).masked(7 .. 7))
  r = tmp.PFIC_CFGR_Fields

proc `KEYCODE=`*(r: var PFIC_CFGR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 31)
  tmp.setMask((val shl 16).masked(16 .. 31))
  r = tmp.PFIC_CFGR_Fields

func NESTSTA*(r: PFIC_GISR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

func GACTSTA*(r: PFIC_GISR_Fields): bool {.inline.} =
  r.uint32.bitsliced(8 .. 8).bool

func GPENDSTA*(r: PFIC_GISR_Fields): bool {.inline.} =
  r.uint32.bitsliced(9 .. 9).bool

func VTFID0*(r: PFIC_VTFIDR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(0 .. 7)

proc `VTFID0=`*(r: var PFIC_VTFIDR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.PFIC_VTFIDR_Fields

func VTFID1*(r: PFIC_VTFIDR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(8 .. 15)

proc `VTFID1=`*(r: var PFIC_VTFIDR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(8 .. 15)
  tmp.setMask((val shl 8).masked(8 .. 15))
  r = tmp.PFIC_VTFIDR_Fields

func VTFID2*(r: PFIC_VTFIDR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(16 .. 23)

proc `VTFID2=`*(r: var PFIC_VTFIDR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(16 .. 23)
  tmp.setMask((val shl 16).masked(16 .. 23))
  r = tmp.PFIC_VTFIDR_Fields

func VTFID3*(r: PFIC_VTFIDR_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(24 .. 31)

proc `VTFID3=`*(r: var PFIC_VTFIDR_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(24 .. 31)
  tmp.setMask((val shl 24).masked(24 .. 31))
  r = tmp.PFIC_VTFIDR_Fields

func VTF0EN*(r: PFIC_VTFADDRR0_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `VTF0EN=`*(r: var PFIC_VTFADDRR0_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.PFIC_VTFADDRR0_Fields

func ADDR0*(r: PFIC_VTFADDRR0_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(1 .. 31)

proc `ADDR0=`*(r: var PFIC_VTFADDRR0_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 31)
  tmp.setMask((val shl 1).masked(1 .. 31))
  r = tmp.PFIC_VTFADDRR0_Fields

func VTF1EN*(r: PFIC_VTFADDRR1_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `VTF1EN=`*(r: var PFIC_VTFADDRR1_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.PFIC_VTFADDRR1_Fields

func ADDR1*(r: PFIC_VTFADDRR1_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(1 .. 31)

proc `ADDR1=`*(r: var PFIC_VTFADDRR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 31)
  tmp.setMask((val shl 1).masked(1 .. 31))
  r = tmp.PFIC_VTFADDRR1_Fields

func VTF2EN*(r: PFIC_VTFADDRR2_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `VTF2EN=`*(r: var PFIC_VTFADDRR2_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.PFIC_VTFADDRR2_Fields

func ADDR2*(r: PFIC_VTFADDRR2_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(1 .. 31)

proc `ADDR2=`*(r: var PFIC_VTFADDRR2_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 31)
  tmp.setMask((val shl 1).masked(1 .. 31))
  r = tmp.PFIC_VTFADDRR2_Fields

func VTF3EN*(r: PFIC_VTFADDRR3_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `VTF3EN=`*(r: var PFIC_VTFADDRR3_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.PFIC_VTFADDRR3_Fields

func ADDR3*(r: PFIC_VTFADDRR3_Fields): uint32 {.inline.} =
  r.uint32.bitsliced(1 .. 31)

proc `ADDR3=`*(r: var PFIC_VTFADDRR3_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 31)
  tmp.setMask((val shl 1).masked(1 .. 31))
  r = tmp.PFIC_VTFADDRR3_Fields

proc `INTEN=`*(r: var PFIC_IENR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 31)
  tmp.setMask((val shl 12).masked(12 .. 31))
  r = tmp.PFIC_IENR1_Fields

proc `INTEN=`*(r: var PFIC_IENR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.PFIC_IENR4_Fields

proc `INTRSET=`*(r: var PFIC_IRER1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 31)
  tmp.setMask((val shl 12).masked(12 .. 31))
  r = tmp.PFIC_IRER1_Fields

proc `INTRSET=`*(r: var PFIC_IRER4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.PFIC_IRER4_Fields

proc `PENDSET2_3=`*(r: var PFIC_IPSR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.PFIC_IPSR1_Fields

proc `PENDSET12_31=`*(r: var PFIC_IPSR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 31)
  tmp.setMask((val shl 12).masked(12 .. 31))
  r = tmp.PFIC_IPSR1_Fields

proc `PENDSET=`*(r: var PFIC_IPSR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.PFIC_IPSR4_Fields

proc `PENDRESET2_3=`*(r: var PFIC_IPRR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.PFIC_IPRR1_Fields

proc `PENDRESET12_31=`*(r: var PFIC_IPRR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 31)
  tmp.setMask((val shl 12).masked(12 .. 31))
  r = tmp.PFIC_IPRR1_Fields

proc `PENDRESET=`*(r: var PFIC_IPRR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.PFIC_IPRR4_Fields

proc `IACTS2_3=`*(r: var PFIC_IACTR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 3)
  tmp.setMask((val shl 2).masked(2 .. 3))
  r = tmp.PFIC_IACTR1_Fields

proc `IACTS12_31=`*(r: var PFIC_IACTR1_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(12 .. 31)
  tmp.setMask((val shl 12).masked(12 .. 31))
  r = tmp.PFIC_IACTR1_Fields

proc `IACTS=`*(r: var PFIC_IACTR4_Fields, val: uint32) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 7)
  tmp.setMask((val shl 0).masked(0 .. 7))
  r = tmp.PFIC_IACTR4_Fields

func SLEEPONEXIT*(r: PFIC_SCTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `SLEEPONEXIT=`*(r: var PFIC_SCTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.PFIC_SCTLR_Fields

func SLEEPDEEP*(r: PFIC_SCTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `SLEEPDEEP=`*(r: var PFIC_SCTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.PFIC_SCTLR_Fields

func WFITOWFE*(r: PFIC_SCTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `WFITOWFE=`*(r: var PFIC_SCTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.PFIC_SCTLR_Fields

func SEVONPEND*(r: PFIC_SCTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `SEVONPEND=`*(r: var PFIC_SCTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.PFIC_SCTLR_Fields

func SETEVENT*(r: PFIC_SCTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `SETEVENT=`*(r: var PFIC_SCTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.PFIC_SCTLR_Fields

func SYSRESET*(r: PFIC_SCTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(31 .. 31).bool

proc `SYSRESET=`*(r: var PFIC_SCTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(31 .. 31)
  tmp.setMask((val.uint32 shl 31).masked(31 .. 31))
  r = tmp.PFIC_SCTLR_Fields

func STE*(r: PFIC_STK_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `STE=`*(r: var PFIC_STK_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.PFIC_STK_CTLR_Fields

func STIE*(r: PFIC_STK_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(1 .. 1).bool

proc `STIE=`*(r: var PFIC_STK_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(1 .. 1)
  tmp.setMask((val.uint32 shl 1).masked(1 .. 1))
  r = tmp.PFIC_STK_CTLR_Fields

func STCLK*(r: PFIC_STK_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(2 .. 2).bool

proc `STCLK=`*(r: var PFIC_STK_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(2 .. 2)
  tmp.setMask((val.uint32 shl 2).masked(2 .. 2))
  r = tmp.PFIC_STK_CTLR_Fields

func STRE*(r: PFIC_STK_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(3 .. 3).bool

proc `STRE=`*(r: var PFIC_STK_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(3 .. 3)
  tmp.setMask((val.uint32 shl 3).masked(3 .. 3))
  r = tmp.PFIC_STK_CTLR_Fields

func MODE*(r: PFIC_STK_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(4 .. 4).bool

proc `MODE=`*(r: var PFIC_STK_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(4 .. 4)
  tmp.setMask((val.uint32 shl 4).masked(4 .. 4))
  r = tmp.PFIC_STK_CTLR_Fields

func INIT*(r: PFIC_STK_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(5 .. 5).bool

proc `INIT=`*(r: var PFIC_STK_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(5 .. 5)
  tmp.setMask((val.uint32 shl 5).masked(5 .. 5))
  r = tmp.PFIC_STK_CTLR_Fields

func SWIE*(r: PFIC_STK_CTLR_Fields): bool {.inline.} =
  r.uint32.bitsliced(31 .. 31).bool

proc `SWIE=`*(r: var PFIC_STK_CTLR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(31 .. 31)
  tmp.setMask((val.uint32 shl 31).masked(31 .. 31))
  r = tmp.PFIC_STK_CTLR_Fields

func CNTIF*(r: PFIC_STK_SR_Fields): bool {.inline.} =
  r.uint32.bitsliced(0 .. 0).bool

proc `CNTIF=`*(r: var PFIC_STK_SR_Fields, val: bool) {.inline.} =
  var tmp = r.uint32
  tmp.clearMask(0 .. 0)
  tmp.setMask((val.uint32 shl 0).masked(0 .. 0))
  r = tmp.PFIC_STK_SR_Fields

