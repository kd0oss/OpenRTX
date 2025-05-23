
#include "interfaces/arch_registers.h"
#include "core/interrupts.h" //For the unexpected interrupt call
#include "core/cache_cortexMx.h"
#include "drivers/pll.h"
#include "kernel/stage_2_boot.h"
#include <string.h>

/*
 * startup.cpp
 * STM32 C++ startup.
 * NOTE: for stm32h753 devices ONLY.
 * Supports interrupt handlers in C++ without extern "C"
 * Developed by Terraneo Federico, based on ST startup code.
 * Additionally modified to boot Miosix.
 */

/**
 * Called by Reset_Handler, performs initialization and calls main.
 * Never returns.
 */
void program_startup() __attribute__((noreturn));
void program_startup()
{
    __disable_irq();

    //SystemInit() is called *before* initializing .data and zeroing .bss
    //Despite all startup files provided by ST do the opposite, there are three
    //good reasons to do so:
    //First, the CMSIS specifications say that SystemInit() must not access
    //global variables, so it is actually possible to call it before
    //Second, when running Miosix with the xram linker scripts .data and .bss
    //are placed in the external RAM, so we *must* call SystemInit(), which
    //enables xram, before touching .data and .bss
    //Third, this is a performance improvement since the loops that initialize
    //.data and zeros .bss now run with the CPU at full speed instead of 8MHz
    SystemInit();
    startPll();

    miosix::IRQconfigureCache();

    //These are defined in the linker script
    extern unsigned char _etext asm("_etext");
    extern unsigned char _data asm("_data");
    extern unsigned char _edata asm("_edata");
    extern unsigned char _bss_start asm("_bss_start");
    extern unsigned char _bss_end asm("_bss_end");

    //Initialize .data section, clear .bss section
    unsigned char *etext=&_etext;
    unsigned char *data=&_data;
    unsigned char *edata=&_edata;
    unsigned char *bss_start=&_bss_start;
    unsigned char *bss_end=&_bss_end;
    memcpy(data, etext, edata-data);
    memset(bss_start, 0, bss_end-bss_start);

    //Move on to stage 2
    _init();

    //If main returns, reboot
    NVIC_SystemReset();
    for(;;) ;
}

/**
 * Reset handler, called by hardware immediately after reset
 */
void Reset_Handler() __attribute__((__interrupt__, noreturn));
void Reset_Handler()
{
    /*
     * Initialize process stack and switch to it.
     * This is required for booting Miosix, a small portion of the top of the
     * heap area will be used as stack until the first thread starts. After,
     * this stack will be abandoned and the process stack will point to the
     * current thread's stack.
     */
    asm volatile("ldr r0,  =_heap_end          \n\t"
                 "msr psp, r0                  \n\t"
                 "movw r0, #2                  \n\n" //Privileged, process stack
                 "msr control, r0              \n\t"
                 "isb                          \n\t":::"r0");

    program_startup();
}

/**
 * All unused interrupts call this function.
 */
extern "C" void Default_Handler() 
{
    unexpectedInterrupt();
}

//System handlers
void /*__attribute__((weak))*/ Reset_Handler();     //These interrupts are not
void /*__attribute__((weak))*/ NMI_Handler();       //weak because they are
void /*__attribute__((weak))*/ HardFault_Handler(); //surely defined by Miosix
void /*__attribute__((weak))*/ MemManage_Handler();
void /*__attribute__((weak))*/ BusFault_Handler();
void /*__attribute__((weak))*/ UsageFault_Handler();
void /*__attribute__((weak))*/ SVC_Handler();
void /*__attribute__((weak))*/ DebugMon_Handler();
void /*__attribute__((weak))*/ PendSV_Handler();
void /*__attribute__((weak))*/ SysTick_Handler();

//Interrupt handlers
void __attribute__((weak)) WWDG_IRQHandler();
void __attribute__((weak)) PVD_AVD_IRQHandler();
void __attribute__((weak)) TAMP_STAMP_IRQHandler();
void __attribute__((weak)) RTC_WKUP_IRQHandler();
void __attribute__((weak)) FLASH_IRQHandler();
void __attribute__((weak)) RCC_IRQHandler();
void __attribute__((weak)) EXTI0_IRQHandler();
void __attribute__((weak)) EXTI1_IRQHandler();
void __attribute__((weak)) EXTI2_IRQHandler();
void __attribute__((weak)) EXTI3_IRQHandler();
void __attribute__((weak)) EXTI4_IRQHandler();
void __attribute__((weak)) DMA1_Stream0_IRQHandler();
void __attribute__((weak)) DMA1_Stream1_IRQHandler();
void __attribute__((weak)) DMA1_Stream2_IRQHandler();
void __attribute__((weak)) DMA1_Stream3_IRQHandler();
void __attribute__((weak)) DMA1_Stream4_IRQHandler();
void __attribute__((weak)) DMA1_Stream5_IRQHandler();
void __attribute__((weak)) DMA1_Stream6_IRQHandler();
void __attribute__((weak)) ADC_IRQHandler();
void __attribute__((weak)) FDCAN1_IT0_IRQHandler();
void __attribute__((weak)) FDCAN2_IT0_IRQHandler();
void __attribute__((weak)) FDCAN1_IT1_IRQHandler();
void __attribute__((weak)) FDCAN2_IT1_IRQHandler();
void __attribute__((weak)) EXTI9_5_IRQHandler();
void __attribute__((weak)) TIM1_BRK_IRQHandler();
void __attribute__((weak)) TIM1_UP_IRQHandler();
void __attribute__((weak)) TIM1_TRG_COM_IRQHandler();
void __attribute__((weak)) TIM1_CC_IRQHandler();
void __attribute__((weak)) TIM2_IRQHandler();
void __attribute__((weak)) TIM3_IRQHandler();
void __attribute__((weak)) TIM4_IRQHandler();
void __attribute__((weak)) I2C1_EV_IRQHandler();
void __attribute__((weak)) I2C1_ER_IRQHandler();
void __attribute__((weak)) I2C2_EV_IRQHandler();
void __attribute__((weak)) I2C2_ER_IRQHandler();
void __attribute__((weak)) SPI1_IRQHandler();
void __attribute__((weak)) SPI2_IRQHandler();
void __attribute__((weak)) USART1_IRQHandler();
void __attribute__((weak)) USART2_IRQHandler();
void __attribute__((weak)) USART3_IRQHandler();
void __attribute__((weak)) EXTI15_10_IRQHandler();
void __attribute__((weak)) RTC_Alarm_IRQHandler();
void __attribute__((weak)) TIM8_BRK_TIM12_IRQHandler();
void __attribute__((weak)) TIM8_UP_TIM13_IRQHandler();
void __attribute__((weak)) TIM8_TRG_COM_TIM14_IRQHandler();
void __attribute__((weak)) TIM8_CC_IRQHandler();
void __attribute__((weak)) DMA1_Stream7_IRQHandler();
void __attribute__((weak)) FMC_IRQHandler();
void __attribute__((weak)) SDMMC1_IRQHandler();
void __attribute__((weak)) TIM5_IRQHandler();
void __attribute__((weak)) SPI3_IRQHandler();
void __attribute__((weak)) UART4_IRQHandler();
void __attribute__((weak)) UART5_IRQHandler();
void __attribute__((weak)) TIM6_DAC_IRQHandler();
void __attribute__((weak)) TIM7_IRQHandler();
void __attribute__((weak)) DMA2_Stream0_IRQHandler();
void __attribute__((weak)) DMA2_Stream1_IRQHandler();
void __attribute__((weak)) DMA2_Stream2_IRQHandler();
void __attribute__((weak)) DMA2_Stream3_IRQHandler();
void __attribute__((weak)) DMA2_Stream4_IRQHandler();
void __attribute__((weak)) ETH_IRQHandler();
void __attribute__((weak)) ETH_WKUP_IRQHandler();
void __attribute__((weak)) FDCAN_CAL_IRQHandler();
void __attribute__((weak)) DMA2_Stream5_IRQHandler();
void __attribute__((weak)) DMA2_Stream6_IRQHandler();
void __attribute__((weak)) DMA2_Stream7_IRQHandler();
void __attribute__((weak)) USART6_IRQHandler();
void __attribute__((weak)) I2C3_EV_IRQHandler();
void __attribute__((weak)) I2C3_ER_IRQHandler();
void __attribute__((weak)) OTG_HS_EP1_OUT_IRQHandler();
void __attribute__((weak)) OTG_HS_EP1_IN_IRQHandler();
void __attribute__((weak)) OTG_HS_WKUP_IRQHandler();
void __attribute__((weak)) OTG_HS_IRQHandler();
void __attribute__((weak)) DCMI_IRQHandler();
void __attribute__((weak)) RNG_IRQHandler();
void __attribute__((weak)) FPU_IRQHandler();
void __attribute__((weak)) UART7_IRQHandler();
void __attribute__((weak)) UART8_IRQHandler();
void __attribute__((weak)) SPI4_IRQHandler();
void __attribute__((weak)) SPI5_IRQHandler();
void __attribute__((weak)) SPI6_IRQHandler();
void __attribute__((weak)) SAI1_IRQHandler();
void __attribute__((weak)) LTDC_IRQHandler();
void __attribute__((weak)) LTDC_ER_IRQHandler();
void __attribute__((weak)) DMA2D_IRQHandler();
void __attribute__((weak)) SAI2_IRQHandler();
void __attribute__((weak)) QUADSPI_IRQHandler();
void __attribute__((weak)) LPTIM1_IRQHandler();
void __attribute__((weak)) CEC_IRQHandler();
void __attribute__((weak)) I2C4_EV_IRQHandler();
void __attribute__((weak)) I2C4_ER_IRQHandler();
void __attribute__((weak)) SPDIF_RX_IRQHandler();
void __attribute__((weak)) OTG_FS_EP1_OUT_IRQHandler();
void __attribute__((weak)) OTG_FS_EP1_IN_IRQHandler();
void __attribute__((weak)) OTG_FS_WKUP_IRQHandler();
void __attribute__((weak)) OTG_FS_IRQHandler();
void __attribute__((weak)) DMAMUX1_OVR_IRQHandler();
void __attribute__((weak)) HRTIM1_Master_IRQHandler();
void __attribute__((weak)) HRTIM1_TIMA_IRQHandler();
void __attribute__((weak)) HRTIM1_TIMB_IRQHandler();
void __attribute__((weak)) HRTIM1_TIMC_IRQHandler();
void __attribute__((weak)) HRTIM1_TIMD_IRQHandler();
void __attribute__((weak)) HRTIM1_TIME_IRQHandler();
void __attribute__((weak)) HRTIM1_FLT_IRQHandler();
void __attribute__((weak)) DFSDM1_FLT0_IRQHandler();
void __attribute__((weak)) DFSDM1_FLT1_IRQHandler();
void __attribute__((weak)) DFSDM1_FLT2_IRQHandler();
void __attribute__((weak)) DFSDM1_FLT3_IRQHandler();
void __attribute__((weak)) SAI3_IRQHandler();
void __attribute__((weak)) SWPMI1_IRQHandler();
void __attribute__((weak)) TIM15_IRQHandler();
void __attribute__((weak)) TIM16_IRQHandler();
void __attribute__((weak)) TIM17_IRQHandler();
void __attribute__((weak)) MDIOS_WKUP_IRQHandler();
void __attribute__((weak)) MDIOS_IRQHandler();
void __attribute__((weak)) JPEG_IRQHandler();
void __attribute__((weak)) MDMA_IRQHandler();
void __attribute__((weak)) SDMMC2_IRQHandler();
void __attribute__((weak)) HSEM1_IRQHandler();
void __attribute__((weak)) ADC3_IRQHandler();
void __attribute__((weak)) DMAMUX2_OVR_IRQHandler();
void __attribute__((weak)) BDMA_Channel0_IRQHandler();
void __attribute__((weak)) BDMA_Channel1_IRQHandler();
void __attribute__((weak)) BDMA_Channel2_IRQHandler();
void __attribute__((weak)) BDMA_Channel3_IRQHandler();
void __attribute__((weak)) BDMA_Channel4_IRQHandler();
void __attribute__((weak)) BDMA_Channel5_IRQHandler();
void __attribute__((weak)) BDMA_Channel6_IRQHandler();
void __attribute__((weak)) BDMA_Channel7_IRQHandler();
void __attribute__((weak)) COMP1_IRQHandler();
void __attribute__((weak)) LPTIM2_IRQHandler();
void __attribute__((weak)) LPTIM3_IRQHandler();
void __attribute__((weak)) LPTIM4_IRQHandler();
void __attribute__((weak)) LPTIM5_IRQHandler();
void __attribute__((weak)) LPUART1_IRQHandler();
void __attribute__((weak)) CRS_IRQHandler();
void __attribute__((weak)) ECC_IRQHandler();
void __attribute__((weak)) SAI4_IRQHandler();
void __attribute__((weak)) WAKEUP_PIN_IRQHandler();

//Stack top, defined in the linker script
extern char _main_stack_top asm("_main_stack_top");

//Interrupt vectors, must be placed @ address 0x00000000
//The extern declaration is required otherwise g++ optimizes it out
extern void (* const __Vectors[])();
void (* const __Vectors[])() __attribute__ ((section(".isr_vector"))) =
{
    reinterpret_cast<void (*)()>(&_main_stack_top),/* Stack pointer*/
    Reset_Handler,              /* Reset Handler */
    NMI_Handler,                /* NMI Handler */
    HardFault_Handler,          /* Hard Fault Handler */
    MemManage_Handler,          /* MPU Fault Handler */
    BusFault_Handler,           /* Bus Fault Handler */
    UsageFault_Handler,         /* Usage Fault Handler */
    0,                          /* Reserved */
    0,                          /* Reserved */
    0,                          /* Reserved */
    0,                          /* Reserved */
    SVC_Handler,                /* SVCall Handler */
    DebugMon_Handler,           /* Debug Monitor Handler */
    0,                          /* Reserved */
    PendSV_Handler,             /* PendSV Handler */
    SysTick_Handler,            /* SysTick Handler */

    /* External Interrupts */
    WWDG_IRQHandler,
    PVD_AVD_IRQHandler,
    TAMP_STAMP_IRQHandler,
    RTC_WKUP_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_Stream0_IRQHandler,
    DMA1_Stream1_IRQHandler,
    DMA1_Stream2_IRQHandler,
    DMA1_Stream3_IRQHandler,
    DMA1_Stream4_IRQHandler,
    DMA1_Stream5_IRQHandler,
    DMA1_Stream6_IRQHandler,
    ADC_IRQHandler,
    FDCAN1_IT0_IRQHandler,
    FDCAN2_IT0_IRQHandler,
    FDCAN1_IT1_IRQHandler,
    FDCAN2_IT1_IRQHandler,
    EXTI9_5_IRQHandler,
    TIM1_BRK_IRQHandler,
    TIM1_UP_IRQHandler,
    TIM1_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM4_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    USART3_IRQHandler,
    EXTI15_10_IRQHandler,
    RTC_Alarm_IRQHandler,
    0,
    TIM8_BRK_TIM12_IRQHandler,
    TIM8_UP_TIM13_IRQHandler,
    TIM8_TRG_COM_TIM14_IRQHandler,
    TIM8_CC_IRQHandler,
    DMA1_Stream7_IRQHandler,
    FMC_IRQHandler,
    SDMMC1_IRQHandler,
    TIM5_IRQHandler,
    SPI3_IRQHandler,
    UART4_IRQHandler,
    UART5_IRQHandler,
    TIM6_DAC_IRQHandler,
    TIM7_IRQHandler,
    DMA2_Stream0_IRQHandler,
    DMA2_Stream1_IRQHandler,
    DMA2_Stream2_IRQHandler,
    DMA2_Stream3_IRQHandler,
    DMA2_Stream4_IRQHandler,
    ETH_IRQHandler,
    ETH_WKUP_IRQHandler,
    FDCAN_CAL_IRQHandler,
    0,
    0,
    0,
    0,
    DMA2_Stream5_IRQHandler,
    DMA2_Stream6_IRQHandler,
    DMA2_Stream7_IRQHandler,
    USART6_IRQHandler,
    I2C3_EV_IRQHandler,
    I2C3_ER_IRQHandler,
    OTG_HS_EP1_OUT_IRQHandler,
    OTG_HS_EP1_IN_IRQHandler,
    OTG_HS_WKUP_IRQHandler,
    OTG_HS_IRQHandler,
    DCMI_IRQHandler,
    0,
    RNG_IRQHandler,
    FPU_IRQHandler,
    UART7_IRQHandler,
    UART8_IRQHandler,
    SPI4_IRQHandler,
    SPI5_IRQHandler,
    SPI6_IRQHandler,
    SAI1_IRQHandler,
    LTDC_IRQHandler,
    LTDC_ER_IRQHandler,
    DMA2D_IRQHandler,
    SAI2_IRQHandler,
    QUADSPI_IRQHandler,
    LPTIM1_IRQHandler,
    CEC_IRQHandler,
    I2C4_EV_IRQHandler,
    I2C4_ER_IRQHandler,
    SPDIF_RX_IRQHandler,
    OTG_FS_EP1_OUT_IRQHandler,
    OTG_FS_EP1_IN_IRQHandler,
    OTG_FS_WKUP_IRQHandler,
    OTG_FS_IRQHandler,
    DMAMUX1_OVR_IRQHandler,
    HRTIM1_Master_IRQHandler,
    HRTIM1_TIMA_IRQHandler,
    HRTIM1_TIMB_IRQHandler,
    HRTIM1_TIMC_IRQHandler,
    HRTIM1_TIMD_IRQHandler,
    HRTIM1_TIME_IRQHandler,
    HRTIM1_FLT_IRQHandler,
    DFSDM1_FLT0_IRQHandler,
    DFSDM1_FLT1_IRQHandler,
    DFSDM1_FLT2_IRQHandler,
    DFSDM1_FLT3_IRQHandler,
    SAI3_IRQHandler,
    SWPMI1_IRQHandler,
    TIM15_IRQHandler,
    TIM16_IRQHandler,
    TIM17_IRQHandler,
    MDIOS_WKUP_IRQHandler,
    MDIOS_IRQHandler,
    JPEG_IRQHandler,
    MDMA_IRQHandler,
    0,
    SDMMC2_IRQHandler,
    HSEM1_IRQHandler,
    0,
    ADC3_IRQHandler,
    DMAMUX2_OVR_IRQHandler,
    BDMA_Channel0_IRQHandler,
    BDMA_Channel1_IRQHandler,
    BDMA_Channel2_IRQHandler,
    BDMA_Channel3_IRQHandler,
    BDMA_Channel4_IRQHandler,
    BDMA_Channel5_IRQHandler,
    BDMA_Channel6_IRQHandler,
    BDMA_Channel7_IRQHandler,
    COMP1_IRQHandler,
    LPTIM2_IRQHandler,
    LPTIM3_IRQHandler,
    LPTIM4_IRQHandler,
    LPTIM5_IRQHandler,
    LPUART1_IRQHandler,
    0,
    CRS_IRQHandler,
    ECC_IRQHandler,
    SAI4_IRQHandler,
    0,
    0,
    WAKEUP_PIN_IRQHandler
};

#pragma weak WWDG_IRQHandler = Default_Handler
#pragma weak PVD_AVD_IRQHandler = Default_Handler
#pragma weak TAMP_STAMP_IRQHandler = Default_Handler
#pragma weak RTC_WKUP_IRQHandler = Default_Handler
#pragma weak FLASH_IRQHandler = Default_Handler
#pragma weak RCC_IRQHandler = Default_Handler
#pragma weak EXTI0_IRQHandler = Default_Handler
#pragma weak EXTI1_IRQHandler = Default_Handler
#pragma weak EXTI2_IRQHandler = Default_Handler
#pragma weak EXTI3_IRQHandler = Default_Handler
#pragma weak EXTI4_IRQHandler = Default_Handler
#pragma weak DMA1_Stream0_IRQHandler = Default_Handler
#pragma weak DMA1_Stream1_IRQHandler = Default_Handler
#pragma weak DMA1_Stream2_IRQHandler = Default_Handler
#pragma weak DMA1_Stream3_IRQHandler = Default_Handler
#pragma weak DMA1_Stream4_IRQHandler = Default_Handler
#pragma weak DMA1_Stream5_IRQHandler = Default_Handler
#pragma weak DMA1_Stream6_IRQHandler = Default_Handler
#pragma weak ADC_IRQHandler = Default_Handler
#pragma weak FDCAN1_IT0_IRQHandler = Default_Handler
#pragma weak FDCAN2_IT0_IRQHandler = Default_Handler
#pragma weak FDCAN1_IT1_IRQHandler = Default_Handler
#pragma weak FDCAN2_IT1_IRQHandler = Default_Handler
#pragma weak EXTI9_5_IRQHandler = Default_Handler
#pragma weak TIM1_BRK_IRQHandler = Default_Handler
#pragma weak TIM1_UP_IRQHandler = Default_Handler
#pragma weak TIM1_TRG_COM_IRQHandler = Default_Handler
#pragma weak TIM1_CC_IRQHandler = Default_Handler
#pragma weak TIM2_IRQHandler = Default_Handler
#pragma weak TIM3_IRQHandler = Default_Handler
#pragma weak TIM4_IRQHandler = Default_Handler
#pragma weak I2C1_EV_IRQHandler = Default_Handler
#pragma weak I2C1_ER_IRQHandler = Default_Handler
#pragma weak I2C2_EV_IRQHandler = Default_Handler
#pragma weak I2C2_ER_IRQHandler = Default_Handler
#pragma weak SPI1_IRQHandler = Default_Handler
#pragma weak SPI2_IRQHandler = Default_Handler
#pragma weak USART1_IRQHandler = Default_Handler
#pragma weak USART2_IRQHandler = Default_Handler
#pragma weak USART3_IRQHandler = Default_Handler
#pragma weak EXTI15_10_IRQHandler = Default_Handler
#pragma weak RTC_Alarm_IRQHandler = Default_Handler
#pragma weak TIM8_BRK_TIM12_IRQHandler = Default_Handler
#pragma weak TIM8_UP_TIM13_IRQHandler = Default_Handler
#pragma weak TIM8_TRG_COM_TIM14_IRQHandler = Default_Handler
#pragma weak TIM8_CC_IRQHandler = Default_Handler
#pragma weak DMA1_Stream7_IRQHandler = Default_Handler
#pragma weak FMC_IRQHandler = Default_Handler
#pragma weak SDMMC1_IRQHandler = Default_Handler
#pragma weak TIM5_IRQHandler = Default_Handler
#pragma weak SPI3_IRQHandler = Default_Handler
#pragma weak UART4_IRQHandler = Default_Handler
#pragma weak UART5_IRQHandler = Default_Handler
#pragma weak TIM6_DAC_IRQHandler = Default_Handler
#pragma weak TIM7_IRQHandler = Default_Handler
#pragma weak DMA2_Stream0_IRQHandler = Default_Handler
#pragma weak DMA2_Stream1_IRQHandler = Default_Handler
#pragma weak DMA2_Stream2_IRQHandler = Default_Handler
#pragma weak DMA2_Stream3_IRQHandler = Default_Handler
#pragma weak DMA2_Stream4_IRQHandler = Default_Handler
#pragma weak ETH_IRQHandler = Default_Handler
#pragma weak ETH_WKUP_IRQHandler = Default_Handler
#pragma weak FDCAN_CAL_IRQHandler = Default_Handler
#pragma weak DMA2_Stream5_IRQHandler = Default_Handler
#pragma weak DMA2_Stream6_IRQHandler = Default_Handler
#pragma weak DMA2_Stream7_IRQHandler = Default_Handler
#pragma weak USART6_IRQHandler = Default_Handler
#pragma weak I2C3_EV_IRQHandler = Default_Handler
#pragma weak I2C3_ER_IRQHandler = Default_Handler
#pragma weak OTG_HS_EP1_OUT_IRQHandler = Default_Handler
#pragma weak OTG_HS_EP1_IN_IRQHandler = Default_Handler
#pragma weak OTG_HS_WKUP_IRQHandler = Default_Handler
#pragma weak OTG_HS_IRQHandler = Default_Handler
#pragma weak DCMI_IRQHandler = Default_Handler
#pragma weak RNG_IRQHandler = Default_Handler
#pragma weak FPU_IRQHandler = Default_Handler
#pragma weak UART7_IRQHandler = Default_Handler
#pragma weak UART8_IRQHandler = Default_Handler
#pragma weak SPI4_IRQHandler = Default_Handler
#pragma weak SPI5_IRQHandler = Default_Handler
#pragma weak SPI6_IRQHandler = Default_Handler
#pragma weak SAI1_IRQHandler = Default_Handler
#pragma weak LTDC_IRQHandler = Default_Handler
#pragma weak LTDC_ER_IRQHandler = Default_Handler
#pragma weak DMA2D_IRQHandler = Default_Handler
#pragma weak SAI2_IRQHandler = Default_Handler
#pragma weak QUADSPI_IRQHandler = Default_Handler
#pragma weak LPTIM1_IRQHandler = Default_Handler
#pragma weak CEC_IRQHandler = Default_Handler
#pragma weak I2C4_EV_IRQHandler = Default_Handler
#pragma weak I2C4_ER_IRQHandler = Default_Handler
#pragma weak SPDIF_RX_IRQHandler = Default_Handler
#pragma weak OTG_FS_EP1_OUT_IRQHandler = Default_Handler
#pragma weak OTG_FS_EP1_IN_IRQHandler = Default_Handler
#pragma weak OTG_FS_WKUP_IRQHandler = Default_Handler
#pragma weak OTG_FS_IRQHandler = Default_Handler
#pragma weak DMAMUX1_OVR_IRQHandler = Default_Handler
#pragma weak HRTIM1_Master_IRQHandler = Default_Handler
#pragma weak HRTIM1_TIMA_IRQHandler = Default_Handler
#pragma weak HRTIM1_TIMB_IRQHandler = Default_Handler
#pragma weak HRTIM1_TIMC_IRQHandler = Default_Handler
#pragma weak HRTIM1_TIMD_IRQHandler = Default_Handler
#pragma weak HRTIM1_TIME_IRQHandler = Default_Handler
#pragma weak HRTIM1_FLT_IRQHandler = Default_Handler
#pragma weak DFSDM1_FLT0_IRQHandler = Default_Handler
#pragma weak DFSDM1_FLT1_IRQHandler = Default_Handler
#pragma weak DFSDM1_FLT2_IRQHandler = Default_Handler
#pragma weak DFSDM1_FLT3_IRQHandler = Default_Handler
#pragma weak SAI3_IRQHandler = Default_Handler
#pragma weak SWPMI1_IRQHandler = Default_Handler
#pragma weak TIM15_IRQHandler = Default_Handler
#pragma weak TIM16_IRQHandler = Default_Handler
#pragma weak TIM17_IRQHandler = Default_Handler
#pragma weak MDIOS_WKUP_IRQHandler = Default_Handler
#pragma weak MDIOS_IRQHandler = Default_Handler
#pragma weak JPEG_IRQHandler = Default_Handler
#pragma weak MDMA_IRQHandler = Default_Handler
#pragma weak SDMMC2_IRQHandler = Default_Handler
#pragma weak HSEM1_IRQHandler = Default_Handler
#pragma weak ADC3_IRQHandler = Default_Handler
#pragma weak DMAMUX2_OVR_IRQHandler = Default_Handler
#pragma weak BDMA_Channel0_IRQHandler = Default_Handler
#pragma weak BDMA_Channel1_IRQHandler = Default_Handler
#pragma weak BDMA_Channel2_IRQHandler = Default_Handler
#pragma weak BDMA_Channel3_IRQHandler = Default_Handler
#pragma weak BDMA_Channel4_IRQHandler = Default_Handler
#pragma weak BDMA_Channel5_IRQHandler = Default_Handler
#pragma weak BDMA_Channel6_IRQHandler = Default_Handler
#pragma weak BDMA_Channel7_IRQHandler = Default_Handler
#pragma weak COMP1_IRQHandler = Default_Handler
#pragma weak LPTIM2_IRQHandler = Default_Handler
#pragma weak LPTIM3_IRQHandler = Default_Handler
#pragma weak LPTIM4_IRQHandler = Default_Handler
#pragma weak LPTIM5_IRQHandler = Default_Handler
#pragma weak LPUART1_IRQHandler = Default_Handler
#pragma weak CRS_IRQHandler = Default_Handler
#pragma weak ECC_IRQHandler = Default_Handler
#pragma weak SAI4_IRQHandler = Default_Handler
#pragma weak WAKEUP_PIN_IRQHandler = Default_Handler
