#include "ch32v00x.h"  

// GPIOC-5 output @10 MHz, push-pull 
void NimHelper_PC5_Output_Init(void)
{
    // enable GPIOC peripheral clock (APB2PCENR bit 4) 
    RCC->APB2PCENR |= (1u << 4);                

    // configure PC5: MODE=01 (10 MHz), CNF=00 (GP push-pull)
    //        CFGLR holds config for pins 0-7  → field = pin*4 bits   

    // clear old 
    GPIOC->CFGLR &= ~(0xFu << (5 * 4));    
    // 0b0001 => 10 MHz PP      
    GPIOC->CFGLR |=  (0x1u << (5 * 4));         
}

// drive PC5 
void NimHelper_PC5_Write(int high)
{
    if (high)
    // set
        GPIOC->BSHR = (1u << 5);          
    else
    // release
        GPIOC->BSHR = (1u << (5 + 16));   
}

// crude busy-wait delay (≈1 ms @48 MHz) 
void NimHelper_DelayMs(uint32_t ms)
{
    for (; ms; --ms)
        for (volatile uint32_t i = 0; i < 7500; ++i)
            __asm__ volatile ("nop");
}