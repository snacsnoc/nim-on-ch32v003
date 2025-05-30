#ifndef NIM_HELPERS_H
#define NIM_HELPERS_H

#include <stdint.h> 

void NimHelper_PC5_Output_Init(void);
// Use 'int' to match 'cint' in Nim
void NimHelper_PC5_Write(int high); 
void NimHelper_DelayMs(uint32_t ms);

#endif
