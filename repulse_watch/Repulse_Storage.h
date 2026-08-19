#pragma once

#include <stdint.h>

bool Repulse_Storage_Init(void);
bool Repulse_Storage_Ready(void);
uint32_t Repulse_Storage_Card_Size_MB(void);
bool Repulse_Storage_Append(const char *csv_line);
