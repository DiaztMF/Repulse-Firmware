#pragma once

#include <stdbool.h>
#include <stdint.h>

void Repulse_UI_Init(void);

/* Data terbaru dari gelang. Layar hanya menggambar — tangga eskalasi hidup
 * di firmware gelang (PRD §4.1 Aturan 1), jadi `stage` diterima apa adanya
 * dan tidak pernah dihitung di sini. Berhenti dipanggil selama 5 detik dan
 * UI kembali ke mode demo. */
void Repulse_UI_Feed(uint8_t bpm_value, uint8_t spo2_value, bool worn,
                     uint8_t stage, bool linked);
