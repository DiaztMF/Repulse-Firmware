#pragma once

/*
 * Jalur UART dari gelang ESP32-C3.
 *
 * Layar tidak memutuskan apa pun (PRD §4.1 Aturan 1). Tangga eskalasi hidup
 * di gelang; di sini ia hanya digambar. Kalau baris berhenti datang, UI
 * kembali ke mode demo sendiri supaya panel juri tetap bisa dipakai.
 */

void Repulse_Link_Init(void);
void Repulse_Link_Loop(void);
