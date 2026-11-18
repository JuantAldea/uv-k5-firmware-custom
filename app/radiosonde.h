/* Radiosonde Application Module for UV-K5
 * Provides the main application loop for RS41 decoding mode
 */

#ifndef APP_RADIOSONDE_H
#define APP_RADIOSONDE_H

#include <stdint.h>
#include <stdbool.h>
#include "rs41.h"

// Radiosonde app states
typedef enum {
    SONDE_MODE_DIAGNOSTIC,
    SONDE_MODE_MONITOR,
    SONDE_MODE_QR,
} SondeMode_t;


// Application context
typedef struct {
    SondeMode_t mode;
    RS41_Decoder_t decoder;
    uint32_t frequency;        // current frequency in Hz/10 (BK4819 format)

    uint8_t gain_mode;         // 0: Auto, 1: Low, 2: Mid, 3: High
    bool audio_enabled;        // Audio on/off

    uint16_t last_rssi;        // RSSI of current channel
    uint16_t last_adc_p2p;     // ADC peak-to-peak amplitude
    bool     adc_clipped;      // ADC clipping detected
    bool     exit_requested;   // user wants to exit
    uint16_t timeout_frames;   // frames without decode before timeout
    uint16_t last_adc_raw;     // raw ADC sample for debug
    uint32_t adc_avg_x512;     // DC average (accumulated)
    uint16_t last_amplitude;   // Peak amplitude after DC removal

    uint8_t  step_idx;         // Index into SONDE_STEPS
    uint16_t p2p_avg;          // Filtered P2P for AGC
    uint8_t  history[8];       // Decoding history symbols
    uint8_t  history_ptr;      // Pointer for history buffer
    char     pending_history;  // Latched symbol for the 1s cycle
    uint32_t history_sample_acc;
    bool     hpf_compensation;
    uint8_t  lpf_mode;         // 0: 4.5kHz (WID), 1: 3.0kHz (MID), 2: 2.5kHz (NAR)

#if defined(ENABLE_APRS) && defined(ENABLE_RS41_RELAY)
    bool     relay_armed;      // PTT toggles this: auto-relay fixes over APRS
    uint16_t relay_cooldown;   // seconds until the next relay is allowed
#endif
} SondeApp_t;

extern SondeApp_t gSondeApp;

// Main entry point — takes over the main loop (similar to APP_RunSpectrum)
void APP_RunRadiosonde(void);

#endif // APP_RADIOSONDE_H
