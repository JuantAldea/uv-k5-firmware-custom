/* Radiosonde Application Module for UV-K5
 * Self-contained main loop (like APP_RunSpectrum)
 * Handles BK4819 tuning, bit recovery simulation, and RS41 decoding
 */

#include "radiosonde.h"
#include "rs41.h"
#ifdef ENABLE_APRS
// Must come before driver/bk4819.h: both define APRS_MAX_PACKET_SIZE, and
// only bk4819.h's #define is guarded against a prior definition.
#include "aprs_minimal.h"
#endif
#include "../driver/eeprom.h"
#include "../driver/system.h"
#include "../driver/uart.h"  // SYSTEM_DelayMs — already included below, but needed for safe write
#include <string.h>

// aprs's EEPROM_WriteBuffer always burns a full 8-byte row at once (no length
// argument); pad with zeros since 0x0E28 is a standalone free row (ends right
// before APRS_EE_A at 0x0E30 — see settings.c), so there is nothing adjacent
// to disturb.
static void Sonde_SafeEEPROMWrite(uint32_t address, const void *buf, uint8_t size) {
    uint8_t row[8] = {0};
    memcpy(row, buf, size);
    EEPROM_WriteBuffer(address, row);
}

// Only the tuned frequency is persisted here: the aprs tree's EEPROM map
// (settings.c) leaves just one genuinely free 8-byte row at 0x0E28 (0x0E30
// is APRS_EE_A) — not enough room for a restored position/altitude/sonde ID
// the way the upstream radiosonde build keeps them.
typedef struct {
    uint32_t frequency;
} SavedSonde_t;

#ifdef ENABLE_RS41_QR
extern void miniqr_encode(const char *text, uint8_t qrcode[25][25]);
#endif



#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../driver/bk4819.h"
#include "../driver/uart.h"
#include "../driver/bk4819-regs.h"
#include "../driver/st7565.h"
#include "../driver/system.h"
#include "../driver/systick.h"
#include "../driver/keyboard.h"
#include "../driver/gpio.h"
#include "../driver/backlight.h"
#include "../misc.h"
#include "../radio.h"
#include "../settings.h"
#include "../ui/helper.h"
#include "../board.h"
#include "../driver/adc.h"
#include "../external/printf/printf.h"
#include "../font.h"
#include "../audio.h"
#include "../bsp/dp32g030/gpio.h"
#include "../bsp/dp32g030/portcon.h"
#include "../bsp/dp32g030/saradc.h"
#include "ARMCM0.h"
#include "../ui/ui.h"

// Default RS41 frequency range (400.000 - 406.000 MHz)
// BK4819 uses freq/10 format
#define SONDE_FREQ_START   40000000   // 400.000 MHz
#define SONDE_FREQ_END     40600000   // 406.000 MHz
#define SONDE_FREQ_DEFAULT 40300000   // 403.000 MHz default

static const uint32_t SONDE_STEPS[] = { 100000, 10000, 1000, 100 };
static void Sonde_DrawPixel(int x, int y, bool black);

#define SONDE_SCAN_DWELL_MS   2000   // 2s per channel (RS41 transmits every 1s)
#define SONDE_RSSI_THRESHOLD  500    // RSSI threshold for signal detect
#define SONDE_TIMEOUT_FRAMES  300    // ~30 seconds at 10 fps

SondeApp_t gSondeApp;

// ============================================================
// Display helpers
// ============================================================

static void Sonde_DrawStatusBar(void) {
    // Clear status line
    memset(gStatusLine, 0, sizeof(gStatusLine));

    // Title - "*" marks the APRS relay armed (PTT toggles it). Reusing this
    // same call for the indicator (rather than a separate draw) matters at
    // our flash margin: see the ENABLE_RS41 note in the Makefile.
#ifdef ENABLE_APRS
    UI_PrintStringSmallBufferNormal(gSondeApp.relay_armed ? "SONDE*" : "SONDE", gStatusLine);
#else
    UI_PrintStringSmallBufferNormal("SONDE", gStatusLine);
#endif

    // History Bar (Center)
    char hstr[12];
    uint8_t ptr = gSondeApp.history_ptr;
    for (int i = 0; i < 8; i++) {
        hstr[i] = (char)gSondeApp.history[(ptr + i) & 7];
    }
    hstr[8] = '\0';
    UI_PrintStringSmallBufferNormal(hstr, gStatusLine + 36);

    // RSSI Bar
    // Map RSSI (0-1000) to pixels (0-24)
    int rssi_len = (gSondeApp.last_rssi * 24) / 1000;
    if (rssi_len > 24) rssi_len = 24;
    for (int i = 0; i < rssi_len; i++) {
        for (int j = 1; j < 5; j++) {
            Sonde_DrawPixel(100 + i, j, true);
        }
    }
}


static void Sonde_DrawDiagnostic(void)
{
    char str[32];

    sprintf(str, "RSSI:%d P2P:%u", gSondeApp.last_rssi, gSondeApp.last_adc_p2p);
    GUI_DisplaySmallest(str, 0, 8, false, true);

    sprintf(str, "DC:%u G:%u %s", gSondeApp.adc_avg_x512 >> 9, gSondeApp.gain_mode, gSondeApp.adc_clipped ? "CLIP" : "");
    GUI_DisplaySmallest(str, 0, 16, false, true);

    const RS41_Decoder_t *dec = &gSondeApp.decoder;

    sprintf(str, "Min:%d Err:%d/%d", dec->min_errors, dec->last_errors, dec->last_errors_inv);
    GUI_DisplaySmallest(str, 0, 24, false, true);

    sprintf(str, "Rx:%d OK:%d", (int)dec->frames_received, (int)dec->frames_crc_ok);
    GUI_DisplaySmallest(str, 0, 32, false, true);

    sprintf(str, "Sh:%d Inv:%d P:%02X",
            dec->diag_best_shift,
            (int)dec->diag_best_invert,
            dec->diag_best_pos);
    GUI_DisplaySmallest(str, 0, 40, false, true);

    sprintf(str, "S:%02X P:%02X %04X/%04X",
            dec->diag_status_id,
            dec->diag_status_pos,
            dec->diag_status_crc_calc,
            dec->diag_status_crc_frame);
    GUI_DisplaySmallest(str, 0, 48, false, true);
}

static void Sonde_DrawData(const RS41_Data_t *d)
{
    char str[32];

    // Line 1: Sonde ID
    if (gSondeApp.decoder.frames_crc_ok == 0) {
        sprintf(str, "ID: %.8s (saved)", d->sonde_id);
    } else {
        sprintf(str, "ID: %.8s (live)", d->sonde_id);
    }
    GUI_DisplaySmallest(str, 0, 8, false, true);

    // Line 2: Latitude
    int32_t lat_deg = d->lat_1e6 / 1000000;
    int32_t lat_frac = d->lat_1e6 % 1000000;
    if (lat_frac < 0) lat_frac = -lat_frac;
    char lat_dir = (d->lat_1e6 >= 0) ? 'N' : 'S';
    sprintf(str, "Lat: %ld.%04ld %c", (long)lat_deg, (long)(lat_frac / 100), lat_dir);
    GUI_DisplaySmallest(str, 0, 16, false, true);

    // Line 3: Longitude
    int32_t lon_deg = d->lon_1e6 / 1000000;
    int32_t lon_frac = d->lon_1e6 % 1000000;
    if (lon_frac < 0) lon_frac = -lon_frac;
    char lon_dir = (d->lon_1e6 >= 0) ? 'E' : 'W';
    sprintf(str, "Lon: %ld.%04ld %c", (long)lon_deg, (long)(lon_frac / 100), lon_dir);
    GUI_DisplaySmallest(str, 0, 24, false, true);

    // Line 4: Altitude and AGC Gain
    int32_t alt_m = d->alt_cm / 100;
    const char* g_modes[] = {"AUTO", "LOW", "MID", "HIGH", "MAX"};
    sprintf(str, "Alt: %ldm   G:%s", (long)alt_m, g_modes[gSondeApp.gain_mode]);
    GUI_DisplaySmallest(str, 0, 32, false, true);

    // Line 5: Velocity
    int h_spd = d->vH_cm / 100;
    int v_spd = d->vV_cm / 100;
    sprintf(str, "Vel: H:%dm/s V:%dm/s", h_spd, v_spd);
    GUI_DisplaySmallest(str, 0, 40, false, true);

    // Line 6: GPS Satellites, Time, Battery
    // When showing restored data (no valid frames decoded yet), time/sat/batt are not saved — show placeholder
    if (gSondeApp.decoder.frames_crc_ok == 0) {  // Saved position
        GUI_DisplaySmallest("  --:--:-- (saved pos)", 0, 48, false, true);
    } else {
        sprintf(str, "Sat:%2u %02u:%02u:%02u %u.%uV",
                d->numSV, d->gps_hour, d->gps_min, d->gps_sec,
                d->batt_mv / 1000, (d->batt_mv % 1000) / 100);
        GUI_DisplaySmallest(str, 0, 48, false, true);
    }
}

static void Sonde_DrawPixel(int x, int y, bool black) {
    if (x < 0 || x >= 128 || y < 0 || y >= 64) return;

    if (y < 8) {
        // Line 0 is gStatusLine
        if (black) {
            gStatusLine[x] |= (1 << y);
        } else {
            gStatusLine[x] &= ~(1 << y);
        }
    } else {
        // Lines 1-7 are gFrameBuffer
        int line = (y / 8) - 1;
        int bit = y % 8;
        if (black) {
            gFrameBuffer[line][x] |= (1 << bit);
        } else {
            gFrameBuffer[line][x] &= ~(1 << bit);
        }
    }
}


#ifdef ENABLE_RS41_QR
static void Sonde_DrawQRCode(const RS41_Data_t *d) {
    if (!d->valid || (d->lat_1e6 == 0 && d->lon_1e6 == 0)) {
        GUI_DisplaySmallest("NO GPS DATA", 0, 24, false, true);
        return;
    }

    char url[64];
    int32_t lat_deg = d->lat_1e6 / 1000000;
    int32_t lat_frac = d->lat_1e6 % 1000000;
    if (lat_frac < 0) lat_frac = -lat_frac;
    int32_t lon_deg = d->lon_1e6 / 1000000;
    int32_t lon_frac = d->lon_1e6 % 1000000;
    if (lon_frac < 0) lon_frac = -lon_frac;

    // geo:lat,lon
    sprintf(url, "geo:%ld.%04ld,%ld.%04ld",
            (long)lat_deg, (long)(lat_frac / 100),
            (long)lon_deg, (long)(lon_frac / 100));

    static uint8_t qrcode[25][25];
    miniqr_encode(url, qrcode);

    int size = 25; // Fixed size for version 2
    int scale_x = 3; // 3x2 scaling compensates for the tall pixels of the UV-K5 LCD
    int scale_y = 2;

    // Center the QR code, but shift slightly up to avoid the bottom black bezel
    // and provide a guaranteed white quiet zone.
    int offset_x = (128 - size * scale_x) / 2;
    int offset_y = 6; // 6 pixels of white at top, perfectly centers the 50px tall QR code

    // Draw white background
    for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 128; x++) {
            Sonde_DrawPixel(x, y, false);
        }
    }

    // Draw QR
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            bool isDark = qrcode[y][x];
            for (int dy = 0; dy < scale_y; dy++) {
                for (int dx = 0; dx < scale_x; dx++) {
                    Sonde_DrawPixel(offset_x + x * scale_x + dx, offset_y + y * scale_y + dy, isDark);
                }
            }
        }
    }
}
#endif // ENABLE_RS41_QR


static void Sonde_Render(void)
{
    UI_DisplayClear();
    memset(gStatusLine, 0, sizeof(gStatusLine));

    char fstr[32];
    const char *step_str = (gSondeApp.step_idx == 0) ? "1M" :
                           (gSondeApp.step_idx == 1) ? "100k" :
                           (gSondeApp.step_idx == 2) ? "10k" : "1k";
    const char *lpf_names[] = {"WID", "MID", "NAR"};
    sprintf(fstr, "FREQ:%3u.%03u %s %s %s",
            gSondeApp.frequency / 100000,
            (gSondeApp.frequency % 100000) / 100,
            step_str,
            gSondeApp.hpf_compensation ? "INT" : "RAW",
            lpf_names[gSondeApp.lpf_mode % 3]);
    GUI_DisplaySmallest(fstr, 0, 0, false, true);

    Sonde_DrawStatusBar();

    const RS41_Data_t *d = RS41_GetData(&gSondeApp.decoder);

#ifdef ENABLE_RS41_QR
    if (gSondeApp.mode == SONDE_MODE_QR) {
        Sonde_DrawQRCode(d);
    } else
#endif
    if (gSondeApp.mode == SONDE_MODE_DIAGNOSTIC) {
        Sonde_DrawDiagnostic();
    } else {
        Sonde_DrawData(d);
    }

    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}
static void Sonde_ApplyLPFMode(uint8_t mode)
{
    uint16_t lpf_val = 4u; // Default: 4.5kHz (WID)
    if (mode == 1) {
        lpf_val = 0u; // 3.0kHz (MID)
    } else if (mode == 2) {
        lpf_val = 1u; // 2.5kHz (NAR)
    }

    BK4819_WriteRegister(0x43,
        (7u << 12) |  // RF filter bandwidth = 4.5kHz * 2
        (7u << 9)  |  // RF filter bandwidth weak = 4.5kHz * 2
        (lpf_val << 6) |  // AFTxLPF2
        (2u << 4)  |  // 25k BW mode
        (1u << 3)  |  // fixed
        (0u << 2)  |  // Normal gain (was +6dB, caused ADC clipping)
        (0u << 0));   // reserved
}

// ============================================================
// BK4819 setup for RS41 reception
// ============================================================

static void Sonde_SetupReceiver(uint32_t freq)
{
    gSondeApp.frequency = freq;

    // Disable all interrupts and squelch
    BK4819_WriteRegister(BK4819_REG_3F, 0);

    // Set frequency
    BK4819_SetFrequency(freq);
    // BK4819_REG_30: Power management.
    BK4819_WriteRegister(BK4819_REG_30, 0);
    BK4819_WriteRegister(BK4819_REG_30, 0xBFF1);

    // FM narrow demod, wider RF bandwidth for GFSK signal
    // RS41 uses 4800 baud FSK with 4.8kHz peak-to-peak deviation.
    // 12.5kHz or 25kHz filter is required (25kHz recommended for stability without AFC)
    BK4819_SetFilterBandwidth(BK4819_FILTER_BW_WIDE, true);

    // Apply current LPF filter bandwidth mode (WID/MID/NAR)
    Sonde_ApplyLPFMode(gSondeApp.lpf_mode);

    // (Removed GPIO0 override so it correctly works as RX_ENABLE)

    // Ensure AF output is not muted and has moderate gain
    // REG_48: <13> = Mute, <11:10> = Gain-1 (0), <9:4> = Gain-2 (30), <3:0> = DAC Gain (8)
    uint16_t reg_48 = BK4819_ReadRegister(0x48);
    reg_48 &= ~(1u << 13); // Unmute
    reg_48 = (reg_48 & ~0x0FF0) | (30u << 4); // Set moderate Gain-2 (30 out of 63)
    BK4819_WriteRegister(0x48, reg_48);

    // Disable RX AF filters for raw FSK data:
    uint16_t reg_2b = BK4819_ReadRegister(0x2B);
    reg_2b |= (1u << 10) | (1u << 9) | (1u << 8); // Disable high-pass, low-pass, de-emphasis
    BK4819_WriteRegister(0x2B, reg_2b);

    // Disable squelch
    BK4819_WriteRegister(BK4819_REG_4D, 0);
    BK4819_WriteRegister(BK4819_REG_4E, 0x206F);

    // Setup AGC for sensitive reception
    BK4819_InitAGC(false);
    BK4819_WriteRegister(0x13, 0x03FF); // Allow absolute maximum RF gain for weak signals
    BK4819_SetAGC(true);

    // Enable RX
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);

    // Ensure audio path stays on and AF is routed
    gEnableSpeaker = true;
    BK4819_SetAF(BK4819_AF_FM);
}

// ============================================================
// Read AF level from BK4819 for bit recovery
// Returns 1 or 0 based on FM demodulated output polarity
// ============================================================

__attribute__((unused)) static uint8_t Sonde_ReadBit(void)
{
    // REG_69: AF output level after FM demod
    // Bit 15 gives sign of FM deviation (frequency above or below center)
    uint16_t reg69 = BK4819_ReadRegister(0x69);
    return (reg69 & 0x8000) ? 1 : 0;
}

// ============================================================
// Keyboard handling
// Returns true if should exit
// ============================================================

static bool Sonde_HandleKeys(void)
{
#ifdef ENABLE_APRS
    // PTT isn't in KEY_Code_t here (KEYBOARD_Poll never returns KEY_PTT), so
    // it's read straight off the GPIO like the main app does. Edge-triggered:
    // arms/disarms the auto-relay in APP_RunRadiosonde, doesn't transmit here.
    static bool ptt_was_down = false;
    const bool ptt_down = !GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT);
    if (ptt_down && !ptt_was_down) {
        gSondeApp.relay_armed = !gSondeApp.relay_armed;
        AUDIO_PlayBeep(gSondeApp.relay_armed ? BEEP_440HZ_500MS : BEEP_500HZ_60MS_DOUBLE_BEEP);
    }
    ptt_was_down = ptt_down;
#endif

    KEY_Code_t key = KEYBOARD_Poll();

    if (key == KEY_INVALID)
        return false;

    // Debounce
    SYSTEM_DelayMs(50);

    switch (key) {
        case KEY_EXIT:
            return true;

        case KEY_UP:
            gSondeApp.frequency += SONDE_STEPS[gSondeApp.step_idx];
            if (gSondeApp.frequency > SONDE_FREQ_END)
                gSondeApp.frequency = SONDE_FREQ_START;
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;

        case KEY_DOWN:
            if (gSondeApp.frequency <= SONDE_FREQ_START + SONDE_STEPS[gSondeApp.step_idx])
                gSondeApp.frequency = SONDE_FREQ_END;
            else
                gSondeApp.frequency -= SONDE_STEPS[gSondeApp.step_idx];
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;

        case KEY_STAR:
            // Cycle step
            gSondeApp.step_idx = (gSondeApp.step_idx + 1) % 4;
            break;

        case KEY_0: // 400.000 MHz
            gSondeApp.frequency = 40000000;
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;
        case KEY_1: // 401.000 MHz
            gSondeApp.frequency = 40100000;
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;
        case KEY_2: // 402.000 MHz
            gSondeApp.frequency = 40200000;
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;
        case KEY_3: // 403.000 MHz
            gSondeApp.frequency = 40300000;
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;
        case KEY_4: // 404.000 MHz
            gSondeApp.frequency = 40400000;
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;
        case KEY_5: // 405.000 MHz
            gSondeApp.frequency = 40500000;
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;
        case KEY_6: // 406.000 MHz
            gSondeApp.frequency = 40600000;
            Sonde_SetupReceiver(gSondeApp.frequency);
            RS41_Reset(&gSondeApp.decoder);
            break;

        case KEY_7:
            gSondeApp.mode = SONDE_MODE_DIAGNOSTIC;
            gSondeApp.timeout_frames = 0;
            break;
        case KEY_8:
            gSondeApp.mode = SONDE_MODE_MONITOR;
            gSondeApp.timeout_frames = 0;
            break;
#ifdef ENABLE_RS41_QR
        case KEY_9:
            gSondeApp.mode = SONDE_MODE_QR;
            gSondeApp.timeout_frames = 0;
            break;
#endif

        case KEY_MENU:
            // Cycle Gain mode: Auto -> Low -> Mid -> High -> Max
            gSondeApp.gain_mode = (gSondeApp.gain_mode + 1) % 5;
            break;

        case KEY_F:
            // Toggle audio output
            gSondeApp.audio_enabled = !gSondeApp.audio_enabled;
            if (gSondeApp.audio_enabled) {
                AUDIO_AudioPathOn();
            } else {
                AUDIO_AudioPathOff();
            }
            break;

        case KEY_SIDE1:
            gSondeApp.hpf_compensation = !gSondeApp.hpf_compensation;
            break;

        case KEY_SIDE2:
            gSondeApp.lpf_mode = (gSondeApp.lpf_mode + 1) % 3;
            Sonde_ApplyLPFMode(gSondeApp.lpf_mode);
            break;

        default:
            break;
    }

    return false;
}

// SCAN MODE HAS BEEN REMOVED

// ============================================================
// Main application loop
// Takes over from main loop, similar to APP_RunSpectrum()
// ============================================================

void APP_RunRadiosonde(void)
{
    // Initialize application state
    memset(&gSondeApp, 0, sizeof(SondeApp_t));
    gSondeApp.decoder.data.valid = false;
    gSondeApp.timeout_frames = 0;
    gSondeApp.mode = SONDE_MODE_DIAGNOSTIC;
    gSondeApp.frequency = SONDE_FREQ_DEFAULT;
    gSondeApp.step_idx = 1; // 100 kHz default
    gSondeApp.gain_mode = 0; // Auto Gain
    gSondeApp.audio_enabled = true;
    memset(gSondeApp.history, ' ', 8);
    gSondeApp.history_ptr = 0;
    gSondeApp.pending_history = '.';
    gSondeApp.history_sample_acc = 0;

    RS41_Init(&gSondeApp.decoder);

    // Persistent demodulator state variables
    int32_t integrated = 0;
    int32_t integrated_avg = 0;
    int32_t centered_old = 0;
    int32_t centered_older = 0;
    int32_t phase = 0;
    int32_t bit_integrator = 0;
    int32_t zc_phase = 0;
    int confirmed_state = 1;
    bool zc_pending = false;

    uint16_t save_cooldown = 0;

    // Restore last tuned frequency from EEPROM (0x0E28) — see SavedSonde_t.
    SavedSonde_t saved;
    EEPROM_ReadBuffer(0x0E28, &saved, sizeof(saved));
    if (saved.frequency >= SONDE_FREQ_START && saved.frequency <= SONDE_FREQ_END) {
        gSondeApp.frequency = saved.frequency;
    }

    // ADC setup for signal monitoring (MCU Pin 9 - PA8 / UART1_RX -> ADC_CH3)
    PORTCON_PORTA_IE &= ~PORTCON_PORTA_IE_A8_MASK;
    PORTCON_PORTA_PU &= ~PORTCON_PORTA_PU_A8_MASK;   // Disable Pull-Up (relying on external 10k pull-up)
    PORTCON_PORTA_PD |= PORTCON_PORTA_PD_A8_MASK;   // Enable Pull-Down to create DC offset bias
    PORTCON_PORTA_SEL1 &= ~PORTCON_PORTA_SEL1_A8_MASK;
    PORTCON_PORTA_SEL1 |= PORTCON_PORTA_SEL1_A8_BITS_SARADC_CH3;

    // Enable backlight and setup receiver
    BACKLIGHT_TurnOn();
    Sonde_SetupReceiver(gSondeApp.frequency);
    AUDIO_AudioPathOn();
    Sonde_Render();

    // Main loop — runs until user presses EXIT
    while (!gSondeApp.exit_requested) {

        // Handle keyboard inputs
        if (Sonde_HandleKeys()) {
            gSondeApp.exit_requested = true;
            break;
        }

        if (gSondeApp.mode != SONDE_MODE_QR) {
            // Configure ADC to read CH3 (MCU Pin 9 - PA8)
            SARADC_CFG = (SARADC_CFG & ~SARADC_CFG_CH_SEL_MASK) |
                         ((ADC_CH3 << SARADC_CFG_CH_SEL_SHIFT) & SARADC_CFG_CH_SEL_MASK);

            uint32_t sample_count = 0;
            bool frame_decoded = false;
            int32_t local_min = 0;
            int32_t local_max = 0;
            bool adc_clipped = false;

            uint32_t start_val = SysTick->VAL;

            // Inner bit-recovery loop (processes up to 1 second of samples)
            while (1) {
                sample_count++;

                // Read ADC sample
                ADC_Start();
                while (!ADC_CheckEndOfConversion(ADC_CH3)) {}
                uint16_t adc_val = ADC_GetValue(ADC_CH3);

                // Check if raw signal is clipping the ADC
                if (adc_val < 10 || adc_val > 4085) {
                    adc_clipped = true;
                }

                // Dynamic DC threshold (slow EMA, alpha = 1/256)
                gSondeApp.last_adc_raw = adc_val;
                if (gSondeApp.adc_avg_x512 == 0) gSondeApp.adc_avg_x512 = (uint32_t)adc_val << 9;
                gSondeApp.adc_avg_x512 = (gSondeApp.adc_avg_x512 * 255 + ((uint32_t)adc_val << 9)) >> 8;

                int32_t centered = (int32_t)adc_val - (int32_t)(gSondeApp.adc_avg_x512 >> 9);

                int32_t signal = centered;
                if (gSondeApp.hpf_compensation) {
                    integrated = integrated - (integrated >> 8) + centered;
                    if (integrated_avg == 0 && integrated != 0) {
                        integrated_avg = integrated << 8;
                    }
                    integrated_avg = (integrated_avg * 255 + integrated) >> 8;
                    signal = integrated - (integrated_avg >> 8);
                } else {
                    integrated = 0;
                    integrated_avg = 0;
                }

                gSondeApp.last_amplitude = (uint16_t)((signal > 0) ? signal : -signal);

                // 3-tap LPF to smooth out noise spikes
                int32_t filtered = (signal + (centered_old << 1) + centered_older) >> 2;
                centered_older = centered_old;
                centered_old = signal;

                // Track min/max of the filtered signal for demodulation amplitude tracking
                if (filtered < local_min) local_min = filtered;
                if (filtered > local_max) local_max = filtered;

                // Dynamic Hysteresis based on average signal strength
                int32_t hysteresis = gSondeApp.p2p_avg >> 3;
                if (hysteresis < 30) hysteresis = 30;

                // DPLL timing recovery
                phase += 10;
                bit_integrator += filtered;

                // 1. Detect zero-crossing in the opposite direction of the confirmed state
                int sign = (filtered >= 0) ? 1 : -1;
                if (sign != confirmed_state && !zc_pending) {
                    zc_phase = phase;
                    zc_pending = true;
                }

                // 2. Confirm zero-crossing if it crosses the hysteresis threshold
                if (zc_pending) {
                    if (sign != confirmed_state) {
                        if ((sign == 1 && filtered > hysteresis) || (sign == -1 && filtered < -hysteresis)) {
                            int phase_error = zc_phase;
                            if (phase_error >= 40) phase_error -= 80;
                            phase -= phase_error / 2; // 50% loop gain
                            confirmed_state = sign;   // Update confirmed state
                            zc_pending = false;
                        }
                    } else {
                        // Signal returned to previous state before confirming - discard glitch
                        zc_pending = false;
                    }
                }

                if (phase >= 80) {
                    phase -= 80;
                    if (zc_pending) {
                        zc_phase -= 80;
                    }
                    int bit = (bit_integrator > 0) ? 1 : 0;
                    if (RS41_ProcessBit(&gSondeApp.decoder, bit)) {
                        frame_decoded = true;
                    }
                    bit_integrator = 0;
                }

                // Breakout conditions
                if (frame_decoded) break;
                // 8x oversampling: 200ms = 7680 samples, 1s = 38400 samples
                if (gSondeApp.decoder.state != RS41_STATE_COLLECT_FRAME && sample_count > 7680) break;
                if (sample_count > 38400) break;

                // Wait 26µs (1250 ticks at 48MHz) for 38400 Hz sampling rate
                while (1) {
                    uint32_t current_val = SysTick->VAL;
                    uint32_t elapsed = (start_val >= current_val) ? (start_val - current_val) : (SysTick->LOAD - current_val + start_val);
                    if (elapsed >= 1250) break;
                }
                // Advance start_val by 1250 for isochronous sampling (SysTick counts down)
                start_val = (start_val >= 1250) ? (start_val - 1250) : (SysTick->LOAD - (1250 - start_val) + 1);

                // --- History Update (Strictly once per 38400 samples, precisely 1 second) ---
                if (++gSondeApp.history_sample_acc >= 38400) {
                    gSondeApp.history_sample_acc = 0;
                    gSondeApp.history[gSondeApp.history_ptr] = (uint8_t)gSondeApp.pending_history;
                    gSondeApp.history_ptr = (gSondeApp.history_ptr + 1) & 7;
                    gSondeApp.pending_history = '.'; // Reset for next second
                }
            } // End of bit-recovery loop

            // Restore ADC for battery measurement
            SARADC_CFG = (SARADC_CFG & ~SARADC_CFG_CH_SEL_MASK) |
                         (((ADC_CH4 | ADC_CH9) << SARADC_CFG_CH_SEL_SHIFT) & SARADC_CFG_CH_SEL_MASK);

            gSondeApp.adc_clipped = adc_clipped;

            if (save_cooldown > 0) {
                save_cooldown--;
            }
#ifdef ENABLE_APRS
            if (gSondeApp.relay_cooldown > 0) {
                gSondeApp.relay_cooldown--;
            }
#endif

            // Update timeout counters and latch successful decodes
            if (frame_decoded) {
                gSondeApp.timeout_frames = 0;
                // Mark success for history latch
                gSondeApp.pending_history = (gSondeApp.decoder.data.lat_1e6 != 0) ? '|' : '-';

                // Auto-switch to Monitor if we have valid coordinates and are currently looking at Diagnostics
                if (gSondeApp.mode == SONDE_MODE_DIAGNOSTIC && gSondeApp.decoder.data.valid) {
                    gSondeApp.mode = SONDE_MODE_MONITOR;
                }

                // Periodic save to EEPROM with 15-second cooldown
                if (gSondeApp.decoder.data.valid && gSondeApp.decoder.data.lat_1e6 != 0) {
                    if (save_cooldown == 0) {
                        SavedSonde_t saved_data = { .frequency = gSondeApp.frequency };
                        Sonde_SafeEEPROMWrite(0x0E28, &saved_data, sizeof(saved_data));
                        save_cooldown = 15;
                    }
                }

#ifdef ENABLE_APRS
                // Relay the fix over APRS: armed (PTT toggle), a real fix, not
                // transmitting faster than once every 30s (shared-channel
                // etiquette - RS41 itself updates roughly every second, far
                // faster than APRS traffic should), and a real callsign set.
                if (gSondeApp.relay_armed && gSondeApp.relay_cooldown == 0 &&
                    gSondeApp.decoder.data.valid && gSondeApp.decoder.data.lat_1e6 != 0 &&
                    APRS_Configured()) {
                    APRS_BeaconItem(gSondeApp.decoder.data.sonde_id,
                                     gSondeApp.decoder.data.lat_1e6,
                                     gSondeApp.decoder.data.lon_1e6,
                                     gSondeApp.decoder.data.alt_cm);
                    gSondeApp.relay_cooldown = 30;
                    // APRS_BeaconItem reprogrammed BK4819 for Bell202 TX; tune
                    // back to RS41 RX before the next sampling pass.
                    Sonde_SetupReceiver(gSondeApp.frequency);
                    RS41_Reset(&gSondeApp.decoder);
                }
#endif
            } else {
                gSondeApp.timeout_frames++;

                // Auto-switch back to Diagnostics if signal is lost for 30 seconds
                // But NOT if we are displaying saved LAST GPS coordinates
                if (gSondeApp.timeout_frames > 30 && gSondeApp.mode == SONDE_MODE_MONITOR) {
                    if (gSondeApp.decoder.data.sonde_id[0] != 'L') {
                        gSondeApp.mode = SONDE_MODE_DIAGNOSTIC;
                    }
                }
            }

            // Calculate and filter Peak-to-Peak amplitude
            gSondeApp.last_adc_p2p = (local_max > local_min) ? (uint16_t)(local_max - local_min) : 0;
            if (gSondeApp.p2p_avg == 0) {
                gSondeApp.p2p_avg = gSondeApp.last_adc_p2p;
            } else {
                gSondeApp.p2p_avg = (gSondeApp.p2p_avg * 7 + gSondeApp.last_adc_p2p) >> 3;
            }

            // --- Software AGC / Manual Gain ---
            uint16_t reg_48 = BK4819_ReadRegister(0x48);
            uint8_t current_gain = (reg_48 >> 4) & 0x3F; // Extract Gain-2 (0-63)
            bool changed = false;

            if (gSondeApp.gain_mode == 0) {
                // Auto Gain Mode
                if (gSondeApp.last_rssi > 120) {
                    // Target P2P range: 600 - 1000
                    if (gSondeApp.p2p_avg < 600 && current_gain < 60) {
                        current_gain += 1;
                        changed = true;
                    } else if (gSondeApp.p2p_avg > 1000 && current_gain > 2) {
                        // Fast responsive decrease proportional to overflow
                        int32_t diff = (int32_t)(gSondeApp.p2p_avg - 1000) >> 9;
                        if (diff < 2) diff = 2;
                        if (diff > 15) diff = 15;
                        if (current_gain > diff) {
                            current_gain -= diff;
                        } else {
                            current_gain = 2;
                        }
                        changed = true;
                    }
                } else {
                    // No signal? Reset to a safe baseline gain (30)
                    if (current_gain != 30) {
                        current_gain = 30;
                        changed = true;
                    }
                }
            } else {
                // Manual Gain Mode
                // 1: LOW (10), 2: MID (30), 3: HIGH (50), 4: MAX (63)
                uint8_t target_gain = (gSondeApp.gain_mode == 1) ? 10 :
                                      (gSondeApp.gain_mode == 2) ? 30 :
                                      (gSondeApp.gain_mode == 3) ? 50 : 63;
                if (current_gain != target_gain) {
                    current_gain = target_gain;
                    changed = true;
                }
            }

            if (changed) {
                reg_48 = (reg_48 & ~0x03F0) | ((uint16_t)current_gain << 4);
                BK4819_WriteRegister(0x48, reg_48);
            }

        } else {
            // QR Mode: Pause processing, just wait and render
            SYSTEM_DelayMs(100);
        }

        // Update display with latest metrics
        gSondeApp.last_rssi = BK4819_GetRSSI();
        Sonde_Render();
        BACKLIGHT_TurnOn();
    } // End of main loop

    // Save the tuned frequency to EEPROM on clean exit.
    SavedSonde_t final_save = { .frequency = gSondeApp.frequency };
    Sonde_SafeEEPROMWrite(0x0E28, &final_save, sizeof(final_save));

    // ============================================================
    // Cleanup and hardware restoration on exit
    // ============================================================

    AUDIO_AudioPathOff();

    // Restore ADC multiplexer
    SARADC_CFG = (SARADC_CFG & ~SARADC_CFG_CH_SEL_MASK) |
                 (((ADC_CH4 | ADC_CH9) << SARADC_CFG_CH_SEL_SHIFT) & SARADC_CFG_CH_SEL_MASK);

    // Restore MCU Pin 9 - PA8 (UART1_RX)
    PORTCON_PORTA_SEL1 &= ~PORTCON_PORTA_SEL1_A8_MASK;
    PORTCON_PORTA_PU &= ~PORTCON_PORTA_PU_A8_MASK;
    PORTCON_PORTA_PD &= ~PORTCON_PORTA_PD_A8_MASK;

    // Reset BK4819 to normal mode
    BK4819_Init();
    RADIO_SetupRegisters(true);

    // Clear display and return to main UI seamlessly
    ST7565_FillScreen(0);
    GUI_SelectNextDisplay(DISPLAY_MAIN);
}
