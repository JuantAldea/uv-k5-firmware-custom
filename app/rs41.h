/* RS41 Radiosonde Decoder for Quansheng UV-K5
 * Ported from rs1729/RS (rs41mod.c) - GPL License
 * Adapted for ARM Cortex-M0 (DP32G030) with limited RAM/Flash
 *
 * RS41 Protocol:
 *   - 4800 baud GFSK on 400-406 MHz
 *   - 320 byte frames, XOR scrambled
 *   - CRC-16 per block, Reed-Solomon ECC (48 bytes)
 *   - Blocks: STATUS(0x79), GPS_TIME(0x7C), GPS_POS(0x7B), PTU(0x7A)
 */

#ifndef APP_RS41_H
#define APP_RS41_H

#include <stdint.h>
#include <stdbool.h>

// RS41 frame constants
#define RS41_FRAME_LEN       320   // standard frame length
#define RS41_HEADER_LEN      8     // sync header bytes (after XOR descramble)
#define RS41_ECC_LEN         48    // Reed-Solomon ECC bytes (skipped)
#define RS41_XOR_MASK_LEN    64    // LFSR XOR mask length
#define RS41_PREAMBLE_BITS   320   // preamble 0101... bits
#define RS41_SYNC_BITS       64    // header sync = 8 bytes

// Block IDs
#define RS41_BLK_STATUS      0x79
#define RS41_BLK_MEAS        0x7A  // PTU measurements
#define RS41_BLK_GPS_INFO    0x7C  // GPS week + TOW
#define RS41_BLK_GPS_POS     0x7B  // ECEF position + velocity

// CRC bitmask for validation
#define RS41_CRC_STATUS      (1 << 0)
#define RS41_CRC_PTU         (1 << 1)
#define RS41_CRC_GPS1        (1 << 2)
#define RS41_CRC_GPS3        (1 << 3)

// Decoder states
typedef enum {
    RS41_STATE_WAIT_PREAMBLE,
    RS41_STATE_WAIT_HEADER,
    RS41_STATE_COLLECT_FRAME,
    RS41_STATE_FRAME_READY,
} RS41_State_t;

// Decoded telemetry data — minimal struct (~60 bytes without frame buffer)
typedef struct {
    // STATUS block
    uint16_t frame_nr;
    char     sonde_id[10];    // 8 chars + NUL + pad
    uint16_t batt_mv;         // battery millivolts

    // GPS TIME block (GPS1 / 0x7C)
    uint16_t gps_week;
    uint32_t gps_tow_ms;      // GPS time of week in ms
    uint8_t  gps_hour;
    uint8_t  gps_min;
    uint8_t  gps_sec;

    // GPS POSITION block (GPS3 / 0x7B)
    // Stored as fixed-point to avoid float: value * 1e6
    int32_t  lat_1e6;         // latitude  * 1e6 (degrees)
    int32_t  lon_1e6;         // longitude * 1e6 (degrees)
    int32_t  alt_cm;          // altitude in centimeters
    int16_t  vH_cm;           // horizontal speed cm/s
    int16_t  vV_cm;           // vertical speed cm/s
    uint16_t heading;         // degrees * 10
    uint8_t  numSV;           // number of satellites

    // CRC validation
    uint8_t  crc_ok;          // bitmask of validated blocks

    // Decode status
    bool     valid;           // at least one block decoded successfully
} RS41_Data_t;

// Internal decoder context
typedef struct {
    RS41_State_t state;

    // Bit-level shift register for sync detection
    uint64_t shift_reg;
    uint16_t bit_count;

    // Frame buffer
    uint8_t  frame[RS41_FRAME_LEN];
    uint16_t frame_byte_idx;
    uint8_t  frame_bit_idx;
    bool     inverted;

    // Output
    RS41_Data_t data;

    uint32_t frames_received;
    uint32_t frames_crc_ok;

    uint8_t min_errors;
    uint8_t last_errors;
    uint8_t last_errors_inv;
    uint32_t last_shift_hi;  // upper 32 bits of shift_reg at detection
    uint32_t last_shift_lo;  // lower 32 bits
    uint8_t  diag_status_pos;
    uint8_t  diag_status_len;
    uint8_t  diag_status_id;
    uint16_t diag_status_crc_calc;
    uint16_t diag_status_crc_frame;
    int8_t   diag_best_shift;
    bool     diag_best_invert;
    uint16_t diag_best_pos;
} RS41_Decoder_t;

//
// Public API
//

// Initialize the decoder
void RS41_Init(RS41_Decoder_t *dec);

// Feed one demodulated bit (0 or 1) into the decoder
// Returns true when a complete frame has been decoded
bool RS41_ProcessBit(RS41_Decoder_t *dec, uint8_t bit);

// Get pointer to last decoded data (valid after RS41_ProcessBit returns true)
const RS41_Data_t* RS41_GetData(const RS41_Decoder_t *dec);

// Reset decoder to initial state
void RS41_Reset(RS41_Decoder_t *dec);

#endif // APP_RS41_H
