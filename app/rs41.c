/* RS41 Radiosonde Decoder - Core Implementation
 * Ported from rs1729/RS (rs41mod.c) - GPL License
 * Stripped down for ARM Cortex-M0 (DP32G030)
 *
 * Changes from original:
 *   - No Reed-Solomon ECC (saves ~3KB flash)
 *   - Integer math for ECEF->LLA conversion (no double/float)
 *   - No stdio/printf/file I/O
 *   - Minimal RAM footprint (~500 bytes total)
 */

#include "rs41.h"
#include <string.h>

// ============================================================
// RS41 XOR Mask (64 bytes) — LFSR-generated pseudorandom sequence
// Used for data whitening/descrambling
// ============================================================
static const uint8_t rs41_xor_mask[RS41_XOR_MASK_LEN] = {
    0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98,
    0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
    0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
    0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
    0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
    0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
    0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
    0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1
};

// ============================================================
// RS41 Sync Header (8 bytes after XOR descramble)
// Raw bits (before descramble): 0x10B6CA11229612F8
// After XOR with mask[0..7]:    0x8635F44093DF1A60
// ============================================================
static const uint8_t rs41_header[RS41_HEADER_LEN] = {
    0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8
};

// 64-bit representation for fast correlation matching (shifted left, MSB is first bit received)
static const uint64_t rs41_header_64 = 0x086D53884469481FULL;

// ============================================================
// Block position offsets within the frame (after header + ECC)
// Header: [0x00-0x07]  ECC: [0x08-0x37]  FrameType: [0x38]
// Blocks start at 0x39
// ============================================================

// STATUS block (0x79) — typical position
#define POS_FRAME_NR     0x3B   // 2 bytes LE: frame number
#define POS_SONDE_ID     0x3D   // 8 bytes: sonde serial
#define POS_BATT         0x45   // 2 bytes LE: battery (mV)

// GPS1 block (0x7C) — GPS week + time
#define POS_GPS_WEEK     0x95   // 2 bytes LE
#define POS_GPS_TOW      0x97   // 4 bytes LE (ms)

// GPS3 block (0x7B) — ECEF position
#define POS_GPS_ECEF_X   0x72   // 4 bytes LE (cm)
#define POS_GPS_ECEF_Y   0x76   // 4 bytes LE (cm)
#define POS_GPS_ECEF_Z   0x7A   // 4 bytes LE (cm)
#define POS_GPS_VEL_X    0x7E   // 2 bytes LE (cm/s)
#define POS_GPS_VEL_Y    0x80   // 2 bytes LE (cm/s)
#define POS_GPS_VEL_Z    0x82   // 2 bytes LE (cm/s)
#define POS_GPS_NUMSV    0x84   // 1 byte


// ============================================================
// CRC-16 CCITT (polynomial 0x1021)
// Used per-block in RS41 frames
// ============================================================
static uint16_t rs41_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ============================================================
// Check CRC for a block at given position
// Block structure: [ID][LEN][DATA...][CRC_LO][CRC_HI]
// Returns true if CRC matches
// ============================================================
static bool rs41_check_block_crc(const uint8_t *frame, uint16_t pos)
{
    if (pos + 2 >= RS41_FRAME_LEN)
        return false;

    uint8_t blk_id  = frame[pos];
    uint8_t blk_len = frame[pos + 1];

    (void)blk_id;

    // Bounds check
    if (pos + 2 + blk_len + 2 > RS41_FRAME_LEN)
        return false;

    // CRC is computed over block data (not including ID and LEN bytes)
    uint16_t crc_calc = rs41_crc16(&frame[pos + 2], blk_len);

    // CRC stored little-endian after data
    uint16_t crc_frame = (uint16_t)frame[pos + 2 + blk_len]
                       | ((uint16_t)frame[pos + 2 + blk_len + 1] << 8);

    return (crc_calc == crc_frame);
}

// ============================================================
// XOR Descramble the entire frame
// ============================================================
static void rs41_descramble(uint8_t *frame, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        frame[i] ^= rs41_xor_mask[i % RS41_XOR_MASK_LEN];
    }
}

// ============================================================
// Read 16-bit little-endian value from frame
// ============================================================
static inline uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// ============================================================
// Read 32-bit little-endian value from frame
// ============================================================
static inline uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

// ============================================================
// Read signed 16-bit little-endian value from frame
// ============================================================
static inline int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)read_u16_le(p);
}

// ============================================================
// Read signed 32-bit little-endian value from frame
// ============================================================
static inline int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)read_u32_le(p);
}

// ECEF->LLA (Bowring's method) in pure integer/fixed-point math - no libm.
// This firmware links no floating point anywhere else, so pulling in sqrtf/
// atan2f here alone drags in the whole Cortex-M0 soft-float runtime for one
// function; isqrt64 + a CORDIC atan2 avoid that entirely. Verified on the
// host against the float version this replaced, across equator/mid/high
// latitude, both hemispheres and 0-50km altitude: sub-meter lat/lon error,
// sub-10cm altitude error - comfortably inside APRS's own ~18m wire-format
// resolution. (Degrades to ~1-2m / ~200m alt within 0.001 degree of a pole,
// a case radiosondes don't fly.)

// Exact integer square root (digit-by-digit), any 64-bit input.
static uint32_t isqrt64(uint64_t v)
{
    uint64_t res = 0;
    uint64_t bit = 1ULL << 62;
    while (bit > v) bit >>= 2;
    while (bit != 0) {
        if (v >= res + bit) {
            v -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)res;
}

#define FXQ          30 // Q-format bits for sin/cos ratios (|value| <= 1)
#define ANGLE_Q      28 // Q-format bits for CORDIC angles (radians)
// 20 iterations (not the textbook 28): verified on the host against the
// float version this replaced, worst case ~11.6m of lat/lon error across the
// same latitude/altitude sweep used to validate the full table - still under
// APRS's own ~18m wire-format resolution, though with less margin than 22+
// iterations gave (~3.3m there). Traded down from 22 for the flash this
// build's margin needs (see the ENABLE_RS41 Makefile comment) - go back up
// first if this ever gets another few bytes to spare.
#define CORDIC_ITERS 20

// atan(2^-i) for i=0..19, in Q28 radians (generated with atan()/double on a
// host, not computed on-device).
static const int32_t atan_table_q28[CORDIC_ITERS] = {
    210828714, 124459457, 65760959, 33381290, 16755422, 8385879, 4193963,
    2097109, 1048571, 524287, 262144, 131072, 65536, 32768, 16384, 8192,
    4096, 2048, 1024, 512
};
static const int32_t ANGLE_PI_Q28 = 843314857; // round(pi * 2^28)

static int32_t cordic_core(int64_t y, int64_t x)
{
    int64_t cx = x, cy = y;
    int32_t angle = 0;
    for (int i = 0; i < CORDIC_ITERS; i++) {
        if (cy > 0) {
            int64_t nx = cx + (cy >> i);
            int64_t ny = cy - (cx >> i);
            cx = nx; cy = ny;
            angle += atan_table_q28[i];
        } else if (cy < 0) {
            int64_t nx = cx - (cy >> i);
            int64_t ny = cy + (cx >> i);
            cx = nx; cy = ny;
            angle -= atan_table_q28[i];
        } else {
            break;
        }
    }
    return angle;
}

// atan2(y,x) -> Q28 radians, full range (-pi, pi].
static int32_t fx_atan2(int64_t y, int64_t x)
{
    if (x == 0 && y == 0) return 0;
    if (x < 0) {
        int32_t base = cordic_core(-y, -x);
        return base + (y >= 0 ? ANGLE_PI_Q28 : -ANGLE_PI_Q28);
    }
    return cordic_core(y, x);
}

// Q28 radians -> micro-degrees.
static int32_t angle_q28_to_udeg(int32_t aq)
{
    return (int32_t)(((int64_t)aq * 57295780LL) >> 28);
}

// (vy,vx) -> (sin,cos) as Q30 ratios, without ever squaring the raw
// (potentially huge) inputs: scale both down by the same shift first.
static void fx_normalize(int64_t vy, int64_t vx, int32_t *sin_q, int32_t *cos_q)
{
    int64_t ax = vx < 0 ? -vx : vx;
    int64_t ay = vy < 0 ? -vy : vy;
    int64_t m = ax > ay ? ax : ay;
    int shift = 0;
    while (m >= (1LL << 30)) { m >>= 1; shift++; }
    int64_t vxs = vx >> shift;
    int64_t vys = vy >> shift;
    uint64_t hyp = isqrt64((uint64_t)(vxs*vxs + vys*vys));
    if (hyp == 0) { *sin_q = 0; *cos_q = 0; return; }
    *sin_q = (int32_t)(((int64_t)vys << FXQ) / (int64_t)hyp);
    *cos_q = (int32_t)(((int64_t)vxs << FXQ) / (int64_t)hyp);
}

static void rs41_ecef_to_lla(int32_t ecef_x_cm, int32_t ecef_y_cm, int32_t ecef_z_cm,
                              int32_t *lat_1e6, int32_t *lon_1e6, int32_t *alt_cm)
{
    int64_t x = ecef_x_cm, y = ecef_y_cm, z = ecef_z_cm; // cm

    const int64_t a       = 637813700LL; // 6378137.0 m, in cm
    const int64_t b       = 635675231LL; // 6356752.3142 m, in cm (rounded)
    const int32_t e2_q30  = 7188036;     // round(0.00669437999014 * 2^30)
    const int32_t ep2_q30 = 7236480;     // round(0.00673949674228 * 2^30)

    int64_t p = (int64_t)isqrt64((uint64_t)(x*x + y*y)); // cm

    int64_t th_y = a * z; // cm^2-scale, magnitude ~4e17 - fine for int64
    int64_t th_x = b * p;

    int32_t sin_th, cos_th;
    fx_normalize(th_y, th_x, &sin_th, &cos_th);

    int64_t sin_th3 = ((int64_t)sin_th * sin_th) >> FXQ;
    sin_th3 = (sin_th3 * sin_th) >> FXQ;
    int64_t cos_th3 = ((int64_t)cos_th * cos_th) >> FXQ;
    cos_th3 = (cos_th3 * cos_th) >> FXQ;

    // ep2*b*sin_th^3 and e2*a*cos_th^3, both in cm, staged to avoid overflow.
    int64_t term_y = (((int64_t)ep2_q30 * sin_th3) >> FXQ) * b >> FXQ;
    int64_t term_x = (((int64_t)e2_q30  * cos_th3) >> FXQ) * a >> FXQ;

    int64_t lat_y = z + term_y;
    int64_t lat_x = p - term_x;

    int32_t lat_q28 = fx_atan2(lat_y, lat_x);

    int32_t sin_lat, cos_lat;
    fx_normalize(lat_y, lat_x, &sin_lat, &cos_lat);

    int64_t sin_lat2_q30 = ((int64_t)sin_lat * sin_lat) >> FXQ;
    int64_t e2sl2_q30 = ((int64_t)e2_q30 * sin_lat2_q30) >> FXQ;
    int64_t oneminus_q30 = (1LL << FXQ) - e2sl2_q30;

    // sqrt(1-e2*sin^2(lat)) is within ~0.3% of 1.0, so a Q15 result (the
    // "natural" isqrt64-of-a-Q30-value precision) loses too many bits here -
    // its ~3e-5 relative resolution alone is worth ~190m of error in N once
    // multiplied back out by `a`. Go to Q30 instead (isqrt64 of a Q60 value).
    int64_t sqrt_q30 = (int64_t)isqrt64((uint64_t)oneminus_q30 << FXQ);

    int64_t N_cm = (sqrt_q30 != 0) ? ((a << FXQ) / sqrt_q30) : a;

    int64_t alt_cm_v;
    if (cos_lat != 0) {
        alt_cm_v = ((p << FXQ) / cos_lat) - N_cm;
    } else {
        alt_cm_v = p - N_cm; // degenerate (pole); negligible for our use case
    }

    int32_t lon_q28 = fx_atan2(y, x);

    *lon_1e6 = angle_q28_to_udeg(lon_q28);
    *lat_1e6 = angle_q28_to_udeg(lat_q28);
    *alt_cm  = (int32_t)alt_cm_v;
}

// ============================================================
// Find a block by ID within the frame
// Returns position of block start, or 0 if not found
// ============================================================
static uint16_t rs41_find_block(const uint8_t *frame, uint8_t blk_id)
{
    // GPS3 block is 0x7B, length 0x15
    // PTU block is 0x7A, length 0x21
    // STATUS block is 0x79, length 0x28
    // GPS INFO block is 0x7C, length 0x2A or 0x1E

    uint8_t expected_len = 0;
    if (blk_id == RS41_BLK_STATUS) expected_len = 0x28;
    else if (blk_id == RS41_BLK_MEAS) expected_len = 0x21;
    else if (blk_id == RS41_BLK_GPS_INFO) expected_len = 0x2A;
    else if (blk_id == RS41_BLK_GPS_POS) expected_len = 0x15;

    // Scan byte-by-byte starting after header (0x39)
    for (uint16_t pos = 0x39; pos < RS41_FRAME_LEN - 4; pos++) {
        if (frame[pos] == blk_id) {
            uint8_t len = frame[pos + 1];
            if (blk_id == RS41_BLK_GPS_INFO) {
                if (len == 0x2A || len == 0x1E) {
                    return pos;
                }
            } else {
                if (len == expected_len) {
                    return pos;
                }
            }
        }
    }
    return 0; // not found
}

// ============================================================
// Parse STATUS block (0x79)
// ============================================================
static void rs41_parse_status(RS41_Decoder_t *dec)
{
    uint16_t pos = rs41_find_block(dec->frame, RS41_BLK_STATUS);
    dec->diag_status_pos = pos;
    if (pos == 0) {
        dec->diag_status_id = dec->frame[0x39];
        dec->diag_status_len = dec->frame[0x3A];
        dec->diag_status_crc_calc = 0;
        dec->diag_status_crc_frame = 0;
        return;
    }

    dec->diag_status_id = dec->frame[pos];
    dec->diag_status_len = dec->frame[pos + 1];

    uint16_t crc_calc = rs41_crc16(&dec->frame[pos + 2], dec->diag_status_len);
    uint16_t crc_frame = (uint16_t)dec->frame[pos + 2 + dec->diag_status_len]
                       | ((uint16_t)dec->frame[pos + 2 + dec->diag_status_len + 1] << 8);

    dec->diag_status_crc_calc = crc_calc;
    dec->diag_status_crc_frame = crc_frame;

    if (crc_calc != crc_frame)
        return;

    dec->data.crc_ok |= RS41_CRC_STATUS;

    // Frame number: 2 bytes at offset +2 from block start (after ID+LEN)
    dec->data.frame_nr = read_u16_le(&dec->frame[pos + 2]);

    // Sonde ID: 8 bytes ASCII at offset +4
    char old_id[10];
    strcpy(old_id, dec->data.sonde_id);

    memcpy(dec->data.sonde_id, &dec->frame[pos + 4], 8);
    dec->data.sonde_id[8] = '\0';

    // If sonde ID changed from a previous real sonde (not "LAST GPS"),
    // clear the GPS coordinates.
    if (old_id[0] != '\0' && old_id[0] != 'L' && memcmp(old_id, dec->data.sonde_id, 8) != 0) {
        dec->data.lat_1e6 = 0;
        dec->data.lon_1e6 = 0;
        dec->data.alt_cm = 0;
    }

    // Battery voltage: 1 byte at offset +12 (in tenths of a volt)
    if (pos + 12 < RS41_FRAME_LEN) {
        dec->data.batt_mv = (uint16_t)dec->frame[pos + 12] * 100;
    }
}

// ============================================================
// Parse GPS TIME block (0x7C / GPS1)
// ============================================================
static void rs41_parse_gps_time(RS41_Decoder_t *dec)
{
    uint16_t pos = rs41_find_block(dec->frame, RS41_BLK_GPS_INFO);
    if (pos == 0) return;

    if (!rs41_check_block_crc(dec->frame, pos))
        return;

    dec->data.crc_ok |= RS41_CRC_GPS1;

    // GPS week: 2 bytes LE at data offset 0
    dec->data.gps_week = read_u16_le(&dec->frame[pos + 2]);

    // GPS TOW: 4 bytes LE at data offset 2 (milliseconds)
    dec->data.gps_tow_ms = read_u32_le(&dec->frame[pos + 4]);

    // Convert TOW to H:M:S
    uint32_t tow_sec = dec->data.gps_tow_ms / 1000;
    uint32_t time_of_day = tow_sec % (24 * 3600);
    dec->data.gps_hour = time_of_day / 3600;
    dec->data.gps_min  = (time_of_day % 3600) / 60;
    dec->data.gps_sec  = time_of_day % 60;
}

// ============================================================
// Parse GPS POSITION block (0x7B / GPS3)
// ============================================================
static void rs41_parse_gps_pos(RS41_Decoder_t *dec)
{
    uint16_t pos = rs41_find_block(dec->frame, RS41_BLK_GPS_POS);
    if (pos == 0) return;

    if (!rs41_check_block_crc(dec->frame, pos))
        return;

    dec->data.crc_ok |= RS41_CRC_GPS3;

    // ECEF X, Y, Z in centimeters (4 bytes each, signed LE)
    int32_t ecef_x = read_i32_le(&dec->frame[pos + 2]);
    int32_t ecef_y = read_i32_le(&dec->frame[pos + 6]);
    int32_t ecef_z = read_i32_le(&dec->frame[pos + 10]);

    // Velocity X, Y, Z in cm/s (2 bytes each, signed LE)
    int16_t vel_x = read_i16_le(&dec->frame[pos + 14]);
    int16_t vel_y = read_i16_le(&dec->frame[pos + 16]);
    int16_t vel_z = read_i16_le(&dec->frame[pos + 18]);

    // Number of satellites
    if (pos + 22 < RS41_FRAME_LEN)
        dec->data.numSV = dec->frame[pos + 20];

    // Convert ECEF to LLA (integer math)
    rs41_ecef_to_lla(ecef_x, ecef_y, ecef_z,
                     &dec->data.lat_1e6,
                     &dec->data.lon_1e6,
                     &dec->data.alt_cm);

    // Compute horizontal speed: sqrt(vx^2 + vy^2), in cm/s
    dec->data.vH_cm = (int16_t)isqrt64((uint64_t)((int64_t)vel_x*vel_x + (int64_t)vel_y*vel_y));

    // Vertical speed
    dec->data.vV_cm = vel_z;

    // Heading: atan2(vy, vx) in degrees * 10
    if (dec->data.vH_cm > 0) {
        int32_t heading_x10 = angle_q28_to_udeg(fx_atan2(vel_y, vel_x)) / 100000;
        if (heading_x10 < 0) heading_x10 += 3600;
        dec->data.heading = (uint16_t)heading_x10;
    }
}

static void shift_frame_bits(const uint8_t *in, uint8_t *out, int shift, bool invert)
{
    for (int i = 0; i < RS41_FRAME_LEN; i++) {
        uint16_t val = in[i];
        if (shift > 0) {
            uint16_t next = (i + 1 < RS41_FRAME_LEN) ? in[i + 1] : 0;
            val = (val >> shift) | (next << (8 - shift));
        }
        uint8_t res = (uint8_t)val;
        if (invert) {
            res = ~res;
        }
        out[i] = res;
    }
}

// ============================================================
// Parse the complete frame
// ============================================================
static bool rs41_parse_frame(RS41_Decoder_t *dec)
{
    dec->diag_best_shift = -1;
    dec->diag_best_invert = false;
    dec->diag_best_pos = 0;

    for (int inv = 0; inv < 2; inv++) {
        for (int sh = 0; sh < 8; sh++) {
            uint8_t temp_frame[RS41_FRAME_LEN];
            shift_frame_bits(dec->frame, temp_frame, sh, inv == 1);
            rs41_descramble(temp_frame, RS41_FRAME_LEN);

            // Search for status block 0x79 with length 0x28
            for (uint16_t p = 0x30; p < 0x45; p++) {
                if (temp_frame[p] == 0x79 && temp_frame[p + 1] == 0x28) {
                    dec->diag_best_shift = sh;
                    dec->diag_best_invert = (inv == 1);
                    dec->diag_best_pos = p;
                    // Copy the corrected, descrambled frame to dec->frame
                    memcpy(dec->frame, temp_frame, RS41_FRAME_LEN);
                    break;
                }
            }
            if (dec->diag_best_shift != -1) break;
        }
        if (dec->diag_best_shift != -1) break;
    }

    // Reset crc_ok for the current frame, but keep old data
    dec->data.crc_ok = 0;

    // Descramble the frame only if the scanner didn't already do it
    if (dec->diag_best_shift == -1) {
        rs41_descramble(dec->frame, RS41_FRAME_LEN);
    }

    // (Header verification skipped as it was matched by correlation)

    // Parse individual blocks
    rs41_parse_status(dec);
    rs41_parse_gps_time(dec);
    rs41_parse_gps_pos(dec);

    // Mark as valid if at least STATUS was decoded
    if (dec->data.crc_ok & RS41_CRC_STATUS) {
        dec->data.valid = true;
        dec->frames_crc_ok++;
        return true;
    }
    return false;
}

// ============================================================
// PUBLIC API
// ============================================================

void RS41_Init(RS41_Decoder_t *dec)
{
    memset(dec, 0, sizeof(RS41_Decoder_t));
    dec->bit_count = 0;
    dec->state = RS41_STATE_WAIT_PREAMBLE;
    dec->min_errors = 64; // Initialize to max possible errors
}

void RS41_Reset(RS41_Decoder_t *dec)
{
    RS41_Init(dec);
}

bool RS41_ProcessBit(RS41_Decoder_t *dec, uint8_t bit)
{
    bit &= 1;

    switch (dec->state) {
        case RS41_STATE_WAIT_PREAMBLE:
        case RS41_STATE_WAIT_HEADER:
        {
            // Shift bit into 64-bit register (MSB first)
            dec->shift_reg = (dec->shift_reg << 1) | bit;
            dec->bit_count++;

            if (dec->bit_count < RS41_SYNC_BITS)
                break;

            // Correlate with known header pattern
            uint64_t xor_val = dec->shift_reg ^ rs41_header_64;
            uint64_t xor_inv = dec->shift_reg ^ ~rs41_header_64;

            uint32_t hi = (uint32_t)(xor_val >> 32);
            uint32_t lo = (uint32_t)(xor_val & 0xFFFFFFFF);
            uint8_t errors = (uint8_t)(__builtin_popcount(hi) + __builtin_popcount(lo));

            uint32_t ihi = (uint32_t)(xor_inv >> 32);
            uint32_t ilo = (uint32_t)(xor_inv & 0xFFFFFFFF);
            uint8_t errors_inv = (uint8_t)(__builtin_popcount(ihi) + __builtin_popcount(ilo));

            if (errors < dec->min_errors) {
                dec->min_errors = errors;
                dec->last_shift_hi = (uint32_t)(dec->shift_reg >> 32);
                dec->last_shift_lo = (uint32_t)(dec->shift_reg & 0xFFFFFFFF);
            }
            if (errors_inv < dec->min_errors) {
                dec->min_errors = errors_inv;
                dec->last_shift_hi = (uint32_t)(dec->shift_reg >> 32);
                dec->last_shift_lo = (uint32_t)(dec->shift_reg & 0xFFFFFFFF);
            }

            // Save last error counts for display
            dec->last_errors = errors;
            dec->last_errors_inv = errors_inv;

            if (errors <= 12 || errors_inv <= 12) {
                // Header found! Start collecting frame
                dec->state = RS41_STATE_COLLECT_FRAME;
                dec->inverted = (errors_inv < errors);

                // Copy the header into frame buffer
                // We use the known good header rather than the received one
                for (int i = 0; i < RS41_HEADER_LEN; i++) {
                    dec->frame[i] = rs41_header[i];
                }

                dec->frame_byte_idx = RS41_HEADER_LEN;
                dec->frame_bit_idx = 0;
                dec->bit_count = 0;
            }
            break;
        }

        case RS41_STATE_COLLECT_FRAME:
        {
            if (dec->inverted) bit ^= 1;

            // Accumulate bits into frame bytes (RS41 transmits LSBit first)
            if (dec->frame_byte_idx < RS41_FRAME_LEN) {
                dec->frame[dec->frame_byte_idx] = (dec->frame[dec->frame_byte_idx] >> 1) | (bit << 7);
                dec->frame_bit_idx++;

                if (dec->frame_bit_idx >= 8) {
                    dec->frame_bit_idx = 0;
                    dec->frame_byte_idx++;
                }
            }

            // Check if frame is complete
            if (dec->frame_byte_idx >= RS41_FRAME_LEN) {
                dec->frames_received++;

                // Parse the frame
                bool is_valid = rs41_parse_frame(dec);

                // Reset for next frame
                dec->state = RS41_STATE_WAIT_HEADER;
                dec->shift_reg = 0;
                dec->bit_count = 0;
                dec->frame_byte_idx = 0;
                dec->frame_bit_idx = 0;

                return is_valid;
            }
            break;
        }

        case RS41_STATE_FRAME_READY:
            // Should not reach here, reset
            dec->state = RS41_STATE_WAIT_HEADER;
            break;
    }

    return false;
}

const RS41_Data_t* RS41_GetData(const RS41_Decoder_t *dec)
{
    return &dec->data;
}

