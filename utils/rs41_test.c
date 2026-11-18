// Host-side verification of the real RS41 decoder in app/rs41.c.
// Unlike aprs_hdlc_test.c (which has to copy the firmware's logic because
// aprs_minimal.c pulls in BK4819/GPIO headers), rs41.c has zero MCU
// dependencies, so this links the actual decoder unmodified. Builds a
// byte-correct RS41 frame (real sync header, STATUS/GPS_TIME/GPS_POS blocks
// with correct CRC16 and XOR scrambling - the same framing a real RS41
// transmits), feeds it bit-by-bit into RS41_ProcessBit() exactly as the
// live RX path would, and checks the decoded fields round-trip.
//
// Build/run:
//   cc -Wall -o /tmp/rs41_test utils/rs41_test.c app/rs41.c -lm && /tmp/rs41_test
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../app/rs41.h"

// ---- copied from app/rs41.c (both are `static` there, not exported) ----
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
static const uint8_t rs41_header[RS41_HEADER_LEN] = {
    0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8
};

static uint16_t rs41_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put_u16_le(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void put_u32_le(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void put_i16_le(uint8_t *p, int16_t v)  { put_u16_le(p, (uint16_t)v); }
static void put_i32_le(uint8_t *p, int32_t v)  { put_u32_le(p, (uint32_t)v); }

// Writes a block [ID][LEN][DATA(len)][CRC16 LE] at *pos, advances *pos.
// Returns a pointer to the data region so the caller can fill it in.
static uint8_t *write_block_header(uint8_t *frame, uint16_t *pos, uint8_t id, uint8_t len)
{
    frame[*pos] = id;
    frame[*pos + 1] = len;
    uint8_t *data = &frame[*pos + 2];
    *pos += 2 + len + 2; // ID + LEN + DATA + CRC, caller fills DATA then we CRC it below
    return data;
}

static void finalize_block_crc(uint8_t *data_start, uint8_t len)
{
    uint16_t crc = rs41_crc16(data_start, len);
    put_u16_le(data_start + len, crc);
}

// Standard WGS84 LLA -> ECEF, double precision (host-only - the firmware's
// own rs41_ecef_to_lla goes the other way, in fixed-point/CORDIC, since it
// can't afford libm on the Cortex-M0 target; here we're the "transmitter"
// simulating what a real sonde's GPS would report, so plain doubles are fine).
static void lla_to_ecef(double lat_deg, double lon_deg, double alt_m,
                         int32_t *x_cm, int32_t *y_cm, int32_t *z_cm)
{
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e2 = f * (2.0 - f);
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
    double sin_lat = sin(lat), cos_lat = cos(lat);
    double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
    double x = (N + alt_m) * cos_lat * cos(lon);
    double y = (N + alt_m) * cos_lat * sin(lon);
    double z = (N * (1.0 - e2) + alt_m) * sin_lat;
    *x_cm = (int32_t)lround(x * 100.0);
    *y_cm = (int32_t)lround(y * 100.0);
    *z_cm = (int32_t)lround(z * 100.0);
}

// Builds a complete, correctly-scrambled 320-byte "on air" RS41 frame for
// the given test telemetry. Mirrors exactly what a real RS41 transmits:
// unscrambled sync header, then XOR-scrambled blocks.
static void build_test_frame(uint8_t onair[RS41_FRAME_LEN],
                              const char *sonde_id, uint16_t frame_nr, uint8_t batt_decivolt_x10,
                              double lat_deg, double lon_deg, double alt_m,
                              int16_t vel_x_cms, int16_t vel_y_cms, int16_t vel_z_cms, uint8_t numSV,
                              uint16_t gps_week, uint32_t gps_tow_ms)
{
    uint8_t plain[RS41_FRAME_LEN];
    memset(plain, 0, sizeof(plain));
    memcpy(plain, rs41_header, RS41_HEADER_LEN);

    uint16_t pos = 0x39; // where rs41_find_block's scan range starts

    // STATUS block (0x79, len 0x28): frame_nr, sonde_id, battery.
    uint8_t *status_data = write_block_header(plain, &pos, RS41_BLK_STATUS, 0x28);
    put_u16_le(status_data + 0, frame_nr);          // pos+2  (rs41_parse_status)
    memcpy(status_data + 2, sonde_id, 8);            // pos+4
    status_data[10] = batt_decivolt_x10;             // pos+12 (* 100 -> mV)
    finalize_block_crc(status_data, 0x28);

    // GPS_INFO block (0x7C, len 0x2A): GPS week + time-of-week.
    uint8_t *gps1_data = write_block_header(plain, &pos, RS41_BLK_GPS_INFO, 0x2A);
    put_u16_le(gps1_data + 0, gps_week);             // pos+2
    put_u32_le(gps1_data + 2, gps_tow_ms);            // pos+4
    finalize_block_crc(gps1_data, 0x2A);

    // GPS_POS block (0x7B, len 0x15): ECEF position + velocity + sat count.
    int32_t ecef_x, ecef_y, ecef_z;
    lla_to_ecef(lat_deg, lon_deg, alt_m, &ecef_x, &ecef_y, &ecef_z);
    uint8_t *gps3_data = write_block_header(plain, &pos, RS41_BLK_GPS_POS, 0x15);
    put_i32_le(gps3_data + 0, ecef_x);               // pos+2
    put_i32_le(gps3_data + 4, ecef_y);               // pos+6
    put_i32_le(gps3_data + 8, ecef_z);               // pos+10
    put_i16_le(gps3_data + 12, vel_x_cms);            // pos+14
    put_i16_le(gps3_data + 14, vel_y_cms);            // pos+16
    put_i16_le(gps3_data + 16, vel_z_cms);            // pos+18
    gps3_data[18] = numSV;                            // pos+20
    finalize_block_crc(gps3_data, 0x15);

    // Scramble: sync header is transmitted raw/unscrambled (real RS41
    // protocol), everything from byte 8 on is XORed with the running mask -
    // exactly what rs41_descramble() on the RX side undoes.
    memcpy(onair, plain, RS41_HEADER_LEN);
    for (uint16_t i = RS41_HEADER_LEN; i < RS41_FRAME_LEN; i++)
        onair[i] = plain[i] ^ rs41_xor_mask[i % RS41_XOR_MASK_LEN];
}

static int failures = 0;
#define CHECK(cond, fmt, ...) do { \
    if (!(cond)) { failures++; printf("  FAIL: " fmt "\n", ##__VA_ARGS__); } \
    else { printf("  ok:   " fmt "\n", ##__VA_ARGS__); } \
} while (0)

int main(void)
{
    const char *sonde_id = "TEST0001";
    const double lat = 45.0, lon = -93.0, alt = 10000.0;
    const int16_t vel_x = 100, vel_y = 150, vel_z = 500; // cm/s
    const uint8_t numSV = 9, batt_byte = 33;             // -> 3300 mV
    const uint16_t frame_nr = 42, gps_week = 2300;
    const uint32_t gps_tow_ms = 45296000; // -> 12:34:56

    uint8_t onair[RS41_FRAME_LEN];
    build_test_frame(onair, sonde_id, frame_nr, batt_byte, lat, lon, alt,
                      vel_x, vel_y, vel_z, numSV, gps_week, gps_tow_ms);

    RS41_Decoder_t dec;
    RS41_Init(&dec);

    bool frame_decoded = false;
    int total_bits = RS41_FRAME_LEN * 8;
    for (int i = 0; i < total_bits; i++) {
        uint8_t byte = onair[i / 8];
        uint8_t bit = (byte >> (i % 8)) & 1; // RS41 transmits LSBit first
        if (RS41_ProcessBit(&dec, bit)) {
            CHECK(i == total_bits - 1, "frame_decoded fires on the last bit (fired at bit %d/%d)", i, total_bits);
            frame_decoded = true;
        }
    }

    printf("\n--- frame-level ---\n");
    CHECK(frame_decoded, "RS41_ProcessBit returned true");
    CHECK(dec.frames_received == 1, "frames_received == 1 (got %d)", (int)dec.frames_received);
    CHECK(dec.frames_crc_ok == 1, "frames_crc_ok == 1 (got %d)", (int)dec.frames_crc_ok);

    const RS41_Data_t *d = RS41_GetData(&dec);
    printf("\n--- decoded fields vs. what we encoded ---\n");
    CHECK(d->valid, "data.valid == true");
    CHECK(d->crc_ok & RS41_CRC_STATUS, "STATUS block CRC passed");
    CHECK(d->crc_ok & RS41_CRC_GPS1, "GPS_INFO block CRC passed");
    CHECK(d->crc_ok & RS41_CRC_GPS3, "GPS_POS block CRC passed");
    CHECK(strcmp(d->sonde_id, sonde_id) == 0, "sonde_id == \"%s\" (got \"%s\")", sonde_id, d->sonde_id);
    CHECK(d->frame_nr == frame_nr, "frame_nr == %u (got %u)", frame_nr, d->frame_nr);
    CHECK(d->batt_mv == (uint16_t)batt_byte * 100, "batt_mv == %u (got %u)", (unsigned)batt_byte * 100, d->batt_mv);
    CHECK(d->numSV == numSV, "numSV == %u (got %u)", numSV, d->numSV);
    CHECK(d->gps_week == gps_week, "gps_week == %u (got %u)", gps_week, d->gps_week);
    CHECK(d->gps_tow_ms == gps_tow_ms, "gps_tow_ms == %u (got %u)", gps_tow_ms, d->gps_tow_ms);
    CHECK(d->gps_hour == 12 && d->gps_min == 34 && d->gps_sec == 56,
          "gps time == 12:34:56 (got %02u:%02u:%02u)", d->gps_hour, d->gps_min, d->gps_sec);

    // lat/lon/alt go through the firmware's own fixed-point/CORDIC ECEF->LLA
    // (rs41_ecef_to_lla) on the way back out, which the code's own comments
    // document as sub-meter/sub-10cm accurate vs. the float math it replaced -
    // so this tolerance is checking that conversion, not just field plumbing.
    double got_lat = d->lat_1e6 / 1e6, got_lon = d->lon_1e6 / 1e6, got_alt = d->alt_cm / 100.0;
    CHECK(fabs(got_lat - lat) < 0.0001, "lat ~= %.4f (got %.6f, diff %.6f deg)", lat, got_lat, got_lat - lat);
    CHECK(fabs(got_lon - lon) < 0.0001, "lon ~= %.4f (got %.6f, diff %.6f deg)", lon, got_lon, got_lon - lon);
    CHECK(fabs(got_alt - alt) < 1.0, "alt ~= %.2fm (got %.2fm, diff %.2fm)", alt, got_alt, got_alt - alt);

    double expect_vH = sqrt((double)vel_x * vel_x + (double)vel_y * vel_y);
    CHECK(fabs((double)d->vH_cm - expect_vH) <= 1.0, "vH_cm ~= %.0f (got %d)", expect_vH, d->vH_cm);
    CHECK(d->vV_cm == vel_z, "vV_cm == %d (got %d)", vel_z, d->vV_cm);

    printf("\n%s (%d check%s failed)\n", failures == 0 ? "PASS" : "FAIL",
           failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
