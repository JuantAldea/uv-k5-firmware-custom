#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

static uint8_t gf_mul(uint8_t x, uint8_t y) {
    uint8_t z = 0;
    for (int i = 7; i >= 0; i--) {
        z = (z << 1) ^ ((z & 0x80) ? 0x11D : 0);
        if (y & (1 << i)) z ^= x;
    }
    return z;
}

void miniqr_encode(const char *text, uint8_t qrcode[25][25]) {
    uint8_t data[44] = {0};
    int len = strlen(text);
    if (len > 32) len = 32;

    int bit_pos = 0;
    for (int i = 3; i >= 0; i--) {
        if ((4 >> i) & 1) data[bit_pos / 8] |= (1 << (7 - (bit_pos % 8)));
        bit_pos++;
    }
    for (int i = 7; i >= 0; i--) {
        if ((len >> i) & 1) data[bit_pos / 8] |= (1 << (7 - (bit_pos % 8)));
        bit_pos++;
    }
    for (int i = 0; i < len; i++) {
        for (int j = 7; j >= 0; j--) {
            if ((text[i] >> j) & 1) data[bit_pos / 8] |= (1 << (7 - (bit_pos % 8)));
            bit_pos++;
        }
    }
    bit_pos += 4;

    int pad[] = {0xEC, 0x11};
    int pad_idx = 0;
    while ((bit_pos + 7) / 8 <= 34) {
        if (bit_pos % 8 == 0 && bit_pos / 8 < 34) {
            data[bit_pos / 8] = pad[pad_idx];
            pad_idx ^= 1;
        }
        bit_pos += 8;
    }

    const uint8_t gen[10] = {216, 194, 159, 111, 199, 94, 95, 113, 157, 193};
    uint8_t ec[10] = {0};
    for (int i = 0; i < 34; i++) {
        uint8_t factor = data[i] ^ ec[0];
        for (int j = 0; j < 9; j++) ec[j] = ec[j+1] ^ gf_mul(factor, gen[j]);
        ec[9] = gf_mul(factor, gen[9]);
    }
    for (int i = 0; i < 10; i++) data[34 + i] = ec[i];

    memset(qrcode, 2, 25*25);

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            bool dark = (i==0||i==6||j==0||j==6||(i>=2&&i<=4&&j>=2&&j<=4));
            qrcode[i][j] = dark;
            qrcode[i][24-j] = dark;
            qrcode[24-i][j] = dark;
        }
    }
    for (int i=0; i<=7; i++) {
        qrcode[7][i] = 0; qrcode[i][7] = 0;
        qrcode[7][24-i] = 0; qrcode[i][24-7] = 0;
        qrcode[24-7][i] = 0; qrcode[24-i][7] = 0;
    }
    for (int i=-2; i<=2; i++) {
        for (int j=-2; j<=2; j++) {
            qrcode[18+i][18+j] = (i==-2||i==2||j==-2||j==2||(i==0&&j==0));
        }
    }
    for (int i=8; i<25-8; i++) { qrcode[6][i] = (i%2==0); qrcode[i][6] = (i%2==0); }
    qrcode[24-7][8] = 1;

    uint16_t fmt = 0x77c4;
    for (int i=0; i<15; i++) {
        bool dark = (fmt >> i) & 1;
        if (i < 6) { qrcode[i][8] = dark; qrcode[8][25-1-i] = dark; }
        else if (i < 8) { qrcode[i+1][8] = dark; qrcode[8][25-1-i] = dark; }
        else { qrcode[8][i==8 ? 7 : 14-i] = dark; qrcode[25-15+i][8] = dark; }
    }

    int bit_idx = 0;
    int dir = -1;
    int y = 24;
    for (int x = 24; x > 0; x -= 2) {
        if (x == 6) x--;
        while (y >= 0 && y < 25) {
            for (int j = 0; j < 2; j++) {
                if (qrcode[y][x - j] == 2) {
                    bool bit = 0;
                    if (bit_idx < 44 * 8) bit = (data[bit_idx / 8] >> (7 - (bit_idx % 8))) & 1;
                    bool mask = ((y + x - j) % 2 == 0);
                    qrcode[y][x - j] = bit ^ mask;
                    bit_idx++;
                }
            }
            y += dir;
        }
        dir = -dir;
        y += dir;
    }
}
