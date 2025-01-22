// Adapted from: https://www.hanshq.net/fire.html
#include "Arduino.h"
#include "rmt_led_strip.h"

using namespace htcw;
// a "string" of 256 leds on GPIO 18
ws2812 leds(18,256);
// a panel presents a string as a 2d matrix
// here we declare one 8 pixels wide
// 32 height is implicit as 256/8=32
led_panel panel(leds,8);

#define WIDTH 8
#define HEIGHT (32 + 2) // extra 2 rows

static const uint32_t fire_palette[256] = {
// Jare's original FirePal.
#define C(r,g,b) ((((r) * 4) << 16) | (((g) * 4) << 8) | (((b) * 4)<<0))
C( 0,   0,   0), C( 0,   1,   1), C( 0,   4,   5), C( 0,   7,   9),
C( 0,   8,  11), C( 0,   9,  12), C(15,   6,   8), C(25,   4,   4),
C(33,   3,   3), C(40,   2,   2), C(48,   2,   2), C(55,   1,   1),
C(63,   0,   0), C(63,   0,   0), C(63,   3,   0), C(63,   7,   0),
C(63,  10,   0), C(63,  13,   0), C(63,  16,   0), C(63,  20,   0),
C(63,  23,   0), C(63,  26,   0), C(63,  29,   0), C(63,  33,   0),
C(63,  36,   0), C(63,  39,   0), C(63,  39,   0), C(63,  40,   0),
C(63,  40,   0), C(63,  41,   0), C(63,  42,   0), C(63,  42,   0),
C(63,  43,   0), C(63,  44,   0), C(63,  44,   0), C(63,  45,   0),
C(63,  45,   0), C(63,  46,   0), C(63,  47,   0), C(63,  47,   0),
C(63,  48,   0), C(63,  49,   0), C(63,  49,   0), C(63,  50,   0),
C(63,  51,   0), C(63,  51,   0), C(63,  52,   0), C(63,  53,   0),
C(63,  53,   0), C(63,  54,   0), C(63,  55,   0), C(63,  55,   0),
C(63,  56,   0), C(63,  57,   0), C(63,  57,   0), C(63,  58,   0),
C(63,  58,   0), C(63,  59,   0), C(63,  60,   0), C(63,  60,   0),
C(63,  61,   0), C(63,  62,   0), C(63,  62,   0), C(63,  63,   0),
/* Followed by "white heat". */
#define W C(63,63,63)
W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W,
W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W,
W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W,
W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W,
W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W,
W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W,
W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W,
W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W
#undef W
#undef C
};

static uint8_t fire[WIDTH * HEIGHT];
static uint8_t prev_fire[WIDTH * HEIGHT];
static uint32_t framebuf[WIDTH * HEIGHT];


void setup() {
    Serial.begin(115200);
    leds.initialize();
    panel.initialize();
}
void loop() {
    int i;
    uint32_t sum;
    uint8_t avg;    
    for (i = WIDTH + 1; i < (HEIGHT - 1) * WIDTH - 1; i++) {
            /* Average the eight neighbours. */
            sum = prev_fire[i - WIDTH - 1] +
                    prev_fire[i - WIDTH    ] +
                    prev_fire[i - WIDTH + 1] +
                    prev_fire[i - 1] +
                    prev_fire[i + 1] +
                    prev_fire[i + WIDTH - 1] +
                    prev_fire[i + WIDTH    ] +
                    prev_fire[i + WIDTH + 1];
            avg = (uint8_t)(sum / 8);

            /* "Cool" the pixel if the two bottom bits of the
                sum are clear (somewhat random). For the bottom
                rows, cooling can overflow, causing "sparks". */
            if (!(sum & 3) &&
                (avg > 0 || i >= (HEIGHT - 4) * WIDTH)) {
                    avg--;
            }
            fire[i] = avg;
    }

    /* Copy back and scroll up one row.
        The bottom row is all zeros, so it can be skipped. */
    for (i = 0; i < (HEIGHT - 2) * WIDTH; i++) {
            prev_fire[i] = fire[i + WIDTH];
    }

    /* Remove dark pixels from the bottom rows (except again the
        bottom row which is all zeros). */
    for (i = (HEIGHT - 7) * WIDTH; i < (HEIGHT - 1) * WIDTH; i++) {
            if (fire[i] < 15) {
                    fire[i] = 22 - fire[i];
            }
    }

    /* Copy to framebuffer and map to RGBA, scrolling up one row. */
    for (i = 0; i < (HEIGHT - 2) * WIDTH; i++) {
            framebuf[i] = fire_palette[fire[i + WIDTH]];
    }
    // write the frame buffer to the panel
    for(int y = 0;y<HEIGHT-2;++y) {
        for(int x = 0;x<WIDTH;++x) {
            panel.point( x,y,framebuf[y*WIDTH+x]);
        }
    }
    // display the changes
    panel.update();
    delay(50);

}