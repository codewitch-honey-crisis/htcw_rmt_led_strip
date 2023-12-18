// derived from ESP32_LED_STRIP by Lucas Bruder
// https://github.com/Lucas-Bruder/ESP32_LED_STRIP/tree/master/components/led_strip
#include <rmt_led_strip.hpp>

#ifdef ARDUINO
using namespace arduino;
#else
using namespace esp_idf;
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_check.h"
static const char* TAG = "rmt_led_strip";
typedef struct {
    uint32_t resolution; /*!< Encoder resolution, in Hz */
} led_strip_encoder_config_t;
#else
#include <driver/rmt.h>
#endif
#include <driver/gpio.h>
#include <stddef.h>
#include <string.h>

#define LED_STRIP_TASK_SIZE (1024)
#define LED_STRIP_TASK_PRIORITY (configMAX_PRIORITIES - 1)

#define LED_STRIP_REFRESH_PERIOD_MS (30U)  // TODO: add as parameter to led_strip_init

#define LED_STRIP_NUM_RMT_ITEMS_PER_LED (24U)  // Assumes 24 bit color for each led

// RMT Clock source is @ 80 MHz. Dividing it by 8 gives us 10 MHz frequency, or 100ns period.
#define LED_STRIP_RMT_CLK_DIV (8)

/****************************
        WS2812 Timing
 ****************************/
#define LED_STRIP_RMT_TICKS_BIT_1_HIGH_WS2812 9  // 900ns (900ns +/- 150ns per datasheet)
#define LED_STRIP_RMT_TICKS_BIT_1_LOW_WS2812 3   // 300ns (350ns +/- 150ns per datasheet)
#define LED_STRIP_RMT_TICKS_BIT_0_HIGH_WS2812 3  // 300ns (350ns +/- 150ns per datasheet)
#define LED_STRIP_RMT_TICKS_BIT_0_LOW_WS2812 9   // 900ns (900ns +/- 150ns per datasheet)

/****************************
        SK6812 Timing
 ****************************/
#define LED_STRIP_RMT_TICKS_BIT_1_HIGH_SK6812 6
#define LED_STRIP_RMT_TICKS_BIT_1_LOW_SK6812 6
#define LED_STRIP_RMT_TICKS_BIT_0_HIGH_SK6812 3
#define LED_STRIP_RMT_TICKS_BIT_0_LOW_SK6812 9

/****************************
        APA106 Timing
 ****************************/
#define LED_STRIP_RMT_TICKS_BIT_1_HIGH_APA106 14  // 1.36us +/- 150ns per datasheet
#define LED_STRIP_RMT_TICKS_BIT_1_LOW_APA106 3    // 350ns +/- 150ns per datasheet
#define LED_STRIP_RMT_TICKS_BIT_0_HIGH_APA106 3   // 350ns +/- 150ns per datasheet
#define LED_STRIP_RMT_TICKS_BIT_0_LOW_APA106 14   // 1.36us +/- 150ns per datasheet

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)

// Function pointer for generating waveforms based on different LED drivers
typedef void (*led_fill_rmt_items_fn)(uint32_t* led_strip_buf, rmt_item32_t* rmt_items, uint32_t led_strip_length);

static inline void led_strip_fill_item_level(rmt_item32_t* item, int high_ticks, int low_ticks) {
    item->level0 = 1;
    item->duration0 = high_ticks;
    item->level1 = 0;
    item->duration1 = low_ticks;
}

static inline void led_strip_rmt_bit_1_sk6812(rmt_item32_t* item) {
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_1_HIGH_SK6812, LED_STRIP_RMT_TICKS_BIT_1_LOW_SK6812);
}

static inline void led_strip_rmt_bit_0_sk6812(rmt_item32_t* item) {
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_0_HIGH_SK6812, LED_STRIP_RMT_TICKS_BIT_0_LOW_SK6812);
}

static void led_strip_fill_rmt_items_sk6812(uint32_t* led_strip_buf, rmt_item32_t* rmt_items, uint32_t led_strip_length) {
    uint32_t rmt_items_index = 0;
    for (uint32_t led_index = 0; led_index < led_strip_length; led_index++) {
        uint32_t led_color = led_strip_buf[led_index];

        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = (((led_color & 0x00FF00) >> 8) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_sk6812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_sk6812(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = (((led_color & 0xFF0000) >> 16) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_sk6812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_sk6812(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = ((led_color & 0xFF) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_sk6812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_sk6812(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
    }
}

static inline void led_strip_rmt_bit_1_ws2812(rmt_item32_t* item) {
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_1_HIGH_WS2812, LED_STRIP_RMT_TICKS_BIT_1_LOW_WS2812);
}

static inline void led_strip_rmt_bit_0_ws2812(rmt_item32_t* item) {
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_0_HIGH_WS2812, LED_STRIP_RMT_TICKS_BIT_0_LOW_WS2812);
}

static inline void led_strip_rmt_bit_1_apa106(rmt_item32_t* item) {
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_1_HIGH_APA106, LED_STRIP_RMT_TICKS_BIT_1_LOW_APA106);
}

static inline void led_strip_rmt_bit_0_apa106(rmt_item32_t* item) {
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_0_HIGH_APA106, LED_STRIP_RMT_TICKS_BIT_0_LOW_APA106);
}

static void led_strip_fill_rmt_items_apa106(uint32_t* led_strip_buf, rmt_item32_t* rmt_items, uint32_t led_strip_length) {
    uint32_t rmt_items_index = 0;
    for (uint32_t led_index = 0; led_index < led_strip_length; led_index++) {
        uint32_t led_color = led_strip_buf[led_index];

        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = (((led_color & 0xFF0000) >> 16) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_apa106(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_apa106(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = (((led_color & 0x00FF00) >> 8) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_apa106(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_apa106(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = ((led_color & 0xFF) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_apa106(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_apa106(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
    }
}

static void led_strip_fill_rmt_items_ws2812(uint32_t* led_strip_buf, rmt_item32_t* rmt_items, uint32_t led_strip_length) {
    uint32_t rmt_items_index = 0;
    for (uint32_t led_index = 0; led_index < led_strip_length; led_index++) {
        uint32_t led_color = led_strip_buf[led_index];

        for (uint8_t bit = 8; bit != 0; --bit) {
            uint8_t bit_set = (((led_color & 0x00FF00) >> 8) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
            }
            ++rmt_items_index;
        }
        for (uint8_t bit = 8; bit != 0; --bit) {
            uint8_t bit_set = (((led_color & 0xFF0000) >> 16) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
            }
            ++rmt_items_index;
        }
        for (uint8_t bit = 8; bit != 0; --bit) {
            uint8_t bit_set = ((led_color & 0xFF) >> (bit - 1)) & 1;
            if (bit_set) {
                led_strip_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
            }
            ++rmt_items_index;
        }
    }
}
static bool led_strip_initialize(uint8_t pin, size_t length, uint8_t rmt_channel, uint8_t rmt_interrupt, uint32_t** strip, void** rmt_items) {
    if (length == 0) {
        return false;
    }
    *strip = (uint32_t*)malloc(length * sizeof(uint32_t));
    if (*strip == nullptr) {
        return false;
    }
    size_t num_items_malloc = (LED_STRIP_NUM_RMT_ITEMS_PER_LED * length);
    rmt_item32_t* rmts = (rmt_item32_t*)malloc(sizeof(rmt_item32_t) * num_items_malloc);
    if (rmt_items == nullptr) {
        free(*strip);
        *strip = nullptr;
        return false;
    }
    memset(*strip, 0, sizeof(uint32_t) * length);

    rmt_config_t rmt_cfg;
    memset(&rmt_cfg, 0, sizeof(rmt_cfg));

    rmt_cfg.rmt_mode = RMT_MODE_TX;
    rmt_cfg.channel = (rmt_channel_t)rmt_channel;
    rmt_cfg.clk_div = LED_STRIP_RMT_CLK_DIV;
    rmt_cfg.gpio_num = (gpio_num_t)pin;
    rmt_cfg.mem_block_num = 1;

    rmt_cfg.tx_config.loop_en = false;
    rmt_cfg.tx_config.carrier_freq_hz = 100;  // Not used, but has to be set to avoid divide by 0 err
    rmt_cfg.tx_config.carrier_duty_percent = 50;
    rmt_cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    rmt_cfg.tx_config.carrier_en = false;
    rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_cfg.tx_config.idle_output_en = true;

    esp_err_t cfg_ok = rmt_config(&rmt_cfg);
    if (cfg_ok != ESP_OK) {
        free(rmts);
        free(*strip);
        *strip = nullptr;
        return false;
    }
    esp_err_t install_ok = rmt_driver_install(rmt_cfg.channel, 0, 0);
    if (install_ok != ESP_OK) {
        free(*strip);
        free(rmts);
        *strip = nullptr;
        return false;
    }
    *rmt_items = (void*)rmts;
    return true;
}
#else
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = (rmt_encode_state_t)0;
    rmt_encode_state_t state = (rmt_encode_state_t)0;
    size_t encoded_symbols = 0;
    switch (led_encoder->state) {
        case 0:  // send RGB data
            encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                led_encoder->state = 1;  // switch to next state when current encoding session finished
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state = (rmt_encode_state_t)(((uint32_t)state) | RMT_ENCODING_MEM_FULL);
                goto out;  // yield if there's no free space for encoding artifacts
            }
        // fall-through
        case 1:  // send reset code
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                    sizeof(led_encoder->reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                led_encoder->state = 0;  // back to the initial encoding session
                state = (rmt_encode_state_t)(((uint32_t)state) | RMT_ENCODING_COMPLETE);
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state = (rmt_encode_state_t)(((uint32_t)state) | RMT_ENCODING_MEM_FULL);
                goto out;  // yield if there's no free space for encoding artifacts
            }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = 0;
    return ESP_OK;
}

esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, size_t ticks_lo_bit0, size_t ticks_hi_bit0, size_t ticks_lo_bit1, size_t ticks_hi_bit1, rmt_encoder_handle_t *ret_encoder) {
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t *led_encoder = NULL;
    rmt_bytes_encoder_config_t bytes_encoder_config;
    rmt_copy_encoder_config_t copy_encoder_config = {};
    // rmt_symbol_word_t reset_code;
    uint32_t reset_ticks;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    led_encoder = (rmt_led_strip_encoder_t *)calloc(1, sizeof(rmt_led_strip_encoder_t));
    ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for led strip encoder");
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    // different led strip might have its own timing requirements, following parameter is for WS2812
    bytes_encoder_config.bit0.level0 = 1;

    bytes_encoder_config.bit0.duration0 = ((float)ticks_hi_bit0 / 10.0f) * config->resolution / 1000000;  // T0H=0.3us
    bytes_encoder_config.bit0.level1 = 0;
    bytes_encoder_config.bit0.duration1 = ((float)ticks_lo_bit0 / 10.0f) * config->resolution / 1000000;  // T0L=0.9us

    bytes_encoder_config.bit1.level0 = 1;
    bytes_encoder_config.bit1.duration0 = ((float)ticks_hi_bit1 / 10.0f) * config->resolution / 1000000;  // T1H=0.9us
    bytes_encoder_config.bit1.level1 = 0;
    bytes_encoder_config.bit1.duration1 = ((float)ticks_lo_bit1 / 10.0f) * config->resolution / 1000000;  // T1L=0.3us

    bytes_encoder_config.flags.msb_first = 1;  // WS2812 transfer bit order: G7...G0R7...R0B7...B0

    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder), err, TAG, "create copy encoder failed");
    reset_ticks = config->resolution / 1000000 * 50 / 2;  // reset code duration defaults to 50us
    led_encoder->reset_code.level0 = 0;
    led_encoder->reset_code.duration0 = reset_ticks;
    led_encoder->reset_code.level1 = 0;
    led_encoder->reset_code.duration1 = reset_ticks;
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
err:
    if (led_encoder) {
        if (led_encoder->bytes_encoder) {
            rmt_del_encoder(led_encoder->bytes_encoder);
        }
        if (led_encoder->copy_encoder) {
            rmt_del_encoder(led_encoder->copy_encoder);
        }
        free(led_encoder);
    }
    return ret;
}
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000  // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
static bool led_strip_initialize(uint8_t pin, size_t length, uint8_t rmt_channel, uint8_t rmt_interrupt, size_t ticks_lo_bit0, size_t ticks_hi_bit0, size_t ticks_lo_bit1, size_t ticks_hi_bit1, void **strip, void **encoder, void **channel) {
    *encoder = nullptr;
    *channel = nullptr;
    if (length == 0) {
        *strip = nullptr;
        return false;
    }
    *strip = malloc(length * 3);
    if (*strip == nullptr) {
        return false;
    }
    memset(*strip, 0, length * 3);
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config;
    memset(&tx_chan_config, 0, sizeof(rmt_tx_channel_config_t));
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;  // select source clock
    tx_chan_config.gpio_num = (gpio_num_t)pin;
    tx_chan_config.mem_block_symbols = 64;  // increase the block size can make the LED less flickering
    tx_chan_config.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ;
    tx_chan_config.trans_queue_depth = 4;  // set the number of transactions that can be pending in the background

    if (ESP_OK != rmt_new_tx_channel(&tx_chan_config, &led_chan)) {
        free(*strip);
        *strip = nullptr;
        return false;
    }

    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ};
    if (ESP_OK != rmt_new_led_strip_encoder(&encoder_config, ticks_lo_bit0, ticks_hi_bit0, ticks_lo_bit1, ticks_hi_bit1, &led_encoder)) {
        free(*strip);
        *strip = nullptr;
        rmt_del_channel(led_chan);
        return false;
    }

    if (ESP_OK != rmt_enable(led_chan)) {
        free(*strip);
        *strip = nullptr;
        rmt_del_led_strip_encoder(led_encoder);
        rmt_del_channel(led_chan);
    }

    *encoder = led_encoder;
    *channel = led_chan;
    return true;
}
#endif
ws2812::ws2812(uint8_t pin, size_t length, uint8_t rmt_channel, uint8_t rmt_interrupt) : m_pin(pin),
                                                                                         m_length(length),
                                                                                         m_rmt_channel(rmt_channel),
                                                                                         m_rmt_interrupt(rmt_interrupt),
                                                                                         m_strip(nullptr),
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
                                                                                         m_rmt_items(nullptr) {
#else
                                                                                         m_encoder(nullptr),
                                                                                         m_channel(nullptr){
#endif
}
ws2812::ws2812(ws2812&& rhs) {
    do_move(rhs);
}
ws2812& ws2812::operator=(ws2812&& rhs) {
    deinitialize();
    do_move(rhs);
    return *this;
}
ws2812::~ws2812() {
    deinitialize();
}
void ws2812::do_move(ws2812& rhs) {
    m_pin = rhs.m_pin;
    m_length = rhs.m_length;
    rhs.m_length = 0;
    m_rmt_channel = rhs.m_rmt_channel;
    m_rmt_interrupt = rhs.m_rmt_channel;
    m_strip = rhs.m_strip;
    rhs.m_strip = nullptr;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    m_rmt_items = rhs.m_rmt_items;
    rhs.m_rmt_items = nullptr;
#else
    m_encoder = rhs.m_encoder;
    rhs.m_encoder = nullptr;
    m_channel = rhs.m_channel;
    rhs.m_channel = nullptr;
#endif
}
size_t ws2812::length() const {
    return m_length;
}
bool ws2812::initialized() const {
    return m_strip != nullptr;
}
bool ws2812::initialize() {
    if (m_strip != nullptr) {
        return true;
    }
    if (m_length == 0) {
        return false;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    return led_strip_initialize(m_pin, m_length,
                                m_rmt_channel,
                                m_rmt_interrupt,
                                LED_STRIP_RMT_TICKS_BIT_0_LOW_WS2812,
                                LED_STRIP_RMT_TICKS_BIT_0_HIGH_WS2812,
                                LED_STRIP_RMT_TICKS_BIT_1_LOW_WS2812,
                                LED_STRIP_RMT_TICKS_BIT_1_HIGH_WS2812,
                                &m_strip,
                                &m_encoder,
                                &m_channel);
#else
    return led_strip_initialize(m_pin, m_length, m_rmt_channel, m_rmt_interrupt, (uint32_t **)&m_strip, &m_rmt_items);
#endif
}
void ws2812::deinitialize() {
    if (m_strip == nullptr) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rmt_del_led_strip_encoder((rmt_encoder_t*)m_encoder);
    rmt_del_channel((rmt_channel_handle_t)m_channel);

    free(m_strip);
    m_strip = nullptr;
#else
    rmt_driver_uninstall((rmt_channel_t)m_rmt_channel);
    free(m_strip);
    free(m_rmt_items);
    m_strip = nullptr;
#endif
}
void ws2812::update() {
    if (m_strip == nullptr) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rmt_transmit_config_t tx_config;
    tx_config.flags.eot_level = 0;
    tx_config.loop_count = 0;
    rmt_tx_wait_all_done((rmt_channel_handle_t)m_channel, portMAX_DELAY);
    rmt_transmit((rmt_channel_handle_t)m_channel, (rmt_encoder_handle_t)m_encoder, m_strip, m_length * 3, &tx_config);
#else
    rmt_wait_tx_done((rmt_channel_t)m_rmt_channel, portMAX_DELAY);
    led_strip_fill_rmt_items_ws2812((uint32_t *)m_strip, (rmt_item32_t *)m_rmt_items, m_length);
    rmt_write_items((rmt_channel_t)m_rmt_channel, (rmt_item32_t *)m_rmt_items, (LED_STRIP_NUM_RMT_ITEMS_PER_LED * m_length), false);
#endif
}

uint32_t ws2812::color(size_t index) const {
    if (m_strip == nullptr || index >= m_length) {
        return 0;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    size_t i = index * 3;
    return (((uint8_t*)m_strip)[i + 1] << 16) |
           (((uint8_t*)m_strip)[i + 0] << 8) |
           (((uint8_t*)m_strip)[i + 2]);
#else
    return ((uint32_t *)m_strip)[index];
#endif
}
void ws2812::color(size_t index, uint32_t value) {
    if (m_strip == nullptr || index >= m_length) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    int i = index * 3;
    ((uint8_t*)m_strip)[i + 1] = (value & 0xFF0000) >> 16;
    ((uint8_t*)m_strip)[i + 0] = (value & 0x00FF00) >> 8;
    ((uint8_t*)m_strip)[i + 2] = (value & 0x0000FF);
#else
    ((uint32_t *)m_strip)[index] = value;
#endif
}
void ws2812::color(size_t index, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t value = (r << 16) | (g << 8) | b;
    color(index, value);
}

sk6812::sk6812(uint8_t pin, size_t length, uint8_t rmt_channel, uint8_t rmt_interrupt) : m_pin(pin),
                                                                                         m_length(length),
                                                                                         m_rmt_channel(rmt_channel),
                                                                                         m_rmt_interrupt(rmt_interrupt),
                                                                                         m_strip(nullptr),
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
                                                                                         m_rmt_items(nullptr) {
#else
                                                                                         m_encoder(nullptr),
                                                                                         m_channel(nullptr){
#endif
}
sk6812::sk6812(sk6812&& rhs) {
    do_move(rhs);
}
sk6812& sk6812::operator=(sk6812&& rhs) {
    deinitialize();
    do_move(rhs);
    return *this;
}
sk6812::~sk6812() {
    deinitialize();
}
void sk6812::do_move(sk6812& rhs) {
    m_pin = rhs.m_pin;
    m_length = rhs.m_length;
    rhs.m_length = 0;
    m_rmt_channel = rhs.m_rmt_channel;
    m_rmt_interrupt = rhs.m_rmt_channel;
    m_strip = rhs.m_strip;
    rhs.m_strip = nullptr;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    m_rmt_items = rhs.m_rmt_items;
    rhs.m_rmt_items = nullptr;
#else
    m_encoder = rhs.m_encoder;
    rhs.m_encoder = nullptr;
    m_channel = rhs.m_channel;
    rhs.m_channel = nullptr;
#endif
}

size_t sk6812::length() const {
    return m_length;
}
bool sk6812::initialized() const {
    return m_strip != nullptr;
}
bool sk6812::initialize() {
    if (m_strip != nullptr) {
        return true;
    }
    if (m_length == 0) {
        return false;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    return led_strip_initialize(m_pin, m_length,
                                m_rmt_channel,
                                m_rmt_interrupt,
                                LED_STRIP_RMT_TICKS_BIT_0_LOW_SK6812,
                                LED_STRIP_RMT_TICKS_BIT_0_HIGH_SK6812,
                                LED_STRIP_RMT_TICKS_BIT_1_LOW_SK6812,
                                LED_STRIP_RMT_TICKS_BIT_1_HIGH_SK6812,
                                &m_strip,
                                &m_encoder,
                                &m_channel);
#else
    return led_strip_initialize(m_pin, m_length, m_rmt_channel, m_rmt_interrupt, (uint32_t **)&m_strip, &m_rmt_items);
#endif
}
void sk6812::deinitialize() {
    if (m_strip == nullptr) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rmt_del_led_strip_encoder((rmt_encoder_t*)m_encoder);
    rmt_del_channel((rmt_channel_handle_t)m_channel);

    free(m_strip);
    m_strip = nullptr;
#else
    rmt_driver_uninstall((rmt_channel_t)m_rmt_channel);
    free(m_strip);
    free(m_rmt_items);
    m_strip = nullptr;
#endif
}
void sk6812::update() {
    if (m_strip == nullptr) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rmt_transmit_config_t tx_config;
    tx_config.flags.eot_level = 0;
    tx_config.loop_count = 0;
    // rmt_tx_wait_all_done((rmt_channel_handle_t)m_channel,portMAX_DELAY);
    rmt_transmit((rmt_channel_handle_t)m_channel, (rmt_encoder_handle_t)m_encoder, m_strip, m_length * 3, &tx_config);
#else
    rmt_wait_tx_done((rmt_channel_t)m_rmt_channel, portMAX_DELAY);
    led_strip_fill_rmt_items_sk6812((uint32_t *)m_strip, (rmt_item32_t *)m_rmt_items, m_length);
    rmt_write_items((rmt_channel_t)m_rmt_channel, (rmt_item32_t *)m_rmt_items, (LED_STRIP_NUM_RMT_ITEMS_PER_LED * m_length), false);
#endif
}

uint32_t sk6812::color(size_t index) const {
    if (m_strip == nullptr || index >= m_length) {
        return 0;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    size_t i = index * 3;
    return (((uint8_t*)m_strip)[i + 1] << 16) |
           (((uint8_t*)m_strip)[i + 0] << 8) |
           (((uint8_t*)m_strip)[i + 2]);
#else
    return ((uint32_t *)m_strip)[index];
#endif
}
void sk6812::color(size_t index, uint32_t value) {
    if (m_strip == nullptr || index >= m_length) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    int i = index * 3;
    ((uint8_t*)m_strip)[i + 1] = (value & 0xFF0000) >> 16;
    ((uint8_t*)m_strip)[i + 0] = (value & 0x00FF00) >> 8;
    ((uint8_t*)m_strip)[i + 2] = (value & 0x0000FF);
#else
    ((uint32_t *)m_strip)[index] = value;
#endif
}
void sk6812::color(size_t index, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t value = (r << 16) | (g << 8) | b;
    color(index, value);
}

apa106::apa106(uint8_t pin, size_t length, uint8_t rmt_channel, uint8_t rmt_interrupt) : m_pin(pin),
                                                                                         m_length(length),
                                                                                         m_rmt_channel(rmt_channel),
                                                                                         m_rmt_interrupt(rmt_interrupt),
                                                                                         m_strip(nullptr),
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
                                                                                         m_rmt_items(nullptr) {
#else
                                                                                         m_encoder(nullptr),
                                                                                         m_channel(nullptr){
#endif
}
apa106::apa106(apa106&& rhs) {
    do_move(rhs);
}
apa106& apa106::operator=(apa106&& rhs) {
    deinitialize();
    do_move(rhs);
    return *this;
}
apa106::~apa106() {
    deinitialize();
}
void apa106::do_move(apa106& rhs) {
    m_pin = rhs.m_pin;
    m_length = rhs.m_length;
    rhs.m_length = 0;
    m_rmt_channel = rhs.m_rmt_channel;
    m_rmt_interrupt = rhs.m_rmt_channel;
    m_strip = rhs.m_strip;
    rhs.m_strip = nullptr;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    m_rmt_items = rhs.m_rmt_items;
    rhs.m_rmt_items = nullptr;
#else
    m_encoder = rhs.m_encoder;
    rhs.m_encoder = nullptr;
    m_channel = rhs.m_channel;
    rhs.m_channel = nullptr;
#endif
}

size_t apa106::length() const {
    return m_length;
}
bool apa106::initialized() const {
    return m_strip != nullptr;
}
bool apa106::initialize() {
    if (m_strip != nullptr) {
        return true;
    }
    if (m_length == 0) {
        return false;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    return led_strip_initialize(m_pin, m_length,
                                m_rmt_channel,
                                m_rmt_interrupt,
                                LED_STRIP_RMT_TICKS_BIT_0_LOW_APA106,
                                LED_STRIP_RMT_TICKS_BIT_0_HIGH_APA106,
                                LED_STRIP_RMT_TICKS_BIT_1_LOW_APA106,
                                LED_STRIP_RMT_TICKS_BIT_1_HIGH_APA106,
                                &m_strip,
                                &m_encoder,
                                &m_channel);
#else
    return led_strip_initialize(m_pin, m_length, m_rmt_channel, m_rmt_interrupt, (uint32_t **)&m_strip, &m_rmt_items);
#endif
}
void apa106::deinitialize() {
    if (m_strip == nullptr) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rmt_del_led_strip_encoder((rmt_encoder_t*)m_encoder);
    rmt_del_channel((rmt_channel_handle_t)m_channel);

    free(m_strip);
    m_strip = nullptr;
#else
    rmt_driver_uninstall((rmt_channel_t)m_rmt_channel);
    free(m_strip);
    free(m_rmt_items);
    m_strip = nullptr;
#endif
}
void apa106::update() {
    if (m_strip == nullptr) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rmt_transmit_config_t tx_config;
    tx_config.flags.eot_level = 0;
    tx_config.loop_count = 0;
    rmt_tx_wait_all_done((rmt_channel_handle_t)m_channel, portMAX_DELAY);
    rmt_transmit((rmt_channel_handle_t)m_channel, (rmt_encoder_handle_t)m_encoder, m_strip, m_length * 3, &tx_config);
#else
    rmt_wait_tx_done((rmt_channel_t)m_rmt_channel, portMAX_DELAY);
    led_strip_fill_rmt_items_apa106((uint32_t *)m_strip, (rmt_item32_t *)m_rmt_items, m_length);
    rmt_write_items((rmt_channel_t)m_rmt_channel, (rmt_item32_t *)m_rmt_items, (LED_STRIP_NUM_RMT_ITEMS_PER_LED * m_length), false);
#endif
}

uint32_t apa106::color(size_t index) const {
    if (m_strip == nullptr || index >= m_length) {
        return 0;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    size_t i = index * 3;
    return (((uint8_t*)m_strip)[i + 1] << 16) |
           (((uint8_t*)m_strip)[i + 0] << 8) |
           (((uint8_t*)m_strip)[i + 2]);
#else
    return ((uint32_t *)m_strip)[index];
#endif
}
void apa106::color(size_t index, uint32_t value) {
    if (m_strip == nullptr || index >= m_length) {
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    int i = index * 3;
    ((uint8_t*)m_strip)[i + 1] = (value & 0xFF0000) >> 16;
    ((uint8_t*)m_strip)[i + 0] = (value & 0x00FF00) >> 8;
    ((uint8_t*)m_strip)[i + 2] = (value & 0x0000FF);
#else
    ((uint32_t *)m_strip)[index] = value;
#endif
}
void apa106::color(size_t index, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t value = (r << 16) | (g << 8) | b;
    color(index, value);
}