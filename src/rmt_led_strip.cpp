// derived from ESP32_LED_STRIP by Lucas Bruder
// https://github.com/Lucas-Bruder/ESP32_LED_STRIP/tree/master/components/led_strip
#include <rmt_led_strip.hpp>
#ifdef ARDUINO
using namespace arduino;
#else
using namespace esp_idf;
#endif
#include <driver/rmt.h>
#include <driver/gpio.h>
#include <stddef.h>
#include <string.h>

#define LED_STRIP_TASK_SIZE             (1024)
#define LED_STRIP_TASK_PRIORITY         (configMAX_PRIORITIES - 1)

#define LED_STRIP_REFRESH_PERIOD_MS     (30U) // TODO: add as parameter to led_strip_init

#define LED_STRIP_NUM_RMT_ITEMS_PER_LED (24U) // Assumes 24 bit color for each led

// RMT Clock source is @ 80 MHz. Dividing it by 8 gives us 10 MHz frequency, or 100ns period.
#define LED_STRIP_RMT_CLK_DIV (8)

/****************************
        WS2812 Timing
 ****************************/
#define LED_STRIP_RMT_TICKS_BIT_1_HIGH_WS2812 9 // 900ns (900ns +/- 150ns per datasheet)
#define LED_STRIP_RMT_TICKS_BIT_1_LOW_WS2812  3 // 300ns (350ns +/- 150ns per datasheet)
#define LED_STRIP_RMT_TICKS_BIT_0_HIGH_WS2812 3 // 300ns (350ns +/- 150ns per datasheet)
#define LED_STRIP_RMT_TICKS_BIT_0_LOW_WS2812  9 // 900ns (900ns +/- 150ns per datasheet)

/****************************
        SK6812 Timing
 ****************************/
#define LED_STRIP_RMT_TICKS_BIT_1_HIGH_SK6812 6
#define LED_STRIP_RMT_TICKS_BIT_1_LOW_SK6812  6
#define LED_STRIP_RMT_TICKS_BIT_0_HIGH_SK6812 3
#define LED_STRIP_RMT_TICKS_BIT_0_LOW_SK6812  9

/****************************
        APA106 Timing
 ****************************/
#define LED_STRIP_RMT_TICKS_BIT_1_HIGH_APA106 14 // 1.36us +/- 150ns per datasheet
#define LED_STRIP_RMT_TICKS_BIT_1_LOW_APA106   3 // 350ns +/- 150ns per datasheet
#define LED_STRIP_RMT_TICKS_BIT_0_HIGH_APA106  3 // 350ns +/- 150ns per datasheet
#define LED_STRIP_RMT_TICKS_BIT_0_LOW_APA106  14 // 1.36us +/- 150ns per datasheet




// Function pointer for generating waveforms based on different LED drivers
typedef void (*led_fill_rmt_items_fn)(uint32_t*led_strip_buf, rmt_item32_t *rmt_items, uint32_t led_strip_length);

static inline void led_strip_fill_item_level(rmt_item32_t* item, int high_ticks, int low_ticks)
{
    item->level0 = 1;
    item->duration0 = high_ticks;
    item->level1 = 0;
    item->duration1 = low_ticks;
}

static inline void led_strip_rmt_bit_1_sk6812(rmt_item32_t* item)
{
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_1_HIGH_SK6812, LED_STRIP_RMT_TICKS_BIT_1_LOW_SK6812);
}

static inline void led_strip_rmt_bit_0_sk6812(rmt_item32_t* item)
{
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_0_HIGH_SK6812, LED_STRIP_RMT_TICKS_BIT_0_LOW_SK6812);
}

static void led_strip_fill_rmt_items_sk6812(uint32_t *led_strip_buf, rmt_item32_t *rmt_items, uint32_t led_strip_length)
{
    uint32_t rmt_items_index = 0;
    for (uint32_t led_index = 0; led_index < led_strip_length; led_index++) {
        uint32_t led_color = led_strip_buf[led_index];

        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = (((led_color&0x00FF00)>>8) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_sk6812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_sk6812(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = (((led_color&0xFF0000)>>16) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_sk6812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_sk6812(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = ((led_color&0xFF) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_sk6812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_sk6812(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
    }
}

static inline void led_strip_rmt_bit_1_ws2812(rmt_item32_t* item)
{
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_1_HIGH_WS2812, LED_STRIP_RMT_TICKS_BIT_1_LOW_WS2812);
}

static inline void led_strip_rmt_bit_0_ws2812(rmt_item32_t* item)
{
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_0_HIGH_WS2812, LED_STRIP_RMT_TICKS_BIT_0_LOW_WS2812);
}


static inline void led_strip_rmt_bit_1_apa106(rmt_item32_t* item)
{
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_1_HIGH_APA106, LED_STRIP_RMT_TICKS_BIT_1_LOW_APA106);
}

static inline void led_strip_rmt_bit_0_apa106(rmt_item32_t* item)
{
    led_strip_fill_item_level(item, LED_STRIP_RMT_TICKS_BIT_0_HIGH_APA106, LED_STRIP_RMT_TICKS_BIT_0_LOW_APA106);
}

static void led_strip_fill_rmt_items_apa106(uint32_t *led_strip_buf, rmt_item32_t *rmt_items, uint32_t led_strip_length)
{
    uint32_t rmt_items_index = 0;
    for (uint32_t led_index = 0; led_index < led_strip_length; led_index++) {
        uint32_t led_color = led_strip_buf[led_index];

        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = (((led_color&0xFF0000)>>16) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_apa106(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_apa106(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = (((led_color&0x00FF00)>>8) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_apa106(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_apa106(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
        for (uint8_t bit = 8; bit != 0; bit--) {
            uint8_t bit_set = ((led_color&0xFF) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_apa106(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_apa106(&(rmt_items[rmt_items_index]));
            }
            rmt_items_index++;
        }
    }
}

static void led_strip_fill_rmt_items_ws2812(uint32_t *led_strip_buf, rmt_item32_t *rmt_items, uint32_t led_strip_length)
{
    uint32_t rmt_items_index = 0;
    for (uint32_t led_index = 0; led_index < led_strip_length; led_index++) {
        uint32_t led_color = led_strip_buf[led_index];

        for (uint8_t bit = 8; bit != 0; --bit) {
            uint8_t bit_set = (((led_color&0x00FF00)>>8) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
            }
            ++rmt_items_index;
        }
        for (uint8_t bit = 8; bit != 0; --bit) {
            uint8_t bit_set = (((led_color&0xFF0000)>>16) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
            }
            ++rmt_items_index;
        }
        for (uint8_t bit = 8; bit != 0; --bit) {
            uint8_t bit_set = ((led_color&0xFF) >> (bit - 1)) & 1;
            if(bit_set) {
                led_strip_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
            } else {
                led_strip_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
            }
            ++rmt_items_index;
        }
    }
}
static bool led_strip_initialize(uint8_t pin,size_t length,uint8_t rmt_channel,uint8_t rmt_interrupt,uint32_t **strip,void**rmt_items) {
    if(length==0) {
        return false;
    }
    *strip = (uint32_t*)malloc(length *sizeof(uint32_t));
    if(*strip==nullptr) {
        return false;
    }
    size_t num_items_malloc = (LED_STRIP_NUM_RMT_ITEMS_PER_LED * length);
    rmt_item32_t *rmts = (rmt_item32_t*) malloc(sizeof(rmt_item32_t) * num_items_malloc);
    if (rmt_items==nullptr) {
        free(*strip);
        *strip=nullptr;
        return false;
    }
    memset(*strip, 0, sizeof(uint32_t) * length);
    
    rmt_config_t rmt_cfg;
    memset(&rmt_cfg,0,sizeof(rmt_cfg));
    
    rmt_cfg.rmt_mode = RMT_MODE_TX;
    rmt_cfg.channel = (rmt_channel_t)rmt_channel;
    rmt_cfg.clk_div = LED_STRIP_RMT_CLK_DIV;
    rmt_cfg.gpio_num = (gpio_num_t)pin;
    rmt_cfg.mem_block_num = 1;

    rmt_cfg.tx_config.loop_en = false;
    rmt_cfg.tx_config.carrier_freq_hz = 100; // Not used, but has to be set to avoid divide by 0 err
    rmt_cfg.tx_config.carrier_duty_percent = 50;
    rmt_cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    rmt_cfg.tx_config.carrier_en = false;
    rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_cfg.tx_config.idle_output_en = true;


    esp_err_t cfg_ok = rmt_config(&rmt_cfg);
    if (cfg_ok != ESP_OK) {
        free(rmts);
        free(*strip);
        *strip=nullptr;
        return false;
    }
    esp_err_t install_ok = rmt_driver_install(rmt_cfg.channel, 0, 0);
    if (install_ok != ESP_OK) {
        free(*strip);
        free(rmts);
        *strip=nullptr;
        return false;
    }
    *rmt_items = (void*)rmts;
    return true;
}

ws2812::ws2812(uint8_t pin,size_t length, uint8_t rmt_channel, uint8_t rmt_interrupt) : 
        m_pin(pin),
        m_length(length),
        m_rmt_channel(rmt_channel),
        m_rmt_interrupt(rmt_interrupt),
        m_strip(nullptr),
        m_rmt_items(nullptr) {

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
    m_rmt_items = rhs.m_rmt_items;
    rhs.m_rmt_items = nullptr;
}
size_t ws2812::length() const {
    return m_length;
}
bool ws2812::initialized() const {
    return m_strip!=nullptr;
}
bool ws2812::initialize() {
    if(m_length==0) {
        return false;
    }
    return led_strip_initialize(m_pin,m_length,m_rmt_channel,m_rmt_interrupt,&m_strip,&m_rmt_items);
}
void ws2812::deinitialize() {
    if(m_strip==nullptr) {
        return;
    }
    rmt_driver_uninstall((rmt_channel_t)m_rmt_channel);
    free(m_strip);
    free(m_rmt_items);
}
void ws2812::update() {
    if(m_strip == nullptr) {
        return;
    }
    rmt_wait_tx_done((rmt_channel_t)m_rmt_channel,portMAX_DELAY);
    led_strip_fill_rmt_items_ws2812(m_strip, (rmt_item32_t*)m_rmt_items, m_length);
    rmt_write_items((rmt_channel_t)m_rmt_channel, (rmt_item32_t*)m_rmt_items,  (LED_STRIP_NUM_RMT_ITEMS_PER_LED * m_length), false);
}

uint32_t ws2812::color(size_t index) const {
    if(m_strip==nullptr || index<0 || index>=m_length) {
        return 0;
    }
    return m_strip[index];
}
void ws2812::color(size_t index,uint32_t value) {
    if(m_strip==nullptr || index<0 || index>=m_length) {
        return;
    }
    m_strip[index]=value;
}
void ws2812::color(size_t index,uint8_t r,uint8_t g, uint8_t b) {
    uint32_t value = (r<<16)|(g<<8)|b;
    color(index,value);
}

sk6812::sk6812(uint8_t pin,size_t length, uint8_t rmt_channel, uint8_t rmt_interrupt) : 
        m_pin(pin),
        m_length(length),
        m_rmt_channel(rmt_channel),
        m_rmt_interrupt(rmt_interrupt),
        m_strip(nullptr),
        m_rmt_items(nullptr) {

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
    m_rmt_items = rhs.m_rmt_items;
    rhs.m_rmt_items = nullptr;
}

size_t sk6812::length() const {
    return m_length;
}
bool sk6812::initialized() const {
    return m_strip!=nullptr;
}
bool sk6812::initialize() {
    if(m_length==0) {
        return false;
    }
    return led_strip_initialize(m_pin,m_length,m_rmt_channel,m_rmt_interrupt,&m_strip,&m_rmt_items);
}
void sk6812::deinitialize() {
    if(m_strip==nullptr) {
        return;
    }
    rmt_driver_uninstall((rmt_channel_t)m_rmt_channel);
    free(m_strip);
    free(m_rmt_items);
}
void sk6812::update() {
    if(m_strip==nullptr) {
        return;
    }
    rmt_wait_tx_done((rmt_channel_t)m_rmt_channel,portMAX_DELAY);
    led_strip_fill_rmt_items_sk6812(m_strip, (rmt_item32_t*)m_rmt_items, m_length);
    rmt_write_items((rmt_channel_t)m_rmt_channel, (rmt_item32_t*)m_rmt_items,  (LED_STRIP_NUM_RMT_ITEMS_PER_LED * m_length), false);
}

uint32_t sk6812::color(size_t index) const {
    if(m_strip==nullptr || index<0 || index>=m_length) {
        return 0;
    }
    return m_strip[index];
}
void sk6812::color(size_t index,uint32_t value) {
    if(m_strip==nullptr || index<0 || index>=m_length) {
        return;
    }
    m_strip[index]=value;
}
void sk6812::color(size_t index,uint8_t r,uint8_t g, uint8_t b) {
    uint32_t value = (r<<16)|(g<<8)|b;
    color(index,value);
}


apa106::apa106(uint8_t pin,size_t length, uint8_t rmt_channel, uint8_t rmt_interrupt) : 
        m_pin(pin),
        m_length(length),
        m_rmt_channel(rmt_channel),
        m_rmt_interrupt(rmt_interrupt),
        m_strip(nullptr),
        m_rmt_items(nullptr) {

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
    m_rmt_items = rhs.m_rmt_items;
    rhs.m_rmt_items = nullptr;
}

size_t apa106::length() const {
    return m_length;
}
bool apa106::initialized() const {
    return m_strip!=nullptr;
}
bool apa106::initialize() {
    if(m_length==0) {
        return false;
    }
    return led_strip_initialize(m_pin,m_length,m_rmt_channel,m_rmt_interrupt,&m_strip,&m_rmt_items);
}
void apa106::deinitialize() {
    if(m_strip==nullptr) {
        return;
    }
    rmt_driver_uninstall((rmt_channel_t)m_rmt_channel);
    free(m_strip);
    free(m_rmt_items);
}
void apa106::update() {
    if(m_strip==nullptr) {
        return;
    }
    rmt_wait_tx_done((rmt_channel_t)m_rmt_channel,portMAX_DELAY);
    led_strip_fill_rmt_items_apa106(m_strip, (rmt_item32_t*)m_rmt_items, m_length);
    rmt_write_items((rmt_channel_t)m_rmt_channel, (rmt_item32_t*)m_rmt_items,  (LED_STRIP_NUM_RMT_ITEMS_PER_LED * m_length), false);
}

uint32_t apa106::color(size_t index) const {
    if(m_strip==nullptr || index<0 || index>=m_length) {
        return 0;
    }
    return m_strip[index];
}
void apa106::color(size_t index,uint32_t value) {
    if(m_strip==nullptr || index<0 || index>=m_length) {
        return;
    }
    m_strip[index]=value;
}
void apa106::color(size_t index,uint8_t r,uint8_t g, uint8_t b) {
    uint32_t value = (r<<16)|(g<<8)|b;
    color(index,value);
}