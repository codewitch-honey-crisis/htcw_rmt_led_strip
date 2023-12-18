#pragma once
#ifndef ESP_PLATFORM
#error "This library requires an ESP32"
#endif
#if __has_include(<Arduino.h>)
#include <Arduino.h>
namespace arduino {
#else
#include <stddef.h>
#include <inttypes.h>
namespace esp_idf {
#endif
#include <esp_idf_version.h>
class led_strip {
    virtual bool initialized() const = 0;
    virtual bool initialize() = 0;
    virtual void deinitialize() = 0;
    virtual size_t length() const = 0;
    virtual uint32_t color(size_t index) const = 0;
    virtual void color(size_t index, uint32_t color) = 0;
    virtual void color(size_t index, uint8_t red, uint8_t green, uint8_t blue) = 0;
    virtual void update() = 0;
};
class ws2812 final : public led_strip {
    uint8_t m_pin;
    size_t m_length;
    uint8_t m_rmt_channel;
    uint8_t m_rmt_interrupt;
    void* m_strip;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    void* m_rmt_items;
#else
    void* m_encoder;
    void* m_channel;
#endif
    ws2812(const ws2812& rhs)=delete;
    ws2812& operator=(const ws2812& rhs)=delete;
    void do_move(ws2812& rhs);
public:
    ws2812(uint8_t pin,size_t length, uint8_t rmt_channel=0, uint8_t rmt_interrupt=23);
    ws2812(ws2812&& rhs);
    virtual ~ws2812();
    ws2812& operator=(ws2812&& rhs);
    virtual bool initialized() const override;
    virtual bool initialize() override;
    virtual void deinitialize() override;
    virtual size_t length() const override;
    virtual uint32_t color(size_t index) const override;
    virtual void color(size_t index, uint32_t color) override;
    virtual void color(size_t index, uint8_t red, uint8_t green, uint8_t blue) override;
    virtual void update() override;
};

class sk6812 final : public led_strip {
    uint8_t m_pin;
    size_t m_length;
    uint8_t m_rmt_channel;
    uint8_t m_rmt_interrupt;
    void* m_strip;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    void* m_rmt_items;
#else
    void* m_encoder;
    void* m_channel;
#endif
    sk6812(const sk6812& rhs)=delete;
    sk6812& operator=(const sk6812& rhs)=delete;
    void do_move(sk6812& rhs);
public:
    sk6812(uint8_t pin,size_t length, uint8_t rmt_channel=0, uint8_t rmt_interrupt=23);
    sk6812(sk6812&& rhs);
    virtual ~sk6812();
    sk6812& operator=(sk6812&& rhs);
    virtual bool initialized() const override;
    virtual bool initialize() override;
    virtual void deinitialize() override;
    virtual size_t length() const override;
    virtual uint32_t color(size_t index) const override;
    virtual void color(size_t index, uint32_t color) override;
    virtual void color(size_t index, uint8_t red, uint8_t green, uint8_t blue) override;
    virtual void update() override;
};

class apa106 final : public led_strip {
    uint8_t m_pin;
    size_t m_length;
    uint8_t m_rmt_channel;
    uint8_t m_rmt_interrupt;
    void* m_strip;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    void* m_rmt_items;
#else
    void* m_encoder;
    void* m_channel;
#endif
    apa106(const apa106& rhs)=delete;
    apa106& operator=(const apa106& rhs)=delete;
    void do_move(apa106& rhs);
public:
    apa106(uint8_t pin,size_t length, uint8_t rmt_channel=0, uint8_t rmt_interrupt=23);
    apa106(apa106&& rhs);
    virtual ~apa106();
    apa106& operator=(apa106&& rhs);
    virtual bool initialized() const override;
    virtual bool initialize() override;
    virtual void deinitialize() override;
    virtual size_t length() const override;
    virtual uint32_t color(size_t index) const override;
    virtual void color(size_t index, uint32_t color) override;
    virtual void color(size_t index, uint8_t red, uint8_t green, uint8_t blue) override;
    virtual void update() override;

};

}