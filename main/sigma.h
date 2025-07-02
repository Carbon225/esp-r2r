#ifndef SIGMA_H
#define SIGMA_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"
#include "spinlock.h"

#ifndef NUM_BITS
#error "NUM_BITS is not defined"
#endif

#ifndef FIRST_GPIO
#error "FIRST_GPIO is not defined"
#endif

#ifndef SAMPLE_RATE_HZ
#error "SAMPLE_RATE_HZ is not defined"
#endif

#ifndef OVERSAMPLING
#error "OVERSAMPLING is not defined"
#endif

#ifndef SIGMA_BUFFER_WAIT
#define SIGMA_BUFFER_WAIT 0
#endif

#ifndef SIGMA_RETAIN_GPIO
#define SIGMA_RETAIN_GPIO 0
#endif

#ifdef CONFIG_ESP32_DEFAULT_CPU_FREQ_240
#define _SIGMA_CPU_FREQ 240000000
#elif CONFIG_ESP32_DEFAULT_CPU_FREQ_160
#define _SIGMA_CPU_FREQ 160000000
#endif

#ifdef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
#error "CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 is defined"
#endif

#ifdef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
#error "CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 is defined"
#endif

#ifndef CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0
#error "CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0 is not defined"
#endif

#ifdef CONFIG_ESP_INT_WDT_CHECK_CPU1
#error "CONFIG_ESP_INT_WDT_CHECK_CPU1 is defined"
#endif

#ifndef CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0
#error "CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0 is not defined"
#endif

#ifndef CONFIG_ESP_TIMER_ISR_AFFINITY_CPU0
#error "CONFIG_ESP_TIMER_ISR_AFFINITY is not defined"
#endif

#if CONFIG_FREERTOS_TIMER_SERVICE_TASK_CORE_AFFINITY != 0
#error "CONFIG_FREERTOS_TIMER_SERVICE_TASK_CORE_AFFINITY is not 0"
#endif

#define _SIGMA_RESOLUTION (1 << NUM_BITS)
#define _SIGMA_MAX_VALUE (_SIGMA_RESOLUTION - 1)
#define _SIGMA_GPIO_MASK (_SIGMA_MAX_VALUE << FIRST_GPIO)

#define _SIGMA_SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)
#define _SIGMA_MICROSAMPLE_PERIOD_CYCLES (_SIGMA_CPU_FREQ / (SAMPLE_RATE_HZ * OVERSAMPLING))

static volatile DRAM_ATTR float __attribute__((aligned(4))) _sigma_buffer[256] = {0};
static volatile DRAM_ATTR uint8_t __attribute__((aligned(4))) _sigma_buffer_read = 0;
static volatile DRAM_ATTR uint8_t __attribute__((aligned(4))) _sigma_buffer_write = 0;

static volatile uint32_t *const _sigma_gpio_out_reg = (uint32_t *)GPIO_OUT_REG;
static volatile uint32_t *const _sigma_gpio_out_w1ts_reg = (uint32_t *)GPIO_OUT_W1TS_REG;

static IRAM_ATTR void _sigma_task(void *pvParameter)
{
    spinlock_t spinlock;
    spinlock_initialize(&spinlock);
    portENTER_CRITICAL(&spinlock);

    float sample = 0;
    unsigned int prev_tick = xthal_get_ccount();
    for (;;)
    {
        uint8_t rd = _sigma_buffer_read;
        sample = _sigma_buffer[rd];
#if SIGMA_BUFFER_WAIT
        while (rd == _sigma_buffer_write)
            ;
#endif
        _sigma_buffer_read = rd + 1;

        for (int i = 0; i < OVERSAMPLING; i++)
        {
            float sample_os = sigma_next_sample(sample);
            uint32_t sample_u32 = (uint32_t)(sample_os * (float)_SIGMA_MAX_VALUE);
            sample_u32 &= _SIGMA_MAX_VALUE;

#if SIGMA_RETAIN_GPIO
            uint32_t next_reg = (*_sigma_gpio_out_reg & (~_SIGMA_GPIO_MASK)) | (sample_u32 << FIRST_GPIO);
#else
            uint32_t next_reg = sample_u32 << FIRST_GPIO;
#endif

#ifdef CLOCK_GPIO
            *_sigma_gpio_out_w1ts_reg = 1 << CLOCK_GPIO;
#endif

            while (xthal_get_ccount() - prev_tick < _SIGMA_MICROSAMPLE_PERIOD_CYCLES)
                ;

            *_sigma_gpio_out_reg = next_reg;
            prev_tick += _SIGMA_MICROSAMPLE_PERIOD_CYCLES;
        }
    }
}

static IRAM_ATTR void sigma_write(float sample)
{
    uint8_t wr = _sigma_buffer_write;
    uint8_t next_wr = wr + 1;
    while (next_wr == _sigma_buffer_read)
        ;
    _sigma_buffer[wr] = sample;
    _sigma_buffer_write = next_wr;
}

static void sigma_init(void)
{
    gpio_config_t io_conf = {
#ifdef CLOCK_GPIO
        .pin_bit_mask = _SIGMA_GPIO_MASK | (1 << CLOCK_GPIO),
#else
        .pin_bit_mask = _SIGMA_GPIO_MASK,
#endif
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    xTaskCreatePinnedToCore(_sigma_task, "sigma", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
}

#endif
