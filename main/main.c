#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"
#include "spinlock.h"

#define CLOCK_GPIO 14
#define NUM_BITS 5
#define FIRST_GPIO 15
#define SAMPLE_RATE_HZ 50000
#define OVERSAMPLING 40

#define RESOLUTION (1 << NUM_BITS)
#define MAX_VALUE (RESOLUTION - 1)
#define LEVEL_INVERT (1.0f / ((float)MAX_VALUE))

static IRAM_ATTR float sigma_next_sample(float input_signal)
{
    static float integrator1 = 0.0f;
    static float integrator2 = 0.0f;
    static float integrator3 = 0.0f;
    static float feedback = 0.0f;

    integrator1 += input_signal - feedback;
    integrator2 += integrator1 - feedback;
    integrator3 += integrator2 - feedback;

    float quantizer = integrator3 * MAX_VALUE;
    quantizer = (float)((int)(quantizer + 0.5f)) * LEVEL_INVERT;

    feedback = quantizer;

    return quantizer;
}

#include "sigma.h"

static void sample_generator_task(void *pvParameter)
{
    const float freq = 440.0f;
    const float dt_us = 1000000.f / SAMPLE_RATE_HZ;
    const float phi_step = 2.0f * ((float)M_PI) * freq * (dt_us * 1e-6f);
    float phi = 0.0f;

    for (;;)
    {
        phi += phi_step;
        if (phi > M_PI)
        {
            phi -= 2.0f * M_PI;
        }

        float sample = sinf(phi) * 0.4f + 0.5f;

        sigma_write(sample);
    }
}

void app_main(void)
{
    sigma_init();
    xTaskCreatePinnedToCore(sample_generator_task, "generator", 4096, NULL, 5, NULL, 0);
    vTaskSuspend(NULL);
}
