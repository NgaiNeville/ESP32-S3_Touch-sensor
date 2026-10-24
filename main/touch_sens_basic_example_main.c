/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_sens.h"
#include "esp_check.h"
#include "touch_sens_example_config.h"
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

// Touch version 3 supports multiple sample configurations (i.e. supports frequency hopping),
// others only have one set of sample configurations.
#define EXAMPLE_TOUCH_SAMPLE_CFG_NUM        TOUCH_SAMPLE_CFG_NUM  // Up to 'TOUCH_SAMPLE_CFG_NUM'
#define EXAMPLE_TOUCH_CHANNEL_NUM           4
#define EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES  3

#define LED_PIN38 GPIO_NUM_38
#define LED_PIN48 GPIO_NUM_48

touch_sensor_handle_t sens_handle = NULL;
touch_channel_handle_t chan_handle[EXAMPLE_TOUCH_CHANNEL_NUM];
typedef struct {
    int64_t seq_nbr;
    int64_t timestamp;
    int64_t data1;
}measured_data_t;

SemaphoreHandle_t xHardwaresemaphore = NULL;
QueueHandle_t xDataqueue = NULL;

//ESP_STATIC_ASSERT(EXAMPLE_TOUCH_SAMPLE_CFG_NUM <= TOUCH_SAMPLE_CFG_NUM, "sample configuration number exceed the supported number");
//ESP_STATIC_ASSERT(EXAMPLE_TOUCH_CHANNEL_NUM <= (TOUCH_MAX_CHAN_ID - TOUCH_MIN_CHAN_ID + 1), "touch channel number exceed the max supported number ");

static const char* TAG = "Capacitive_mst";

// Active threshold to benchmark ratio. (i.e., touch will be activated when data >= benchmark * (1 + ratio))
static float s_thresh2bm_ratio[EXAMPLE_TOUCH_CHANNEL_NUM] = {
    [0 ... EXAMPLE_TOUCH_CHANNEL_NUM - 1] = 0.015f,  // 1.5%
};
// The touch channel IDs that used in this example
// For the corresponding GPIOs of these channel, please refer to 'touch_sensor_channel.h'
static int s_channel_id[EXAMPLE_TOUCH_CHANNEL_NUM] = {
    TOUCH_MIN_CHAN_ID,
    TOUCH_MIN_CHAN_ID + 1,
    TOUCH_MIN_CHAN_ID + 2,
    TOUCH_MIN_CHAN_ID + 3,
};

void Measurment_task(void *pvParameters)
{
    
    uint32_t data[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = {};
    struct timeval tv_now;
    int counts;
    int i = 0;
    int j = 0;
    int seq = 1;

    while (1) 
    {
        gettimeofday(&tv_now,NULL);
        int64_t time_us = ((int64_t)tv_now.tv_sec * 1000000L) + tv_now.tv_usec ;
        int64_t time_ms = ((int64_t)tv_now.tv_sec * 1000L) + (tv_now.tv_usec / 1000);
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, data));
        measured_data_t read_data;
        int64_t counts = (int64_t)data[j];
        read_data.seq_nbr = seq;
        read_data.timestamp = time_ms;
        read_data.data1 = counts;
        
        ESP_LOGI(TAG, "measurement_task: Sending measured data to queue.................\n");
        if(xQueueSend(xDataqueue, &read_data, pdMS_TO_TICKS(20)) !=pdPASS){
            ESP_LOGI(TAG, "measurement_task: Erroe: queue full, could not send*****************\n");
        
        }
        seq++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }


}

void Processing_task(void *pvParameters)
{
    measured_data_t received_data;
    while(1)
    {
        if(xQueueReceive(xDataqueue, &received_data,portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "Processing task: mst nbr %lld at %lldms and Value %lld \n", received_data.seq_nbr, received_data.timestamp, received_data.data1);
            //ESP_LOGI(TAG, "---------------------------------------------------- \n");
            if(received_data.data1 > 40000)
            {
                gpio_set_level(LED_PIN48, 1);
            }
            else
            gpio_set_level(LED_PIN48, 0);
            
        }
    }
}


bool example_touch_on_active_callback(touch_sensor_handle_t sens_handle, const touch_active_event_data_t *event, void *user_ctx)
{
    ESP_EARLY_LOGI("touch_callback", "[CH %d] active  @@@@@@@@@@@@@@@\n", (int)event->chan_id);
    return false;
}

bool example_touch_on_inactive_callback(touch_sensor_handle_t sens_handle, const touch_inactive_event_data_t *event, void *user_ctx)
{
    ESP_EARLY_LOGI("touch_callback", "[CH %d] inactive  ###############\n", (int)event->chan_id);
    return false;
}

static void example_touch_do_initial_scanning(touch_sensor_handle_t sens_handle, touch_channel_handle_t chan_handle[])
{
    /* Enable the touch sensor to do the initial scanning, so that to initialize the channel data */
    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));

    /* Scan the enabled touch channels for several times, to make sure the initial channel data is stable */
    for (int i = 0; i < EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES; i++) {
        ESP_ERROR_CHECK(touch_sensor_trigger_oneshot_scanning(sens_handle, 2000));
    }

    /* Disable the touch channel to rollback the state */
    ESP_ERROR_CHECK(touch_sensor_disable(sens_handle));

    /* (Optional) Read the initial channel benchmark and reconfig the channel active threshold accordingly */
    printf("Initial benchmark and new threshold are:\n");
    for (int i = 0; i < EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
        /* Read the initial benchmark of the touch channel */
        uint32_t benchmark[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = {};
#if SOC_TOUCH_SUPPORT_BENCHMARK
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_BENCHMARK, benchmark));
#else
        /* Read smooth data instead if the touch V1 hardware does not support benchmark */
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, benchmark));
#endif  // SOC_TOUCH_SUPPORT_BENCHMARK
        /* Calculate the proper active thresholds regarding the initial benchmark */
        printf("Touch [CH %d]", s_channel_id[i]);
        /* Generate the default channel configuration and then update the active threshold based on the real benchmark */
        touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
        for (int j = 0; j < EXAMPLE_TOUCH_SAMPLE_CFG_NUM; j++) {
#if SOC_TOUCH_SENSOR_VERSION == 1
            // Touch V1 (ESP32) uses absolute threshold.
            chan_cfg.abs_active_thresh[j] = (uint32_t)(benchmark[j] * (1 - s_thresh2bm_ratio[i]));
            printf(" %d: %"PRIu32", %"PRIu32"\t", j, benchmark[j], chan_cfg.abs_active_thresh[j]);
#else
            chan_cfg.active_thresh[j] = (uint32_t)(benchmark[j] * s_thresh2bm_ratio[i]);
            printf(" %d: %"PRIu32", %"PRIu32"\t", j, benchmark[j], chan_cfg.active_thresh[j]);
#endif  // SOC_TOUCH_SENSOR_VERSION == 1
        }
        printf("\n");
        /* Update the channel configuration */
        ESP_ERROR_CHECK(touch_sensor_reconfig_channel(chan_handle[i], &chan_cfg));
    }
}

void app_main(void)
{
    /* Handles of touch sensor */
    //touch_sensor_handle_t sens_handle = NULL;
    //touch_channel_handle_t chan_handle[EXAMPLE_TOUCH_CHANNEL_NUM];

    /* Step 1: Create a new touch sensor controller handle with default sample configuration */
    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = EXAMPLE_TOUCH_SAMPLE_CFG_DEFAULT();
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(EXAMPLE_TOUCH_SAMPLE_CFG_NUM, sample_cfg);
    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &sens_handle));

    touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
    /* Allocate new touch channel on the touch controller */
    for (int i = 0; i < EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
        ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, s_channel_id[i], &chan_cfg, &chan_handle[i]));
        /* Display the touch channel corresponding GPIO number, you can also know from `touch_sensor_channel.h` */
        touch_chan_info_t chan_info = {};
        ESP_ERROR_CHECK(touch_sensor_get_channel_info(chan_handle[i], &chan_info));
        printf("Touch [CH %d] enabled on GPIO%d\n", s_channel_id[i], chan_info.chan_gpio);
    }
    printf("=================================\n");

    /* Step 3: Confiture the default filter for the touch sensor (Note: Touch V1 uses software filter) */
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_ERROR_CHECK(touch_sensor_config_filter(sens_handle, &filter_cfg));

    /* Step 4: Do the initial scanning to initialize the touch channel data
     * Without this step, the channel data in the first read will be invalid
     */
    example_touch_do_initial_scanning(sens_handle, chan_handle);

    /* Step 5: Register the touch sensor callbacks, here only take `active` and `inactive` event for example */
    touch_event_callbacks_t callbacks = {
        .on_active = example_touch_on_active_callback,
        .on_inactive = example_touch_on_inactive_callback,
    };
    ESP_ERROR_CHECK(touch_sensor_register_callbacks(sens_handle, &callbacks, NULL));

    /* Step 6: Enable the touch sensor */
    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));

    /* Step 7: Start continuous scanning, you can also trigger oneshot scanning manually */
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(sens_handle));

    xDataqueue = xQueueCreate(5, sizeof(measured_data_t));

    if(xDataqueue != NULL)
    {
        xTaskCreatePinnedToCore(
        Measurment_task,               // fxn to implement task 
        "Measurement_task",            // txt name for debugging
        4096,                          // stack size in bytes
        NULL,                          // Parameter passed into the task
        4,                             // task priority
        NULL,                          // task handle, NULL if not needed for tracking
        1                              // core ID (0 or 1)
        );

        xTaskCreatePinnedToCore(
        Processing_task,               // fxn to implement task 
        "Measurement_task",            // txt name for debugging
        4096,                          // stack size in bytes
        NULL,                          // Parameter passed into the task
        5,                             // task priority
        NULL,                          // task handle, NULL if not needed for tracking
        0                              // core ID (0 or 1)
        ); 

    }
    else {
        ESP_LOGI(TAG, "Main: Initialization Error, failed to create Queue \n");
    }
    
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL <<LED_PIN48) | (1ULL << LED_PIN38),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);

    

    /* Step8: Print the sampled data of each enabled touch channel */
    /*uint32_t data[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = {};
    while (1) {
        int i = 0;
            ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_RAW, data));
            int j = 0 ;
            ESP_LOGI(TAG, "Measd = %d On Channel %d \n", data[j], s_channel_id[i]);
        vTaskDelay(pdMS_TO_TICKS(20));
    }*/
}
