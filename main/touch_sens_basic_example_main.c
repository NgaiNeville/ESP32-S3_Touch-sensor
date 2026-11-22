/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_sens.h"
#include "esp_check.h"
#include <esp_timer.h>
#include "touch_sens_example_config.h"
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "freertos/ringbuf.h"
#include "string.h"

// Touch version 3 supports multiple sample configurations (i.e. supports frequency hopping),
// others only have one set of sample configurations.
#define EXAMPLE_TOUCH_SAMPLE_CFG_NUM        TOUCH_SAMPLE_CFG_NUM  // GPIO 01, Up to 'TOUCH_SAMPLE_CFG_NUM'
#define EXAMPLE_TOUCH_CHANNEL_NUM           4
#define EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES  3

#define LED_PIN38 GPIO_NUM_38
#define LED_PIN48 GPIO_NUM_48
#define BUTTON_GPIO_PIN     GPIO_NUM_0  // Typically the BOOT button on ESP32 DevKits
#define ESP_INTR_FLAG_DEFAULT 0

//#define SAMPLING_PERIOD_MS    20

//ring buffer config
#define SAMPLES_PER_BATCH    256             // 256 samples per ~100 ms window
#define BATCH_PERIOD_US      30000          // 100 ms or 5ms
#define SAMPLE_PERIOD_US     (BATCH_PERIOD_US / SAMPLES_PER_BATCH) // 390 us (~2.56 kHz)
#define RINGBUF_BATCH_COUNT  10              // Holds 1 second (10 batches) of data

#define SIM_CHANNEL_COUNT       3   // Increase to 3, 7, or 11 to stress-test Core 1
#define TOTAL_CHANNELS          (1 + SIM_CHANNEL_COUNT)


// Signal Processing Hyperparameters
#define ALPHA_BASE_SLOW       0.0005f           // Super slow filter for environmental drift
#define ALPHA_BASE_FAST       0.1000f           // Quicker recovery downwards if pad clears out
#define ALPHA_SIGNAL          0.1500f           // Fast and responsive touch detection filter
#define ALPHA_NOISE           0.0100f           // Low-pass variant filter for calculating noise
#define NOISE_SIGMA_THRESHOLD 5.0f              // Sensor triggers touch at 5x the RMS noise

typedef struct {
    float baseline;
    float filtered_signal;
    float noise_variance;
    float rms_noise;
    bool  is_touched;
    float raw_data;
    int64_t timestamp;
    float seq_nbr;
} measured_data_t;

static TaskHandle_t button_task_handle = NULL;
static TaskHandle_t Measurement_task_handle = NULL;
static TaskHandle_t Processing_task_handle = NULL;

touch_sensor_handle_t sens_handle = NULL;
touch_channel_handle_t chan_handle[EXAMPLE_TOUCH_CHANNEL_NUM];
uint32_t dropped_samples_count = 0;

/*typedef struct {
    int64_t seq_nbr;
    int64_t timestamp;
    int64_t data1;
}measured_data_t;*/

static RingbufHandle_t s_touch_ringbuf = NULL;
SemaphoreHandle_t xHardwaresemaphore = NULL;
QueueHandle_t xDataqueue = NULL;
QueueHandle_t xConfigQueue = NULL;
QueueHandle_t xBenchmarkQueue = NULL;
QueueHandle_t xReturnQueue = NULL;

//ESP_STATIC_ASSERT(EXAMPLE_TOUCH_SAMPLE_CFG_NUM <= TOUCH_SAMPLE_CFG_NUM, "sample configuration number exceed the supported number");
//ESP_STATIC_ASSERT(EXAMPLE_TOUCH_CHANNEL_NUM <= (TOUCH_MAX_CHAN_ID - TOUCH_MIN_CHAN_ID + 1), "touch channel number exceed the max supported number ");

static const char* TAG = "Cap_mst";
static const char* TAG1 = ".....";
#define BIT_ISR_EVENT   (1 << 0)
#define BIT_QUEUE_EVENT (1 << 1)

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

// ISR callback: strictly notifies the acquisition task every 390 us
static void IRAM_ATTR sample_timer_isr(void *arg) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(Measurement_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}



// The ISR must reside in IRAM
static void IRAM_ATTR button_gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send a notification directly to the button task
    xTaskNotifyFromISR(button_task_handle,
                       BIT_ISR_EVENT,               // eAction value (not modifying value, just unblocking)
                       eSetBits,
                       &xHigherPriorityTaskWoken);

    // If xHigherPriorityTaskWoken was set to pdTRUE, yield to the higher priority task
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Helper to synthesize realistic capacitive touch data for simulated channels
static void generate_simulated_channel(uint32_t *dst, uint8_t sim_idx, uint32_t frame_id) 
{
    uint32_t base = 22000 + (sim_idx * 1500);
    // Inject a simulated touch + 80 Hz oscillation every 20 frames (2 seconds)
    bool sim_touch_active = ((frame_id / 10) % 4 == (sim_idx % 4));
    float touch_offset = sim_touch_active ? 1800.0f : 0.0f;

    for (int i = 0; i < 256; i++) {
        float wave = sim_touch_active ? (400.0f * sinf(2.0f * M_PI * 80.0f * i / 8448)) : 0.0f;
        float noise = (float)((i * 17 + frame_id) % 40) - 20.0f;
        dst[i] = (uint32_t)(base + touch_offset + wave + noise);
    }
}

// Task handling the button event
static void button_task(void *pvParameters)
{
    uint32_t thread_notification_value;
    measured_data_t received_data22;
    uint32_t ulNotifiedValue;
    int st = 0;

    while (1) 
    {
        // Block indefinitely until EITHER the ISR or the Queue event flags are set
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);

        // Check if woken by Hardware Interrupt
        if(ulNotifiedValue & BIT_ISR_EVENT)
        {
            gpio_set_level(LED_PIN48, 0);
            ESP_LOGI(TAG, "BUTTON pressed^^^^^^^^^^^^^^^^^^^^^^^^^^!");
        }

        if (ulNotifiedValue & BIT_QUEUE_EVENT) 
        {
            while (xQueueReceive(xConfigQueue, &received_data22, 0) == pdTRUE) 
            {
                //ESP_LOGI(TAG, "BUTTON:queued packet received  %6.1f", received_data22.filtered_signal);
                    
                if((received_data22.filtered_signal > 40000)&&(st==0))
                {
                    gpio_set_level(LED_PIN48, 1);
                    ESP_LOGI(TAG, "BUTTON:$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#######@@@@@@@@!!!!");
                    st = 1;
                }
                if((received_data22.filtered_signal < 40000)&&(st==1))
                {
                    st = 0;
                }

                /*if (xReturnQueue != NULL) 
                {
                    // If the queue is full, wait up to 10 freeRTOS ticks.
                    xQueueSend(xReturnQueue, &received_data22, pdMS_TO_TICKS(10));
                    
                }*/
            }
        }
        // Block indefinitely until a notification arrives from the ISR
        /*BaseType_t xResult = xTaskNotifyWait(0x00,      // Clear no bits on entry
                                             ULONG_MAX, // Reset the notification value to 0 on exit
                                             &thread_notification_value,
                                             portMAX_DELAY);

        if (xResult == pdPASS) {
            ESP_LOGI(TAG, "Button was pressed and Task woken from ISR.$$$$$$$$#######@@@@@@@@!!!!\n");
            ESP_LOGI(TAG, "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#######@@@@@@@@!!!!");
           
            
        }

        if(xQueueReceive(xConfigQueue, &received_data22,portMAX_DELAY) == pdPASS)
        {
                if(received_data22.data1 > 40000)
                {
                    gpio_set_level(LED_PIN48, 0);
                     ESP_LOGI(TAG, "BUTTON:$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#######@@@@@@@@!!!!");
                }
        }*/
    }
}

void Measurement_task(void *pvParameters)
{
    
    int i = 0;
    uint32_t it = 0;
    uint32_t rb_overflow = 0;
    //int j = 0;
    //int seq = 1;

    
    

    uint32_t batch_buf[SAMPLES_PER_BATCH];
    uint16_t sample_idx = 0;

    //start 390us periodic timer when task handle is ready
    const esp_timer_create_args_t timer_args = {
        .callback = &sample_timer_isr,
        .name = "touch_tick_isr",
        .dispatch_method = ESP_TIMER_ISR
    };
    esp_timer_handle_t timer_handle = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, SAMPLE_PERIOD_US));

    while (1) 
    {
        //xQueueReceive(xReturnQueue, &read_data, 0);
        /*gettimeofday(&tv_now,NULL);
        int64_t time_us = ((int64_t)tv_now.tv_sec * 1000000L) + tv_now.tv_usec ;
        int64_t time_ms = ((int64_t)tv_now.tv_sec * 1000L) + (tv_now.tv_usec / 1000);*/

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t raw_val = 0;
        touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_RAW, &raw_val);
        batch_buf[sample_idx++] = raw_val;
        
        //after 256th sample, push batch into ring buffer
        if (sample_idx >= SAMPLES_PER_BATCH)
        {
            BaseType_t sent = xRingbufferSend(s_touch_ringbuf, batch_buf, sizeof(batch_buf), 0);
            if (sent != pdTRUE) {
                rb_overflow++;
                ESP_LOGW(TAG, "Ring buffer full! has dropped %lu batches. Consumer task is falling behind", rb_overflow);

            }
            //for(int i=0; i<SAMPLES_PER_BATCH; i++)
            //{
            //    it += batch_buf[i];
            //}
            sample_idx = 0;
            ESP_LOGI(TAG1,"...Ring buffer has been loaded  1st: %lu last: %lu", batch_buf[0], batch_buf[255]);
            //it = 0;
        }

    }


}

void Processing_task(void *pvParameters)
{
    measured_data_t received_data;
    int i = 0;
    int seq = 1;
    size_t item_size = sizeof(uint32_t) * SAMPLES_PER_BATCH;
    uint32_t data;
    uint32_t frame_id = 0;
    static uint32_t sim_raw_buf[SAMPLES_PER_BATCH];
    static uint32_t raw_buf[SIM_CHANNEL_COUNT][SAMPLES_PER_BATCH];

    if(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, &data))
    {
        //read_data.baseline = (float)data;
        received_data.filtered_signal = (float)data;
        received_data.noise_variance = 4.0f; // Initial guess floor (RMS noise = 2.0)
        received_data.rms_noise = sqrtf(received_data.noise_variance);

    }
    received_data.baseline = (float)data;
    received_data.filtered_signal = (float)data;
    received_data.noise_variance = 4.0f; // Initial guess floor (RMS noise = 2.0)
    received_data.rms_noise = sqrtf(received_data.noise_variance);
    received_data.is_touched = false;
    received_data.seq_nbr = seq;
    received_data.timestamp = 0;
    received_data.raw_data = data;

    for (int c = 0; c < (SIM_CHANNEL_COUNT); c++) 
    {
        generate_simulated_channel(sim_raw_buf, c, SAMPLES_PER_BATCH);
        memcpy(raw_buf[c], sim_raw_buf, sizeof(sim_raw_buf));
        
    }


    while(1)
    {
        uint32_t *batch = (uint32_t *)xRingbufferReceive(s_touch_ringbuf, &item_size, portMAX_DELAY);

        int64_t s_time = esp_timer_get_time();
        if (batch != NULL)
        {
            size_t count = item_size / sizeof(uint32_t);
            //ESP_LOGI(TAG, "[Core %d] processed %zu samples | first: %lu, Last: %lu", xPortGetCoreID(), count, batch[0], batch[count - 1]);
            for(int ts = 0; ts < 256; ts++)
            {
                data = batch[ts];
                // 2. Continual IIR Filtering for responsive touch level estimation
                received_data.filtered_signal = (ALPHA_SIGNAL * data) + ((1.0f - ALPHA_SIGNAL) * received_data.filtered_signal);

                // Deviation formula: On ESP32-S3, finger touches INCREASE raw capacitance cycles
                float deviation = (float)data - received_data.baseline;
                float touch_threshold = NOISE_SIGMA_THRESHOLD * received_data.rms_noise;

                // 3. Conditional State Machine (Sticky Baseline Logic)
                if (deviation > touch_threshold) {
        
                    // --- PAD TOUCHED ---
                received_data.is_touched = true;
            
                // TARGET CRITERIA: Completely skip updating the baseline here.
                // Because this block executes while a finger remains on the sensor, 
                // the baseline stays frozen indefinitely, avoiding absorption.
                } else 
                {
                    // --- PAD UNTOUCHED / IDLE ---
                    received_data.is_touched = false;

                    // Handle baseline drift directionally
                    if (deviation < 0.0f) {
                        // If raw values fall below baseline, track fast (environmental drop or release recalibration)
                        received_data.baseline = (ALPHA_BASE_FAST * data) + ((1.0f - ALPHA_BASE_FAST) * received_data.baseline);
                    } else 
                    {
                        // Natural environmental drift tracking
                        received_data.baseline = (ALPHA_BASE_SLOW * data) + ((1.0f - ALPHA_BASE_SLOW) * received_data.baseline);
                    }

                    // 4. Update RMS Noise estimation (Variance is only valid when untouched)
                    float current_variance = deviation * deviation;
                    received_data.noise_variance = (ALPHA_NOISE * current_variance) + ((1.0f - ALPHA_NOISE) * received_data.noise_variance);
                
                    //vTaskDelay(pdMS_TO_TICKS(1));

                    // Guard against noise collapse
                    if (received_data.noise_variance < 0.25f) received_data.noise_variance = 0.25f;
                    received_data.rms_noise = sqrtf(received_data.noise_variance);
                }
            }
                // Telemetry Data Output
            ESP_LOGI(TAG, "Raw: %4lu |Baseline: %6.1f |Fltrd: %6.1f |ClnSig:%5.1f |RMSNoise: %4.2f |Nbr: %5.1f |at: %lld |[%s]",
                data, 
                received_data.baseline, 
                received_data.filtered_signal,
                (received_data.filtered_signal - received_data.baseline), 
                received_data.rms_noise,
                received_data.seq_nbr,
                received_data.timestamp,
                received_data.is_touched ? "TOUCHED" : "RELEASED");

            if (xConfigQueue != NULL) 
                {
                    // If the queue is full, wait up to 10 freeRTOS ticks.
                    xQueueSend(xConfigQueue, &received_data, pdMS_TO_TICKS(0));
                    // Notify the worker task that data is ready in the queue
                    xTaskNotify(button_task_handle, BIT_QUEUE_EVENT, eSetBits);
                }

            
            
            for (int c = 0; c < (SIM_CHANNEL_COUNT); c++) 
            {
                //uint8_t ch_id = 1 + c;
                //generate_simulated_channel(sim_raw_buf, c, SAMPLES_PER_BATCH);
                  //process_channel_baseline(ch_id, true, sim_raw_buf, frame_id);
                  
                    
                    size_t count = item_size / sizeof(uint32_t);
                    //ESP_LOGI(TAG, "[Core %d] processed %zu samples | first: %lu, Last: %lu", xPortGetCoreID(), count, batch[0], batch[count - 1]);
                    for(int ts = 0; ts < SAMPLES_PER_BATCH; ts++)
                    {
                        data = sim_raw_buf[ts];
                        // 2. Continual IIR Filtering for responsive touch level estimation
                        received_data.filtered_signal = (ALPHA_SIGNAL * data) + ((1.0f - ALPHA_SIGNAL) * received_data.filtered_signal);

                        // Deviation formula: On ESP32-S3, finger touches INCREASE raw capacitance cycles
                        float deviation = (float)data - received_data.baseline;
                        float touch_threshold = NOISE_SIGMA_THRESHOLD * received_data.rms_noise;

                        // 3. Conditional State Machine (Sticky Baseline Logic)
                        if (deviation > touch_threshold) {
        
                            // --- PAD TOUCHED ---
                            received_data.is_touched = true;
            
                         // TARGET CRITERIA: Completely skip updating the baseline here.
                         // Because this block executes while a finger remains on the sensor, 
                         // the baseline stays frozen indefinitely, avoiding absorption.
                        } else 
                        {
                            // --- PAD UNTOUCHED / IDLE ---
                            received_data.is_touched = false;

                            // Handle baseline drift directionally
                            if (deviation < 0.0f) {
                                // If raw values fall below baseline, track fast (environmental drop or release recalibration)
                                received_data.baseline = (ALPHA_BASE_FAST * data) + ((1.0f - ALPHA_BASE_FAST) * received_data.baseline);
                            } else 
                            {
                                // Natural environmental drift tracking
                                received_data.baseline = (ALPHA_BASE_SLOW * data) + ((1.0f - ALPHA_BASE_SLOW) * received_data.baseline);
                            }

                            // 4. Update RMS Noise estimation (Variance is only valid when untouched)
                            float current_variance = deviation * deviation;
                            received_data.noise_variance = (ALPHA_NOISE * current_variance) + ((1.0f - ALPHA_NOISE) * received_data.noise_variance);
                
                            //vTaskDelay(pdMS_TO_TICKS(1));

                            // Guard against noise collapse
                            if (received_data.noise_variance < 0.25f) received_data.noise_variance = 0.25f;
                            received_data.rms_noise = sqrtf(received_data.noise_variance);
                        }
                    }
                        // Telemetry Data Output
                    ESP_LOGI(TAG1, "SIM %lu, Raw: %4lu |Baseline: %6.1f |Fltrd: %6.1f |ClnSig:%5.1f |RMSNoise: %4.2f |Nbr: %5.1f |at: %lld |[%s]",
                        (c+1),
                        data, 
                        received_data.baseline, 
                        received_data.filtered_signal,
                        (received_data.filtered_signal - received_data.baseline), 
                        received_data.rms_noise,
                        received_data.seq_nbr,
                        received_data.timestamp,
                        received_data.is_touched ? "TOUCHED" : "RELEASED");
            

            }


                //ESP_LOGI(TAG, " mst nbr %6.1f at %lldms and Value %6.1f \n", received_data.seq_nbr, received_data.timestamp, received_data.filtered_signal);
            
            
                /*if (xConfigQueue != NULL) 
                {
                    // If the queue is full, wait up to 10 freeRTOS ticks.
                    xQueueSend(xConfigQueue, &received_data, pdMS_TO_TICKS(0));
                    // Notify the worker task that data is ready in the queue
                    xTaskNotify(button_task_handle, BIT_QUEUE_EVENT, eSetBits);
                }*/
             /* if (xReturnQueue != NULL) 
                {
                    // If the queue is full, wait up to 10 freeRTOS ticks.
                    xQueueSend(xReturnQueue, &received_data, pdMS_TO_TICKS(0));
                    
                }*/
            vRingbufferReturnItem(s_touch_ringbuf, (void *)batch);
            int64_t p_time = esp_timer_get_time() - s_time ;
             ESP_LOGI(TAG1,"Ring buffer has been computed, Processing time: %llu ", p_time);
            
        }
    }
}

/*
bool example_touch_on_active_callback(touch_sensor_handle_t sens_handle, const touch_active_event_data_t *event, void *user_ctx)
{
    ESP_EARLY_LOGI("touch_callback", "[CH %d] active  @@@@@@@@@@@@@@@\n", (int)event->chan_id);
    return false;
}

bool example_touch_on_inactive_callback(touch_sensor_handle_t sens_handle, const touch_inactive_event_data_t *event, void *user_ctx)
{
    ESP_EARLY_LOGI("touch_callback", "[CH %d] inactive  ###############\n", (int)event->chan_id);
    return false;
}*/

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
        for(int j = 0; j < EXAMPLE_TOUCH_SAMPLE_CFG_NUM; j++) {
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
        uint32_t bmark = benchmark[0];
        if (xBenchmarkQueue != NULL) 
                {
                    // If the queue is full, wait up to 10 freeRTOS ticks.
                    xQueueSend(xBenchmarkQueue, &bmark, pdMS_TO_TICKS(10));
                    // Notify the worker task that data is ready in the queue
                    ESP_LOGI(TAG, " Benchmark = %ld sent to Queue \n", benchmark);
                    //if(Measurement_task_handle != NULL){
                    //xTaskNotify(Measurement_task_handle, BIT_QUEUE_EVENT, eSetBits);
                    //ESP_LOGI(TAG, " Benchmark = %ld notification sent to task \n", benchmark);
                    //}
                }
        
    }
}

void app_main(void)
{
    size_t batch_bytes = sizeof(uint32_t) * SAMPLES_PER_BATCH;
    s_touch_ringbuf = xRingbufferCreate(batch_bytes * RINGBUF_BATCH_COUNT, RINGBUF_TYPE_NOSPLIT);
    if (s_touch_ringbuf == NULL) {
        ESP_LOGE(TAG, "Failed to create ring buffer");
        return;
    }

    // Create a queue capable of holding 1 struct instance at a time
    xConfigQueue = xQueueCreate(1, sizeof(measured_data_t));

    if (xConfigQueue == NULL) {
        // Handle queue creation failure
        return;
    }

    xBenchmarkQueue = xQueueCreate(1, sizeof(uint32_t));
    if (xBenchmarkQueue == NULL ){
        return;
    }

    xReturnQueue = xQueueCreate(1, sizeof(measured_data_t));
    if (xReturnQueue == NULL) {
        return;
    }

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
    //example_touch_do_initial_scanning(sens_handle, chan_handle);

    /* Step 5: Register the touch sensor callbacks, here only take `active` and `inactive` event for example */
    /*touch_event_callbacks_t callbacks = {
        .on_active = example_touch_on_active_callback,
        .on_inactive = example_touch_on_inactive_callback,
    };
    ESP_ERROR_CHECK(touch_sensor_register_callbacks(sens_handle, &callbacks, NULL));*/

    /* Step 6: Enable the touch sensor */
    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));

    /* Step 7: Start continuous scanning, you can also trigger oneshot scanning manually */
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(sens_handle));

    xDataqueue = xQueueCreate(5, sizeof(measured_data_t));

    if(xDataqueue != NULL)
    {
        xTaskCreatePinnedToCore(
        Measurement_task,               // fxn to implement task 
        "Measurement_task",            // txt name for debugging
        4096,                          // stack size in bytes
        NULL,                          // Parameter passed into the task
        11,                             // task priority
        &Measurement_task_handle,        // task handle, NULL if not needed for tracking
        1                              // core ID (0 or 1)
        );

        xTaskCreatePinnedToCore(
        Processing_task,               // fxn to implement task 
        "Processing_task",             // txt name for debugging
        4096,                          // stack size in bytes
        NULL,                          // Parameter passed into the task
        5,                             // task priority
        &Processing_task_handle,       // task handle, NULL if not needed for tracking
        1                              // core ID (0 or 1)
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

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,       // Trigger on falling edge (press down)
        .mode = GPIO_MODE_INPUT,             // Set as input mode
        .pin_bit_mask = (1ULL << BUTTON_GPIO_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE      // Internal pull-up enabled
    };
    gpio_config(&io_conf);

    // 2. Create the processing task
    xTaskCreatePinnedToCore(
                button_task, 
                "button_task", 
                2048, 
                NULL, 
                10,              // High priority to ensure immediate context switch
                &button_task_handle,
                1
                );

    // 3. Install GPIO ISR service and register the handler
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(BUTTON_GPIO_PIN, button_gpio_isr_handler, NULL);


    
}
