/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_cache.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "driver/jpeg_encode.h" // WICHTIG: JPEG Header
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "example_sensor_init.h"
#include "example_config.h"
#include "esp_ldo_regulator.h"

static const char *TAG = "cam_csi";

#define NUM_FRAME_BUFFERS 2

// JPEG Buffer Größe (Schätzung: 1/5 von RGB565 sollte reichen, aber sicherheitshalber großzügig)
#define JPEG_BUFFER_SIZE (CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES * CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES / 5)

typedef struct {
    void *frame_buffers[NUM_FRAME_BUFFERS];
    size_t buffer_size;
    int current_buffer_idx;
} camera_context_t;

static camera_context_t cam_ctx = {0};
static QueueHandle_t xFrameQueue = NULL;
static jpeg_encoder_handle_t jpeg_handle = NULL; // Handle für den Encoder

typedef struct {
    void *buffer;
    size_t len;
    int buffer_idx;
} frame_event_t;

// --- Callbacks bleiben gleich ---
static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    frame_event_t evt;
    evt.buffer = trans->buffer;
    evt.len = trans->buflen;
    xQueueSendFromISR(xFrameQueue, &evt, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

static bool s_camera_get_new_vb(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    camera_context_t *ctx = (camera_context_t *)user_data;
    ctx->current_buffer_idx = (ctx->current_buffer_idx + 1) % NUM_FRAME_BUFFERS;
    trans->buffer = ctx->frame_buffers[ctx->current_buffer_idx];
    trans->buflen = ctx->buffer_size;
    return false;
}

// --- Processing Task mit JPEG Encoding ---
void frame_processing_task(void *arg) {
    frame_event_t evt;
    
    // Buffer für das fertige JPEG Bild allokieren
    uint8_t *jpeg_buffer = heap_caps_aligned_calloc(64, 1, JPEG_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (!jpeg_buffer) {
        ESP_LOGE(TAG, "Failed to alloc JPEG buffer");
        vTaskDelete(NULL);
    }

    while (1) {
        if (xQueueReceive(xFrameQueue, &evt, portMAX_DELAY) == pdTRUE) {
            
            ESP_LOGI(TAG, "Encoding Frame at %p...", evt.buffer);
            
            // 1. Config für dieses Bild
            jpeg_encode_engine_cfg_t encode_eng_cfg = {
                .intr_priority = 0,
            };

            jpeg_encode_memory_alloc_cfg_t rx_mem_cfg = {
                .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
            };

            jpeg_encode_memory_alloc_cfg_t tx_mem_cfg = {
                .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER,
            };

            // 2. Bild-Parameter setzen
            jpeg_encode_cfg_t enc_config = {
                .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                .sub_sample = JPEG_DOWN_SAMPLING_YUV422, // Standard für JPEGs
                .image_quality = 80, // Qualität 0-100
                .width = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES,
                .height = CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES,
            };

            // 3. Encoding durchführen (One-Shot Modus)
            // Der Treiber kümmert sich um DMA und Cache Sync
            
            // ÄNDERUNG: uint32_t statt size_t verwenden
            uint32_t jpeg_size = 0; 
            
            esp_err_t ret = jpeg_encoder_process(jpeg_handle, &enc_config, 
                                                 evt.buffer, evt.len, 
                                                 jpeg_buffer, JPEG_BUFFER_SIZE, 
                                                 &jpeg_size);

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "JPEG Encoded! Size: %u bytes (Ratio: 1:%.1f)", 
                         jpeg_size, (float)evt.len / jpeg_size);
                
                // HIER: jpeg_buffer enthält jetzt das fertige Bild.
                // Sie könnten es jetzt senden oder speichern.
                
            } else {
                ESP_LOGE(TAG, "JPEG Encode failed: %s", esp_err_to_name(ret));
            }
        }
    }
}

void app_main(void)
{
    esp_err_t ret = ESP_FAIL;

    //mipi ldo
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = CONFIG_EXAMPLE_USED_LDO_CHAN_ID,
        .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

    //---------------Buffer Allocation------------------//
    cam_ctx.buffer_size = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES * CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES * EXAMPLE_RGB565_BITS_PER_PIXEL / 8;
    cam_ctx.current_buffer_idx = 0;

    for (int i = 0; i < NUM_FRAME_BUFFERS; i++) {
        cam_ctx.frame_buffers[i] = heap_caps_aligned_calloc(64, 1, cam_ctx.buffer_size, MALLOC_CAP_SPIRAM);
        if (cam_ctx.frame_buffers[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
            return;
        }
        // Init to white
        memset(cam_ctx.frame_buffers[i], 0xFF, cam_ctx.buffer_size);
        esp_cache_msync((void *)cam_ctx.frame_buffers[i], cam_ctx.buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        
        ESP_LOGD(TAG, "Allocated buffer[%d]: %p", i, cam_ctx.frame_buffers[i]);
    }

    //--------Camera Sensor and SCCB Init-----------//
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    example_sensor_init(I2C_NUM_0, &i2c_bus_handle);

    //---------------CSI Init------------------//
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES,
        .v_res = CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES,
        .lane_bit_rate_mbps = EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 2,
    };
    esp_cam_ctlr_handle_t cam_handle = NULL;
    ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "csi init fail[%d]", ret);
        return;
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = s_camera_get_new_vb,
        .on_trans_finished = s_camera_get_finished_trans,
    };
    
    // WICHTIG: Wir übergeben jetzt unsere cam_ctx Struktur als user_data
    if (esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &cam_ctx) != ESP_OK) {
        ESP_LOGE(TAG, "ops register fail");
        return;
    }

    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));

    //---------------ISP Init------------------//
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES,
        .v_res = CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    // --------------------------------------------------------
    // NEU: JPEG Encoder Initialisierung
    // --------------------------------------------------------
    jpeg_encode_engine_cfg_t jpeg_eng_cfg = {
        .intr_priority = 0, // Default
        .timeout_ms = 40,   // Timeout für Encoding
    };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&jpeg_eng_cfg, &jpeg_handle));
    // --------------------------------------------------------

    xFrameQueue = xQueueCreate(5, sizeof(frame_event_t));
    xTaskCreatePinnedToCore(frame_processing_task, "frame_proc", 8192, NULL, 5, NULL, 1); // Stack erhöht!

    esp_cam_ctlr_trans_t trans_init = {
        .buffer = cam_ctx.frame_buffers[0],
        .buflen = cam_ctx.buffer_size,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_receive(cam_handle, &trans_init, 0)); 
    
    ESP_LOGI(TAG, "Starting camera...");
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam_handle));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "in main loop");
    }
}
